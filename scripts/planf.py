#!/usr/bin/env python3
import json
import argparse
import math
import os
import webbrowser

from shapely.geometry import shape, Point, LineString, Polygon
from shapely.ops import split
from shapely.affinity import rotate as shp_rotate, translate
import pyproj
import folium

# 지구 반지름 (m) - 단순 투영 fallback 시 사용 (현재 UTM 사용으로 거의 사용 안함)
R_EARTH = 6371000

# Siyi A8 Mini Camera Specifications (for 1280x720 resolution)
SIYI_A8_SENSOR_WIDTH_MM = 7.6
SIYI_A8_SENSOR_HEIGHT_MM = 5.7
SIYI_A8_FOCAL_LENGTH_MM = 4.45
SIYI_A8_IMAGE_WIDTH_PX = 1280
SIYI_A8_IMAGE_HEIGHT_PX = 720
SIYI_A8_TARGET_PIXEL_SIZE_MM = SIYI_A8_SENSOR_WIDTH_MM / SIYI_A8_IMAGE_WIDTH_PX

# 자동 이륙 지점 생성을 위한 상수 (이제 사용 안함, GeoJSON 첫 두 점 사용)
# TAKEOFF_STANDOFF_DISTANCE_M = 5.0
# TAKEOFF_SEPARATION_DISTANCE_M = 5.0

def get_utm_transformer(lon, lat):
    """주어진 경위도에 대한 UTM 변환기를 반환합니다."""
    try:
        utm_zone = math.floor((lon + 180) / 6) + 1
        # always_xy=True ensures (lon, lat) or (x, y) order for pyproj
        # pyproj > 2.1.0 uses CRS objects
        utm_crs = pyproj.CRS.from_proj4(f"+proj=utm +zone={utm_zone} +ellps=WGS84 +units=m +no_defs{'' if lat >= 0 else ' +south'}")
        wgs84_crs = pyproj.CRS.from_epsg(4326)
        return pyproj.Transformer.from_crs(wgs84_crs, utm_crs, always_xy=True)
    except Exception as e:
        print(f"[-] Warning: Could not create UTM transformer for lon={lon}, lat={lat}: {e}.")
        return None

def wgs_to_utm(lon, lat, transformer):
    """WGS84를 UTM XY로 변환합니다."""
    if not transformer:
        raise ValueError("UTM Transformer is required for wgs_to_utm and was not provided or failed to initialize.")
    return transformer.transform(lon, lat)

def utm_to_wgs(x, y, transformer):
    """UTM XY를 WGS84로 변환합니다."""
    if not transformer:
        raise ValueError("UTM Transformer is required for utm_to_wgs and was not provided or failed to initialize.")
    return transformer.transform(x, y, direction='INVERSE')

def calculate_flight_parameters(gsd_cm_px, focal_length_mm, pixel_size_mm):
    gsd_mm_px = gsd_cm_px * 10
    altitude_mm = (gsd_mm_px * focal_length_mm) / pixel_size_mm
    return altitude_mm / 1000

def calculate_overlap_parameters(sensor_dim_mm, focal_length_mm, altitude_m, overlap_ratio):
    ground_coverage_m = (sensor_dim_mm / focal_length_mm) * altitude_m
    spacing_m = ground_coverage_m * (1 - overlap_ratio)
    return ground_coverage_m, spacing_m

def build_grid(poly_area_wgs, line_spacing_m, side_margin_m, transformer_utm, fixed_theta_deg=None):
    """지정된 폴리곤 내부에 그리드 포인트를 생성 (UTM 기반)."""
    if not poly_area_wgs.is_valid: poly_area_wgs = poly_area_wgs.buffer(0)
    if poly_area_wgs.is_empty or not poly_area_wgs.is_valid: return []

    exterior_coords_wgs = list(poly_area_wgs.exterior.coords)
    poly_xy_coords = [wgs_to_utm(lon, lat, transformer_utm) for lon, lat in exterior_coords_wgs]
    poly_xy = Polygon(poly_xy_coords)

    if not poly_xy.is_valid: poly_xy = poly_xy.buffer(0)
    if poly_xy.is_empty: return []
    
    theta_deg_to_use = fixed_theta_deg
    if theta_deg_to_use is None: # 고정 각도가 없으면 MBR에서 계산
        mbr_xy = poly_xy.minimum_rotated_rectangle
        mbr_coords_xy = list(mbr_xy.exterior.coords)
        # MBR의 첫번째 변을 기준으로 각도 계산
        dx = mbr_coords_xy[1][0] - mbr_coords_xy[0][0]
        dy = mbr_coords_xy[1][1] - mbr_coords_xy[0][1]
        theta_deg_to_use = math.degrees(math.atan2(dy, dx))

    # 폴리곤을 centroid 기준으로 -theta 회전하여 축 정렬
    poly_xy_aligned = shp_rotate(poly_xy, -theta_deg_to_use, origin=poly_xy.centroid)
    minx_a, miny_a, maxx_a, maxy_a = poly_xy_aligned.bounds

    pts_aligned_temp = [] # (x,y) 튜플 저장
    current_y = miny_a + side_margin_m # 첫 라인은 여유만큼 안쪽에서 시작
    x_left = minx_a + side_margin_m
    x_right = maxx_a - side_margin_m

    if x_left >= x_right or current_y > (maxy_a - side_margin_m):
        # print(f"[-] Warning: Grid area too small for margins/spacing in build_grid. X: {x_left:.1f} to {x_right:.1f}, Y_start: {current_y:.1f}, Y_end: {maxy_a - side_margin_m:.1f}")
        return []

    while current_y <= (maxy_a - side_margin_m): # 마지막 라인 포함하도록 <= 사용
        pts_aligned_temp.append((x_left, current_y))
        pts_aligned_temp.append((x_right, current_y))
        current_y += line_spacing_m
    
    if not pts_aligned_temp: return []

    # 정렬된 그리드 포인트를 원래 폴리곤의 centroid 기준으로 +theta 회전하여 복원
    grid_pts_xy_rotated = [shp_rotate(Point(pt_xy), theta_deg_to_use, origin=poly_xy.centroid) for pt_xy in pts_aligned_temp]
    
    # 최종 XY 좌표 (Point 객체에서 좌표 추출) 및 원본 UTM 폴리곤 내 필터링
    grid_pts_xy_final = [(pt.x, pt.y) for pt in grid_pts_xy_rotated if poly_xy.contains(pt)]

    # WGS84로 변환
    grid_pts_wgs = [utm_to_wgs(x_utm, y_utm, transformer_utm) for x_utm, y_utm in grid_pts_xy_final]
    
    return grid_pts_wgs


def export_qgc_plan(mission_waypoints, takeoff_lon, takeoff_lat, initial_takeoff_alt_m, survey_alt_m,
                    cruise_spd_mps, hover_spd_mps, output_filename, drone_id_str=""):
    if not mission_waypoints:
        # print(f"[-] {drone_id_str}No mission waypoints to export for {output_filename}.")
        return False

    grid_start_lon, grid_start_lat = mission_waypoints[0]
    grid_end_lon, grid_end_lat = mission_waypoints[-1]
    planned_home_lat, planned_home_lon, planned_home_alt = takeoff_lat, takeoff_lon, initial_takeoff_alt_m

    plan = {
        "fileType": "Plan", "version": 1, "firmwareType": 12, "groundStation": "QGroundControl", "vehicleType": 2,
        "mission": {
            "MAV_AUTOPILOT": 12, "plannedHomePosition": [planned_home_lat, planned_home_lon, float(planned_home_alt)],
            "vehicleType": 2, "firmwareType": 12, "cruiseSpeed": float(cruise_spd_mps),
            "hoverSpeed": float(hover_spd_mps), "items": []
        },
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "rallyPoints": {"points": [], "version": 2}
    }

    def mk_item(idx, cmd, p1, p2, p3, p4, lat, lon, alt):
        return {"id": idx, "type": "SimpleItem", "command": cmd, "frame": 3,
                "param1": float(p1), "param2": float(p2), "param3": float(p3), "param4": float(p4), # param4를 float으로 변환
                "coordinate": [float(lat), float(lon), float(alt)], "autoContinue": True}

    items = []
    seq = 0
    # 1. 이륙 (param4: Yaw angle. 0.0으로 설정하여 QGC 호환성 확보 시도)
    items.append(mk_item(seq, 22, 0, 0, 0, 0.0, takeoff_lat, takeoff_lon, float(initial_takeoff_alt_m))); seq += 1
    # 2. 임무 시작 지점으로 이동 (param4: Yaw angle. 0.0은 다음 웨이포인트 방향 지향)
    items.append(mk_item(seq, 16, 0, 0, 0, 0.0, grid_start_lat, grid_start_lon, float(survey_alt_m))); seq += 1
    # 3. 그리드 웨이포인트 순회
    for lon_wp, lat_wp in mission_waypoints:
        items.append(mk_item(seq, 16, 0, 0, 0, 0.0, lat_wp, lon_wp, float(survey_alt_m))); seq += 1
    # 4. 착륙 (param4: Yaw angle)
    items.append(mk_item(seq, 21, 0, 0, 0, 0.0, grid_end_lat, grid_end_lon, 0.0)); seq +=1

    plan["mission"]["items"] = items
    try:
        with open(output_filename, "w") as f: json.dump(plan, f, indent=2, ensure_ascii=False)
        print(f"[+] {drone_id_str}Saved QGC .plan -> {output_filename}")
        return True
    except IOError as e:
        print(f"[-] Error saving QGC plan file {output_filename}: {e}")
        return False


def split_polygon_into_halves(polygon_to_split_wgs, transformer_utm):
    if not polygon_to_split_wgs.is_valid: polygon_to_split_wgs = polygon_to_split_wgs.buffer(0)
    if polygon_to_split_wgs.is_empty: return []

    exterior_coords_wgs = list(polygon_to_split_wgs.exterior.coords)
    poly_xy_coords = [wgs_to_utm(lon, lat, transformer_utm) for lon, lat in exterior_coords_wgs]
    poly_xy = Polygon(poly_xy_coords)

    if not poly_xy.is_valid: poly_xy = poly_xy.buffer(0)
    if poly_xy.is_empty: return []

    mbr_xy = poly_xy.minimum_rotated_rectangle
    mbr_coords_xy = list(mbr_xy.exterior.coords)
    p0_mbr, p1_mbr, p2_mbr = mbr_coords_xy[:3]
    edge1_len = Point(p0_mbr).distance(Point(p1_mbr))
    edge2_len = Point(p1_mbr).distance(Point(p2_mbr))
    centroid_xy = mbr_xy.centroid
    min_x_mbr, min_y_mbr, max_x_mbr, max_y_mbr = mbr_xy.bounds
    # 분할선 길이 조정을 위해 MBR 대각선 길이 사용
    diag_len_mbr = math.hypot(max_x_mbr - min_x_mbr, max_y_mbr - min_y_mbr) * 1.5 # 충분히 길게


    if edge1_len >= edge2_len: # p0_mbr-p1_mbr 이 장축, 분할선은 이와 수직
        dx, dy = p1_mbr[0] - p0_mbr[0], p1_mbr[1] - p0_mbr[1] # 장축 방향 벡터
        s_p1_x, s_p1_y = centroid_xy.x - dy, centroid_xy.y + dx # 수직 방향의 한 점
        s_p2_x, s_p2_y = centroid_xy.x + dy, centroid_xy.y - dx # 수직 방향의 다른 점
    else: # p1_mbr-p2_mbr 이 장축
        dx, dy = p2_mbr[0] - p1_mbr[0], p2_mbr[1] - p1_mbr[1]
        s_p1_x, s_p1_y = centroid_xy.x - dy, centroid_xy.y + dx
        s_p2_x, s_p2_y = centroid_xy.x + dy, centroid_xy.y - dx
    
    # 분할선 벡터 및 길이 정규화/조정
    edge_s_dx, edge_s_dy = s_p2_x - s_p1_x, s_p2_y - s_p1_y
    len_s = math.hypot(edge_s_dx, edge_s_dy)
    norm_factor = diag_len_mbr / len_s if len_s > 1e-9 else 0
    
    # 중심에서 양쪽으로 뻗어나가는 분할선 좌표
    actual_p1 = (centroid_xy.x - edge_s_dx * norm_factor / 2 , centroid_xy.y - edge_s_dy * norm_factor / 2)
    actual_p2 = (centroid_xy.x + edge_s_dx * norm_factor / 2 , centroid_xy.y + edge_s_dy * norm_factor / 2)
    splitter_xy = LineString([actual_p1, actual_p2])
    
    try:
        split_geoms_xy = list(split(poly_xy, splitter_xy).geoms)
        split_polygons_xy = [geom for geom in split_geoms_xy if isinstance(geom, Polygon) and not geom.is_empty and geom.is_valid]

        if len(split_polygons_xy) == 2:
            sub_polygons_wgs = []
            for sub_poly_xy in split_polygons_xy:
                wgs_coords = [utm_to_wgs(x,y,transformer_utm) for x,y in sub_poly_xy.exterior.coords]
                sub_polygons_wgs.append(Polygon(wgs_coords))
            # 면적이 더 큰 폴리곤을 첫번째로 반환 (일관성 시도)
            sub_polygons_wgs.sort(key=lambda p: p.area, reverse=True)
            return sub_polygons_wgs
        else:
            return [polygon_to_split_wgs] # 분할 실패 시 원본 반환
    except Exception as e:
        return [polygon_to_split_wgs]


def create_mission_preview_html(main_geojson_path, drones_data_for_preview, output_html_filename, map_center_latlon):
    """Folium을 사용하여 HTML 미리보기 지도를 생성합니다."""
    m = folium.Map(location=map_center_latlon, zoom_start=17)
    try:
        folium.GeoJson(
            main_geojson_path, name="Main Survey Area",
            style_function=lambda x: {"color": "black", "weight": 2, "fillOpacity": 0.05, "dashArray": "5, 5"}
        ).add_to(m)
    except Exception as e: print(f"[-] Warning: Could not load main GeoJSON for preview: {e}")

    for drone_data in drones_data_for_preview:
        drone_id = drone_data["id_str"]
        takeoff_lat, takeoff_lon = drone_data["takeoff_point_latlon"]
        survey_area = drone_data["survey_area_wgs"]
        flight_path_lonlat = drone_data["flight_path_lonlat"] # [(lon,lat), ...]
        path_color = drone_data["color"]

        if survey_area and not survey_area.is_empty:
            folium.GeoJson(
                survey_area.__geo_interface__, name=f"{drone_id} Area",
                style_function=lambda x, color=path_color: {"fillColor": color, "color": color, "weight": 1.5, "fillOpacity": 0.2}
            ).add_to(m)

        folium.Marker(
            location=[takeoff_lat, takeoff_lon], popup=f"{drone_id} Takeoff", tooltip=f"{drone_id} Takeoff",
            icon=folium.Icon(color=path_color, icon="plane", prefix="fa") # Font Awesome 아이콘 사용
        ).add_to(m)
        
        path_for_folium = [(lat, lon) for lon, lat in flight_path_lonlat] # folium은 (lat,lon) 순서
        
        if path_for_folium:
            folium.PolyLine(
                locations=path_for_folium, color=path_color, weight=2.5, opacity=0.8, tooltip=f"{drone_id} Path"
            ).add_to(m)
            
            if len(path_for_folium) > 1: # 이륙 지점 제외, 첫번째 웨이포인트 (그리드 시작점)
                 grid_start_folium = path_for_folium[1] 
                 folium.CircleMarker(
                    location=grid_start_folium, radius=4, color=path_color, fill=True, fillColor="white",
                    tooltip=f"{drone_id} Grid Start", popup=f"<b>{drone_id}</b><br>Grid Start"
                 ).add_to(m)
            
            # 경로 끝점 (착륙 지점) 표시
            grid_end_folium = path_for_folium[-1]
            folium.CircleMarker(
                location=grid_end_folium, radius=4, color=path_color,fill=True,fillColor="black",
                tooltip=f"{drone_id} Land", popup=f"<b>{drone_id}</b><br>Land"
            ).add_to(m)

    folium.LayerControl().add_to(m)
    try:
        m.save(output_html_filename)
        print(f"[+] Mission preview saved to: {output_html_filename}")
        webbrowser.open(f"file://{os.path.realpath(output_html_filename)}")
    except Exception as e: print(f"[-] Error saving or opening HTML preview: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate QGC mission plan(s) from GeoJSON, GSD, and overlap.")
    parser.add_argument("--geojson", required=True, help="Path to GeoJSON file (Polygon).")
    parser.add_argument("--gsd", required=True, type=float, help="Target GSD (cm/pixel).")
    parser.add_argument("--forward-overlap", required=True, type=float, help="Forward overlap ratio (e.g., 0.8 for 80%).")
    parser.add_argument("--side-overlap", required=True, type=float, help="Side overlap ratio (e.g., 0.7 for 70%).")
    
    parser.add_argument("--side-margin", type=float, default=5.0, help="Side margin from polygon edges (meters). Default 5.0m.")
    parser.add_argument("--initial-takeoff-alt", type=float, default=10.0, help="Initial takeoff altitude from ground (m). Default 10.0m.")
    parser.add_argument("--cruise-spd", type=float, default=5.0, help="Cruise speed (m/s). Default 5.0m/s.")
    parser.add_argument("--hover-spd", type=float, default=2.0, help="Hover speed (m/s). Default 2.0m/s.")
    parser.add_argument("--num-drones", type=int, default=2, choices=[1, 2], help="Number of drones/areas. Default 2. (Only 1 or 2 supported for current takeoff logic).")
    parser.add_argument("--out-prefix", default="mission_drone", help="Output .plan file prefix. Default 'mission_drone'.")
    parser.add_argument("--preview-html", default="mission_preview.html", help="Output HTML preview filename. Default 'mission_preview.html'.")

    args = parser.parse_args()

    survey_alt_m = calculate_flight_parameters(args.gsd, SIYI_A8_FOCAL_LENGTH_MM, SIYI_A8_TARGET_PIXEL_SIZE_MM)
    print(f"[i] Calculated Survey Altitude: {survey_alt_m:.2f} m (for GSD {args.gsd} cm/px)")
    _, line_spacing_m = calculate_overlap_parameters(
        SIYI_A8_SENSOR_WIDTH_MM, SIYI_A8_FOCAL_LENGTH_MM, survey_alt_m, args.side_overlap)
    print(f"[i] Calculated Line Spacing (for grid): {line_spacing_m:.2f} m")

    try:
        with open(args.geojson, 'r', encoding='utf-8') as f: gj_data_for_preview = json.load(f) # 미리보기를 위해 원본 GeoJSON 데이터 로드
        main_polygon_geom_wgs = shape(gj_data_for_preview["features"][0]["geometry"])
        if not main_polygon_geom_wgs.is_valid: main_polygon_geom_wgs = main_polygon_geom_wgs.buffer(0)
        if main_polygon_geom_wgs.is_empty or len(list(main_polygon_geom_wgs.exterior.coords)) < 4:
             raise ValueError("Main polygon is empty or has too few points (minimum 3 vertices + closing point).")
    except Exception as e: print(f"[-] Error loading GeoJSON: {e}"); exit(1)

    main_centroid_wgs = main_polygon_geom_wgs.centroid
    transformer_utm = get_utm_transformer(main_centroid_wgs.x, main_centroid_wgs.y)
    if not transformer_utm:
        print("[-] Critical Error: Could not establish UTM transformer for main polygon. Exiting.")
        exit(1)

    # 이륙 지점 설정: GeoJSON의 첫 번째와 두 번째 점 사용
    takeoff_points_latlon = [] # (lat, lon) 순서로 저장
    main_coords_wgs = list(main_polygon_geom_wgs.exterior.coords) # [(lon, lat), ...]
    
    if args.num_drones >= 1:
        p0_lon, p0_lat = main_coords_wgs[0]
        takeoff_points_latlon.append((p0_lat, p0_lon))
        print(f"[i] Drone 1 Takeoff (P0 of GeoJSON): Lat={p0_lat:.6f}, Lon={p0_lon:.6f}")
    if args.num_drones == 2:
        if len(main_coords_wgs) > 1: # 최소 두 개의 점이 있어야 P1을 사용 가능
            p1_lon, p1_lat = main_coords_wgs[1]
            takeoff_points_latlon.append((p1_lat, p1_lon))
            print(f"[i] Drone 2 Takeoff (P1 of GeoJSON): Lat={p1_lat:.6f}, Lon={p1_lon:.6f}")
        else:
            print("[-] Error: Not enough points in GeoJSON for 2nd drone takeoff. Requires at least 2 distinct vertices.")
            exit(1)
    
    # 메인 폴리곤의 MBR 방향각 계산 (모든 하위 그리드에 동일하게 적용하여 패턴 일관성 시도)
    main_poly_xy_coords = [wgs_to_utm(lon, lat, transformer_utm) for lon, lat in main_coords_wgs]
    main_poly_xy = Polygon(main_poly_xy_coords)
    main_mbr_xy = main_poly_xy.minimum_rotated_rectangle
    main_mbr_coords_xy = list(main_mbr_xy.exterior.coords)
    # MBR의 첫번째 변 기준 방향각
    main_dx = main_mbr_coords_xy[1][0] - main_mbr_coords_xy[0][0]
    main_dy = main_mbr_coords_xy[1][1] - main_mbr_coords_xy[0][1]
    global_grid_theta_deg = math.degrees(math.atan2(main_dy, main_dx))
    print(f"[i] Global grid orientation angle: {global_grid_theta_deg:.2f} degrees (based on main polygon MBR)")

    sub_polygons_wgs = []
    if args.num_drones == 2 :
        temp_sub_polygons = split_polygon_into_halves(main_polygon_geom_wgs, transformer_utm)
        if len(temp_sub_polygons) == 2:
            sub_polygons_wgs = temp_sub_polygons
        else:
            print(f"[-] Warning: Splitting into 2 parts failed. Using original polygon for each drone.")
            sub_polygons_wgs = [main_polygon_geom_wgs] * args.num_drones # 임시로 각 드론에 전체 영역 할당
    elif args.num_drones == 1:
        sub_polygons_wgs = [main_polygon_geom_wgs]
    
    all_drones_preview_data = []
    drone_colors = ["blue", "red", "green", "purple"]

    for i in range(args.num_drones):
        drone_id_str = f"Drone {i+1}"
        current_sub_polygon_wgs = sub_polygons_wgs[i] if i < len(sub_polygons_wgs) else main_polygon_geom_wgs
        takeoff_lat_drone, takeoff_lon_drone = takeoff_points_latlon[i]

        print(f"\n{drone_id_str}: Processing area {i+1}...")
        
        grid_pts_wgs_lonlat = build_grid(current_sub_polygon_wgs, line_spacing_m, args.side_margin, 
                                         transformer_utm, fixed_theta_deg=global_grid_theta_deg)
        
        if not grid_pts_wgs_lonlat or len(grid_pts_wgs_lonlat) < 2:
            print(f"[-] {drone_id_str}: No valid grid points. Skipping.");
            all_drones_preview_data.append({
                "id_str": drone_id_str, "takeoff_point_latlon": (takeoff_lat_drone, takeoff_lon_drone),
                "survey_area_wgs": current_sub_polygon_wgs, "flight_path_lonlat": [], "color": drone_colors[i % len(drone_colors)]})
            continue

        rows = [grid_pts_wgs_lonlat[j:j+2] for j in range(0, len(grid_pts_wgs_lonlat), 2)]
        snake_path_wps_lonlat = [] # (lon, lat) 튜플 리스트
        for k, row in enumerate(rows): snake_path_wps_lonlat.extend(row if k % 2 == 0 else row[::-1])
        
        if not snake_path_wps_lonlat:
            print(f"[-] {drone_id_str}: Snake path empty. Skipping.");
            all_drones_preview_data.append({
                "id_str": drone_id_str, "takeoff_point_latlon": (takeoff_lat_drone, takeoff_lon_drone),
                "survey_area_wgs": current_sub_polygon_wgs, "flight_path_lonlat": [], "color": drone_colors[i % len(drone_colors)]})
            continue

        output_plan_file = f"{args.out_prefix}{i+1}.plan"
        plan_generated = export_qgc_plan(snake_path_wps_lonlat, takeoff_lon_drone, takeoff_lat_drone,
                                         args.initial_takeoff_alt, survey_alt_m, args.cruise_spd,
                                         args.hover_spd, output_plan_file, drone_id_str)
        
        if plan_generated:
            preview_flight_path_lonlat = [(takeoff_lon_drone, takeoff_lat_drone)] + snake_path_wps_lonlat
            all_drones_preview_data.append({
                "id_str": drone_id_str, "takeoff_point_latlon": (takeoff_lat_drone, takeoff_lon_drone),
                "survey_area_wgs": current_sub_polygon_wgs, "flight_path_lonlat": preview_flight_path_lonlat,
                "color": drone_colors[i % len(drone_colors)]})
        else:
             all_drones_preview_data.append({
                "id_str": drone_id_str, "takeoff_point_latlon": (takeoff_lat_drone, takeoff_lon_drone),
                "survey_area_wgs": current_sub_polygon_wgs, "flight_path_lonlat": [], "color": drone_colors[i % len(drone_colors)]})

    if all_drones_preview_data:
        map_center_latlon = (main_centroid_wgs.y, main_centroid_wgs.x) # (lat, lon)
        # 미리보기를 위해 원본 GeoJSON 파일 경로를 전달 (Folium이 직접 읽도록)
        create_mission_preview_html(args.geojson, all_drones_preview_data, args.preview_html, map_center_latlon)
    else:
        print("[-] No data available to generate HTML preview.")


        #leejihyeong@leejihyeong-960XGK:~/workspace/tlstkrh$ python3 planf.py --geojson polygon.geojson --gsd 2.0 --forward-overlap 0.8 --side-overlap 0.7 --side-margin 5 --initial-takeoff-alt 10 --cruise-spd 6 --num-drones 2 --out-prefix my_survey_
