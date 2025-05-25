# ui/makearea.py
import folium, sys
from folium.plugins import Draw
import webbrowser 

def main(lat, lon, zoom=17):
    m = folium.Map(location=[lat, lon], zoom_start=zoom, tiles='OpenStreetMap')
    draw = Draw(
        export=True,
        filename='polygon.geojson',
        draw_options={'rectangle': False, 'polygon': True,
                      'circle': False, 'marker': False, 'polyline': False}
    )
    draw.add_to(m)
    output_file = "map_rect.html"
    m.save(output_file)
    print(f"► {output_file} 생성 완료.")
    webbrowser.open(output_file) 
    print(f"► {output_file}을(를) 기본 웹 브라우저에서 열었습니다.")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python makearea.py <lat> <lon> [zoom]")
        sys.exit(1)
    lat0 = float(sys.argv[1])
    lon0 = float(sys.argv[2])
    zoom = int(sys.argv[3]) if len(sys.argv) > 3 else 17
    main(lat0, lon0, zoom)

#python makearea.py 36.6284 127.4566 15 
#python makearea.py lat:=YOUR_LAT lon:=YOUR_LON
