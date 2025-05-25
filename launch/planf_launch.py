import os
import sys 
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('map_plan_pkg')
    scripts_install_dir = os.path.abspath(
        os.path.join(pkg_share_dir, '..', '..', 'lib', 'map_plan_pkg', 'scripts')
    )
    planf_script_path = os.path.join(scripts_install_dir, 'planf.py')

    python_executable = sys.executable


    geojson_file_arg = DeclareLaunchArgument(
        'geojson_file',
        default_value='polygon.geojson', 
        description='Path to the GeoJSON file (must exist in CWD or be an absolute path)'
    )
    gsd_arg = DeclareLaunchArgument(
        'gsd',
        default_value='2.0', 
        description='Target GSD (cm/pixel)'
    )
    forward_overlap_arg = DeclareLaunchArgument(
        'forward_overlap',
        default_value='0.8',
        description='Forward overlap ratio (e.g., 0.8 for 80%)'
    )
    side_overlap_arg = DeclareLaunchArgument(
        'side_overlap',
        default_value='0.7', 
        description='Side overlap ratio (e.g., 0.7 for 70%)'
    )
    side_margin_arg = DeclareLaunchArgument(
        'side_margin',
        default_value='5.0', 
        description='Side margin from polygon edges (meters)'
    )
    initial_takeoff_alt_arg = DeclareLaunchArgument(
        'initial_takeoff_alt',
        default_value='10.0', 
        description='Initial takeoff altitude from ground (m)'
    )
    cruise_spd_arg = DeclareLaunchArgument(
        'cruise_spd',
        default_value='5.0', 
        description='Cruise speed (m/s)'
    )
    hover_spd_arg = DeclareLaunchArgument(
        'hover_spd',
        default_value='2.0', 
        description='Hover speed (m/s)'
    )
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='1',
        description='Number of drones/areas to split into (1 or 2 supported by planf.py)'
    )
    out_prefix_arg = DeclareLaunchArgument(
        'out_prefix',
        default_value='mission_drone_', 
        description='Output .plan file prefix'
    )
    preview_html_arg = DeclareLaunchArgument(
        'preview_html',
        default_value='mission_preview.html', 
        description='Output HTML preview filename'
    )

    planf_process = ExecuteProcess(
        cmd=[
            python_executable, planf_script_path,
            '--geojson', LaunchConfiguration('geojson_file'),
            '--gsd', LaunchConfiguration('gsd'),
            '--forward-overlap', LaunchConfiguration('forward_overlap'),
            '--side-overlap', LaunchConfiguration('side_overlap'),
            '--side-margin', LaunchConfiguration('side_margin'),
            '--initial-takeoff-alt', LaunchConfiguration('initial_takeoff_alt'),
            '--cruise-spd', LaunchConfiguration('cruise_spd'),
            '--hover-spd', LaunchConfiguration('hover_spd'),
            '--num-drones', LaunchConfiguration('num_drones'),
            '--out-prefix', LaunchConfiguration('out_prefix'),
            '--preview-html', LaunchConfiguration('preview_html')
        ],
        # cwd='.', 
        output='screen',
        shell=False
    )

    return LaunchDescription([
        geojson_file_arg,
        gsd_arg,
        forward_overlap_arg,
        side_overlap_arg,
        side_margin_arg,
        initial_takeoff_alt_arg,
        cruise_spd_arg,
        hover_spd_arg,
        num_drones_arg,
        out_prefix_arg,
        preview_html_arg,
        planf_process
    ])
