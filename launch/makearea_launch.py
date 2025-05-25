import os
import sys 
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('map_plan_pkg')
    scripts_install_dir = os.path.abspath(
        os.path.join(pkg_share_dir, '..', '..', 'lib', 'map_plan_pkg', 'scripts')
    )
    makearea_script_path = os.path.join(scripts_install_dir, 'makearea.py')
    python_executable = sys.executable
    lat_arg = DeclareLaunchArgument(
        'lat',
        default_value='36.6284',
        description='Initial latitude for makearea.py'
    )
    lon_arg = DeclareLaunchArgument(
        'lon',
        default_value='127.4566',
        description='Initial longitude for makearea.py'
    )
    zoom_arg = DeclareLaunchArgument(
        'zoom',
        default_value='17',
        description='Initial zoom level for makearea.py'
    )
    
    inform_user_message = LogInfo(
        msg="\n\n>>> 웹 브라우저가 열리면 폴리곤을 그리고 'Export' 버튼을 눌러 'polygon.geojson' 파일을 저장하세요. <<<\n"
            ">>> 이 파일은 `ros2 launch` 명령어를 실행한 현재 작업 디렉토리에 저장됩니다. <<<\n"
            ">>> 저장 후 이 터미널이나 웹 브라우저를 닫고 다음 planf_launch.py를 실행하세요. <<<\n\n"
    )


    makearea_process = ExecuteProcess(
        cmd=[python_executable, makearea_script_path,
             LaunchConfiguration('lat'),
             LaunchConfiguration('lon'),
             LaunchConfiguration('zoom')],

        output='screen',
        shell=False 
    )

    return LaunchDescription([
        lat_arg,
        lon_arg,
        zoom_arg,
        inform_user_message,
        makearea_process
    ])
