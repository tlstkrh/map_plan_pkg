import os
import sys # sys 모듈 임포트
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지의 공유 디렉토리 경로를 가져옵니다.
    pkg_share_dir = get_package_share_directory('map_plan_pkg')

    # 설치된 스크립트가 위치할 예상 경로를 구성합니다.
    # setup.py에서 lib/<package_name>/scripts 로 설치하도록 설정했으므로,
    # <install_dir>/<package_name>/lib/<package_name>/scripts/ 가 됩니다.
    scripts_install_dir = os.path.abspath(
        os.path.join(pkg_share_dir, '..', '..', 'lib', 'map_plan_pkg', 'scripts')
    )
    makearea_script_path = os.path.join(scripts_install_dir, 'makearea.py')

    # 현재 사용 중인 Python 인터프리터의 경로를 가져옵니다.
    python_executable = sys.executable

    # Launch Arguments 선언
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
    # makearea.py는 현재 'polygon.geojson'으로 파일명을 고정하고 있으므로,
    # 별도의 output_geojson_name_arg는 스크립트 자체에서 사용되지 않습니다.

    # 사용자 안내 메시지
    # LogInfo 액션을 사용하여 터미널에 메시지를 출력할 수 있습니다.
    inform_user_message = LogInfo(
        msg="\n\n>>> 웹 브라우저가 열리면 폴리곤을 그리고 'Export' 버튼을 눌러 'polygon.geojson' 파일을 저장하세요. <<<\n"
            ">>> 이 파일은 `ros2 launch` 명령어를 실행한 현재 작업 디렉토리에 저장됩니다. <<<\n"
            ">>> 저장 후 이 터미널이나 웹 브라우저를 닫고 다음 planf_launch.py를 실행하세요. <<<\n\n"
    )

    # makearea.py 스크립트 실행 설정
    makearea_process = ExecuteProcess(
        cmd=[python_executable, makearea_script_path,
             LaunchConfiguration('lat'),
             LaunchConfiguration('lon'),
             LaunchConfiguration('zoom')],
        # cwd='.', # 기본적으로 ros2 launch를 실행한 디렉토리에서 실행됩니다.
        output='screen',
        shell=False # 보안 및 예측 가능성을 위해 False 권장
    )

    return LaunchDescription([
        lat_arg,
        lon_arg,
        zoom_arg,
        inform_user_message,
        makearea_process
    ])
