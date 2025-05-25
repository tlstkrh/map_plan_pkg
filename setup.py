from setuptools import find_packages, setup # find_packages는 그대로 두거나 제거해도 됩니다.
import os # os 모듈 임포트
from glob import glob # glob 모듈 임포트

package_name = 'map_plan_pkg'

setup(
    name=package_name,
    version='0.1.0', # 버전 업데이트 (예: 0.1.0)
    # packages=find_packages(exclude=['test']), # 이대로 두거나, 아래처럼 명시적으로 빈 리스트 사용 가능
    # 현재 map_plan_pkg/map_plan_pkg/ 내에 직접 사용하는 모듈이 없다면 아래와 같이 할 수 있습니다.
    # 만약 /home/map_plan_pkg/map_plan_pkg/__init__.py 파일이 있다면,
    # packages=[package_name] 또는 find_packages(exclude=['test'])를 사용하는 것이 좋습니다.
    packages=[package_name] if os.path.exists(os.path.join(package_name, '__init__.py')) else find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 디렉토리 내의 모든 .launch.py 파일을 설치
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        # scripts 디렉토리 내의 모든 .py 파일을 lib/패키지명/scripts 에 설치
        (os.path.join('lib', package_name, 'scripts'), glob(os.path.join('scripts', '*.py'))),
    ],
    install_requires=['setuptools'], # 필요한 최소 실행 의존성
    zip_safe=True,
    maintainer='root', # 유지보수자 이름 (변경 가능)
    maintainer_email='zkdlf2013@gmail.com', # 유지보수자 이메일
    description='A ROS 2 package to create mission areas and generate flight plans.', # 패키지 설명 업데이트
    license='Apache License 2.0', # 원하는 라이선스로 변경 (예: Apache 2.0)
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 현재 스크립트들은 ExecuteProcess로 실행되므로, 여기에 직접 등록할 필요는 없습니다.
        ],
    },
)
