from setuptools import find_packages, setup
import os 
from glob import glob 

package_name = 'map_plan_pkg'

setup(
    name=package_name,
    version='0.1.0',

    packages=[package_name] if os.path.exists(os.path.join(package_name, '__init__.py')) else find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('lib', package_name, 'scripts'), glob(os.path.join('scripts', '*.py'))),
    ],
    install_requires=['setuptools'], 
    zip_safe=True,
    maintainer='root', 
    maintainer_email='zkdlf2013@gmail.com',
    description='A ROS 2 package to create mission areas and generate flight plans.', 
    license='Apache License 2.0', 
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
