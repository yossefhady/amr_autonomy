from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps/small_warehouse'), glob('maps/small_warehouse/*')),
        (os.path.join('share', package_name, 'maps/medium_warehouse'), glob('maps/medium_warehouse/*')),
        (os.path.join('share', package_name, 'maps/large_warehouse'), glob('maps/large_warehouse/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yossef',
    maintainer_email='yossefhady53@gmail.com',
    description='SLAM package for AMR robot using slam_toolbox',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'save_map = amr_slam.save_map:main',
        ],
    },
)
