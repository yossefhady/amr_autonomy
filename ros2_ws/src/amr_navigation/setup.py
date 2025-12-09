from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yossef',
    maintainer_email='yossefhady53@gmail.com',
    description='Navigation package for AMR with Nav2 and AMCL localization for Mecanum drive robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_override = amr_navigation.teleop_override:main',
        ],
    },
)
