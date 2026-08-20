from setuptools import setup
import os
from glob import glob

package_name = 'ld_slam_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Copy toàn bộ file trong các thư mục launch, config, rviz
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='naki',
    description='SLAM and Navigation launch for LD14 and IMU',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        	'pose_logger = ld_slam_py.pose_logger:main',
        	'plot_trajectory = ld_slam_py.plot_trajectory:main',
        	'path_visualizer = ld_slam_py.path_visualizer:main',
        ],
    },
)
