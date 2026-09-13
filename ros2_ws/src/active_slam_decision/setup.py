from setuptools import setup
import os
from glob import glob

package_name = 'active_slam_decision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description="Fonction de decision multi-criteres pour l'Active SLAM (ROS 2 + CARLA)",
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'candidate_generator = active_slam_decision.candidate_generator:main',
            'decision_maker = active_slam_decision.decision_maker:main',
            'vehicle_controller_active_slam = active_slam_decision.vehicle_controller_active_slam:main',
            'obstacle_detector = active_slam_decision.obstacle_detector:main',
            'traffic_spawner = active_slam_decision.traffic_spawner:main',
        ],
    },
)
