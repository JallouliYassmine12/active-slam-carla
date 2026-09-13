from setuptools import setup
import os
from glob import glob

package_name = 'slam_evaluation'

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
        # Inclure les scripts pour qu'ils soient installés
        (os.path.join('lib', package_name),
            glob('slam_evaluation/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='SLAM Evaluation Package with Safety',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Déclarer les scripts comme exécutables
            'localization_evaluator = slam_evaluation.localization_evaluator:main',
            'vehicle_controller_eval = slam_evaluation.vehicle_controller_eval:main',
        ],
    },
)
