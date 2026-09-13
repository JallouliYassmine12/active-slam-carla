from setuptools import find_packages, setup

package_name = 'active_slam_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'state_estimator = active_slam_core.state_estimator:main',
                'destination_generator = active_slam_core.destination_generator:main',
                'active_slam_decision = active_slam_core.active_slam_decision:main',
        ],
    },
)
