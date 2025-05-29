from setuptools import setup

package_name = 'dwa_planner_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/dwa_planner_launch.py',
            'launch/test_evasion_launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@your.org',
    description='Full-port of AMSL DWA Planner to Python/ROS2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dwa_planner_node = dwa_planner_ros2.dwa_planner_node:main',
            'fake_sensor_publisher = dwa_planner_ros2.fake_sensor_publisher:main',
            'fake_goal_publisher = dwa_planner_ros2.fake_goal_publisher:main',
        ],
    },
)
