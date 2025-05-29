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
        ('share/' + package_name + '/launch', ['launch/dwa_planner_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JONGHAKKIM',
    maintainer_email='dwefresh1@inha.edu',
    description='DWA Planner in Python for ROS 2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'dwa_planner_node = dwa_planner_ros2.dwa_planner_node:main',
        ],
    },
)
