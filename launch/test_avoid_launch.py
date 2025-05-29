from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dwa_planner_ros2',
            executable='fake_sensor_publisher',
            name='fake_sensor',
            output='screen'
        ),
        Node(
            package='dwa_planner_ros2',
            executable='fake_goal_publisher',
            name='fake_goal',
            output='screen'
        ),
        Node(
            package='dwa_planner_ros2',
            executable='dwa_planner_node',
            name='dwa_planner',
            output='screen',
            parameters=['config/params.yaml'],
        ),
    ])
