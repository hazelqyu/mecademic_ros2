from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='m_behavior_tree',
            executable='bt_executor',
            name='bt_executor',
            output='screen',
            parameters=[]
        )
    ])
