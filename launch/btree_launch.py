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
        ),
        # List all the tree nodes
        Node(
            package='btree',
            executable='track_face',
            name='track_face_node',
            output='screen'
        ),
        Node(
            package='btree',
            executable='idle',
            name='idle_node',
            output='screen'
        ),
        Node(
            package='btree',
            executable='asleep',
            name='asleep_node',
            output='screen'
        ),
    ])
