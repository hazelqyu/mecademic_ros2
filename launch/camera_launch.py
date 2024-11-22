# camera_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        
        # Node for cam2image
        Node(
            package='image_tools',
            executable='cam2image',
            name='cam2image_node',
            output='screen',
            parameters=[
                {'frequency': 10.0},    # Adjust frequency if needed
                {'width': 640},         
                {'height': 480}         
            ],
        ),
        
        # Node for cam_detector
        Node(
            package='camera',   # Replace with the actual package name
            executable='face_detector',
            name='face_detector_node',
            output='screen'
        ),
        # Node(
        #     package='btree',
        #     executable='idle',
        #     name='idle_node',
        #     output='screen'
        # ),
    ])
