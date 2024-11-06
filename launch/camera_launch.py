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
            parameters=[{'frequency': 10.0}],  # Adjust frequency if needed
        ),
        
        # Node for cam_detector
        Node(
            package='camera',   # Replace with the actual package name
            executable='cam_detector',
            name='cam_detector_node',
            output='screen'
        ),
    ])
