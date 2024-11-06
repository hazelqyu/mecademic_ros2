# camera_tf.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Define launch arguments for easy adjustment
        DeclareLaunchArgument('x', default_value='0.0', description='X position of camera relative to robot base'),
        DeclareLaunchArgument('y', default_value='0.0', description='Y position of camera relative to robot base'),
        DeclareLaunchArgument('z', default_value='0.0', description='Z position of camera relative to robot base'),
        DeclareLaunchArgument('roll', default_value='-1.5708', description='Roll orientation of camera'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='Pitch orientation of camera'),
        DeclareLaunchArgument('yaw', default_value='-1.5708', description='Yaw orientation of camera'),

        # Start the static_transform_publisher with these parameters
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                LaunchConfiguration('x'),
                LaunchConfiguration('y'),
                LaunchConfiguration('z'),
                LaunchConfiguration('roll'),
                LaunchConfiguration('pitch'),
                LaunchConfiguration('yaw'),
                'world',  # Parent frame ID (robot base)
                'camera_frame'       # Child frame ID (camera)
            ]
        )
    ])
