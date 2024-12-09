import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'meca500r3.urdf.xml'
    urdf = os.path.join(
        '/home/andrek/ros2_ws/src/mecademic_description', 
        'urdf',
        urdf_file_name)
    
    # Load URDF file content as a string
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('x', default_value='-0.32', description='X position of camera relative to robot base'),
        DeclareLaunchArgument('y', default_value='0', description='Y position of camera relative to robot base'),
        DeclareLaunchArgument('z', default_value='0.55', description='Z position of camera relative to robot base'),
        DeclareLaunchArgument('roll', default_value='-1.5708', description='Roll orientation of camera'),
        DeclareLaunchArgument('pitch', default_value='0', description='Pitch orientation of camera'),
        DeclareLaunchArgument('yaw', default_value='-1.5708', description='Yaw orientation of camera'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]
        ),
        Node(
            package='meca_controller',  
            executable='test_driver',     
            name='mecademic_robot_driver',
            output='screen'
        ),
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
        ),
        # Node to launch RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/path/to/your/config.rviz']  # Optional: specify an RViz config file
        ),
        
        # Node(
        #     package='btree',
        #     executable='idle',
        #     name='idle_node',
        #     output='screen'
        # ),
    ])
