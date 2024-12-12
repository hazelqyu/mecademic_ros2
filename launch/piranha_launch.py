from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from pathlib import Path

def generate_launch_description():
    # Define paths to each launch file
    current_folder = Path(__file__).parent
    btree_launch_path = str(current_folder / 'btree_launch.py')
    face_detection_launch_path = str(current_folder / 'face_detection_launch.py')
    robot_state_launch_path = str(current_folder / 'robot_state_launch.py')

    # Include each launch file
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(btree_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(face_detection_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_launch_path)
        ),
    ])
