import os
import xacro

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch.actions import AppendEnvironmentVariable

def generate_launch_description():
    leg_controller_pkg_path = get_package_share_directory('leg_controller')
    parameters_file_path = os.path.join(leg_controller_pkg_path, 'config', 'parameters.yaml')

    low_level_control = Node(
        package='leg_controller',
        executable='low_level_control_node',
        output='screen',
        parameters= [parameters_file_path]
    )     

    feed_forward_control = Node(
        package='leg_controller',
        executable='feed_forward_control_node',
        output='screen',
        parameters= [parameters_file_path]
    )

    # Create launch description
    command_sequences = [
        # feed_forward_control,
        low_level_control
    ]
    launch_description = LaunchDescription(command_sequences)
    return launch_description