import os
import xacro

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals

from launch.actions import AppendEnvironmentVariable

def generate_launch_description():
    is_visual = LaunchConfiguration('is_visual')
    is_simulation = LaunchConfiguration('is_simulation')
    # VISUALIZE_DESCRIPTION = True
    is_visual_launch_arg = DeclareLaunchArgument(
        'is_visual',
        default_value='False'
    )

    # Get package paths
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    leg_gz_ros_bringup_pkg_path = get_package_share_directory('leg_gz_ros_bringup')
    leg_gazebo_pkg_path = get_package_share_directory('leg_gazebo')
    leg_description_pkg_path = get_package_share_directory('leg_description')
    leg_description_pkg_root_path = '/home/daniel/Workspace/ROS/leg_ws/install/leg_description/share'

    # Set environment variables
    set_env_vars_resources = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', leg_description_pkg_root_path)

    # Gazebo server cmd 
    builtin_gz_launch_path = os.path.join(ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py')
    worlds_file_path = os.path.join(leg_gazebo_pkg_path, 'worlds', 'leg.sdf')
    ign_gazebo_args = [
        '-v4 ',
        worlds_file_path
    ]

    gz_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(builtin_gz_launch_path),
        launch_arguments={'gz_args': ign_gazebo_args, 'on_exit_shutdown': 'true'}.items(),
        condition=LaunchConfigurationNotEquals('is_visual', 'True')
    )

    # Robot state publisher
    xacro_file_path = os.path.join(leg_description_pkg_path, 'urdf', 'leg.urdf')
    robot_description_raw = xacro.process_file(xacro_file_path).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_raw,
                'use_sim_time': True
            }
        ]
    )
    
    # Run rviz 
    # rviz_config_path = os.path.join(leg_gz_ros_bringup_pkg_path, 'config', 'biped_hopper.rviz')
    run_rviz = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(leg_gz_ros_bringup_pkg_path, 'config', 'bridge.yaml'),
            # 'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen',
        condition=LaunchConfigurationNotEquals('is_visual', 'True')
    )     

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=LaunchConfigurationEquals('is_visual', 'True')
    ) 

    # Create launch description
    command_sequences = [
        is_visual_launch_arg,
        set_env_vars_resources,
        gz_simulation,
        bridge,
        joint_state_publisher,
        robot_state_publisher,
        run_rviz,
    ]

    launch_description = LaunchDescription(command_sequences)
    return launch_description