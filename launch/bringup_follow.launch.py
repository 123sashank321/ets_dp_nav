#!/usr/bin/env python3
"""
Launch file for Drone Dynamic Object Following System

Launches:
1. Target Generator - Creates moving circular target
2. Nav2 to PX4 Bridge - Converts Nav2 commands to PX4 setpoints
3. Nav2 Stack - Path planning and following
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for the complete system."""
    
    # Package directories
    pkg_share = get_package_share_directory('ets_dp_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Paths to configuration files
    bt_xml_path = os.path.join(pkg_share, 'config', 'follow_point_bt.xml')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file for Nav2'
    )
    
    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=bt_xml_path,
        description='Full path to the behavior tree XML file'
    )
    
    # Target Generator Node
    target_generator_node = Node(
        package='ets_dp_nav',
        executable='target_generator',
        name='target_generator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        emulate_tty=True
    )
    
    # Nav2 to PX4 Bridge Node
    nav2_px4_bridge_node = Node(
        package='ets_dp_nav',
        executable='nav2_to_px4_bridge',
        name='nav2_to_px4_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        emulate_tty=True
    )
    
    # PX4 TF Publisher Node (publishes map->odom->base_link transforms)
    px4_tf_publisher_node = Node(
        package='ets_dp_nav',
        executable='px4_tf_publisher',
        name='px4_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        emulate_tty=True
    )
    
    # Goal Relay Node (relays /goal_update to Nav2 action server)
    goal_relay_node = Node(
        package='ets_dp_nav',
        executable='goal_relay',
        name='goal_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        emulate_tty=True
    )
    
    # Nav2 Navigation Stack (without localization/SLAM - uses odometry directly)
    # This is better for drone dynamic following where we don't need a pre-built map
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'default_bt_xml_filename': bt_xml_file,
            'autostart': 'true',
        }.items()
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    
    # Add nodes
    ld.add_action(target_generator_node)
    ld.add_action(nav2_px4_bridge_node)
    ld.add_action(px4_tf_publisher_node)
    ld.add_action(goal_relay_node)
    ld.add_action(nav2_bringup)
    
    return ld
