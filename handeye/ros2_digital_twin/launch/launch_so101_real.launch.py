#!/usr/bin/env python3
"""
Launch file for SO-101 digital twin with real robot synchronization.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='/dev/ttyACM0',
        description='Serial port for robot connection'
    )
    
    simulate_arg = DeclareLaunchArgument(
        'simulate',
        default_value='false',
        description='Run in simulation mode without real robot'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2'
    )
    
    # Get package paths
    pkg_lerobot = FindPackageShare('lerobot_description')
    
    # URDF/xacro file path
    urdf_file = PathJoinSubstitution([
        pkg_lerobot,
        'urdf',
        'so101.urdf.xacro'
    ])
    
    # Process xacro to get URDF
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_file
    ])
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )
    
    # Joint state publisher for real robot
    real_robot_sync_node = Node(
        package='handeye',
        executable='sync_real_robot.py',
        name='real_robot_sync',
        output='screen',
        parameters=[{
            'robot_port': LaunchConfiguration('robot_port'),
            'simulate': LaunchConfiguration('simulate'),
            'publish_rate': 30.0
        }]
    )
    
    # RViz2 node
    rviz_config_file = os.path.join(
        os.path.dirname(__file__),
        'config',
        'so101_digital_twin.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # Stationary camera connection (if calibration exists)
    camera_connection_node = Node(
        package='handeye',
        executable='connect_camera_to_robot_stationary.py',
        name='camera_connection',
        output='screen',
        parameters=[{
            'base_frame': 'base',
            'camera_frame': 'camera_link'
        }]
    )
    
    return LaunchDescription([
        # Arguments
        robot_port_arg,
        simulate_arg,
        use_rviz_arg,
        
        # Nodes
        robot_state_publisher_node,
        real_robot_sync_node,
        camera_connection_node,
        rviz_node
    ])