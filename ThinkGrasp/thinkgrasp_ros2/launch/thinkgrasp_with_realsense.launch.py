#!/usr/bin/env python3
"""
Launch file for ThinkGrasp with RealSense camera
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch arguments
    api_host_arg = DeclareLaunchArgument(
        'api_host',
        default_value='localhost',
        description='ThinkGrasp API host'
    )
    
    api_port_arg = DeclareLaunchArgument(
        'api_port',
        default_value='5010',
        description='ThinkGrasp API port'
    )
    
    # Include RealSense launch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'decimation_filter.enable': 'true',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
            'hole_filling_filter.enable': 'true',
        }.items()
    )
    
    # Include main ThinkGrasp launch
    thinkgrasp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('thinkgrasp_ros2'),
                'launch',
                'thinkgrasp.launch.py'
            ])
        ]),
        launch_arguments={
            'api_host': LaunchConfiguration('api_host'),
            'api_port': LaunchConfiguration('api_port'),
            'use_camera': 'false',  # We're launching camera separately
            'publish_markers': 'true'
        }.items()
    )
    
    return LaunchDescription([
        api_host_arg,
        api_port_arg,
        realsense_launch,
        thinkgrasp_launch
    ])