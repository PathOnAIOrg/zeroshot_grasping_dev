#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the virtualenv python executable for graspnet
    python_bin = '/home/pathonai/ros2_ws/src/graspnet_ros/scripts/venv_graspnet312/bin/python'
    
    return LaunchDescription([
        # Launch RealSense camera node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('realsense2_camera'), 
                '/launch/rs_launch.py'
            ]),
            launch_arguments={
                'pointcloud.enable': 'true',
                'align_depth.enable': 'true',
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30'
            }.items()
        ),
        
        # Launch Graspnet node with remapped topic
        Node(
            package='graspnet_ros',
            executable='graspnet_node.py',
            name='graspnet_node',
            output='screen',
            prefix=python_bin,
            remappings=[
                ('/pc_octomap_filtered', '/camera/camera/depth/color/points')
            ]
        )
    ])