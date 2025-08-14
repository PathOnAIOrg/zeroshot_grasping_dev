#!/usr/bin/env python3
"""
Launch file for ThinkGrasp ROS2 system
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
    
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Launch RealSense camera driver'
    )
    
    publish_markers_arg = DeclareLaunchArgument(
        'publish_markers',
        default_value='true',
        description='Publish visualization markers'
    )
    
    # Get launch configurations
    api_host = LaunchConfiguration('api_host')
    api_port = LaunchConfiguration('api_port')
    use_camera = LaunchConfiguration('use_camera')
    publish_markers = LaunchConfiguration('publish_markers')
    
    # ThinkGrasp service node
    thinkgrasp_service_node = Node(
        package='thinkgrasp_ros2',
        executable='thinkgrasp_service_node.py',
        name='thinkgrasp_service',
        output='screen',
        parameters=[{
            'api_host': api_host,
            'api_port': api_port,
            'publish_markers': publish_markers,
            'marker_lifetime': 10.0,
            'temp_dir': '/tmp/thinkgrasp'
        }],
        remappings=[
            ('/detected_grasps', '/thinkgrasp/grasps'),
            ('/grasp_markers', '/thinkgrasp/markers'),
            ('/grasp_point_cloud', '/thinkgrasp/point_cloud')
        ]
    )
    
    # Image capture node
    image_capture_node = Node(
        package='thinkgrasp_ros2',
        executable='image_capture_node.py',
        name='image_capture',
        output='screen',
        parameters=[{
            'rgb_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'capture_dir': '/tmp/thinkgrasp/captures',
            'auto_capture_rate': 0.0
        }]
    )
    
    # Grasp visualizer node (optional)
    grasp_visualizer_node = Node(
        package='thinkgrasp_ros2',
        executable='grasp_visualizer_node.py',
        name='grasp_visualizer',
        output='screen',
        parameters=[{
            'update_rate': 30.0
        }]
    )
    
    # Create launch description
    ld = LaunchDescription([
        # Arguments
        api_host_arg,
        api_port_arg,
        use_camera_arg,
        publish_markers_arg,
        
        # Nodes
        thinkgrasp_service_node,
        image_capture_node,
        grasp_visualizer_node,
    ])
    
    return ld