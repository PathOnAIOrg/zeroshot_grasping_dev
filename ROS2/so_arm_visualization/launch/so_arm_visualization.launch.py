#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Complete SO-ARM101 robot visualization launch file.
    
    This launch file provides:
    - Real hardware joint state reader (SO-ARM101 robot)
    - Robot state publisher (URDF processing + TF publishing)
    - RViz2 visualization with pre-configured display
    
    Usage:
    ros2 launch so_arm_visualization so_arm_visualization.launch.py
    """
    
    # Get package directories  
    package_dir = get_package_share_directory("so_arm_visualization")
    
    # Launch arguments
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(package_dir, "urdf", "so101.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation time"
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=os.path.join(package_dir, "rviz", "display.rviz"),
        description="Path to RViz configuration file"
    )
    
    # Process URDF
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), 
        value_type=str
    )
    
    # Nodes
    
    # 1. SO-ARM101 Hardware Driver - reads real joint positions
    joint_state_reader_node = Node(
        package="so_arm_visualization",
        executable="so_arm_joint_reader",
        name="so_arm_joint_reader",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
        remappings=[
            ("/joint_states", "/joint_states")
        ]
    )
    
    # 2. Robot State Publisher - converts URDF + joint states to TF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }
        ]
    )
    
    # 3. RViz2 Visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )
    
    # Optional: Joint State Publisher GUI (for manual control when hardware is not connected)
    # Uncomment the lines below to enable GUI control instead of hardware
    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="joint_state_publisher_gui"
    # )
    
    return LaunchDescription([
        # Launch arguments
        model_arg,
        use_sim_time_arg,
        rviz_config_arg,
        
        # Information
        LogInfo(msg="🚀 Starting SO-ARM101 Robot Visualization"),
        LogInfo(msg="📡 Hardware driver will connect to /dev/ttyACM0"),
        LogInfo(msg="🎯 RViz2 will display real-time robot pose"),
        
        # Launch nodes
        joint_state_reader_node,
        robot_state_publisher_node,
        rviz_node
    ])