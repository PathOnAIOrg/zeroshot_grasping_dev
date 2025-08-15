from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'camera_topic_prefix',
            default_value='/camera',
            description='Prefix for camera topics'
        ),
        
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='rgb/image_raw',
            description='RGB image topic name'
        ),
        
        DeclareLaunchArgument(
            'depth_topic',
            default_value='depth/image_raw',
            description='Depth image topic name'
        ),
        
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='rgb/camera_info',
            description='Camera info topic name'
        ),
        
        # Object detection node
        Node(
            package='lerobot_perception',
            executable='object_detection_node',
            name='object_detection_node',
            output='screen',
            parameters=[{
                'camera_topic_prefix': LaunchConfiguration('camera_topic_prefix'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'min_area': 1000,
                'max_area': 50000,
            }],
            remappings=[
                ('/camera/color/image_raw', 
                 [LaunchConfiguration('camera_topic_prefix'), '/', LaunchConfiguration('rgb_topic')]),
                ('/camera/depth/image_raw', 
                 [LaunchConfiguration('camera_topic_prefix'), '/', LaunchConfiguration('depth_topic')]),
                ('/camera/camera_info', 
                 [LaunchConfiguration('camera_topic_prefix'), '/', LaunchConfiguration('camera_info_topic')]),
            ]
        ),
    ])

