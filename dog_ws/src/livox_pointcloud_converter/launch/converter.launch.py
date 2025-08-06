from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/livox/lidar',
            description='Input Livox CustomMsg topic'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/livox/pointcloud2',
            description='Output PointCloud2 topic'
        ),
        DeclareLaunchArgument(
            'output_frame_id',
            default_value='livox_frame',
            description='Output frame ID for PointCloud2'
        ),
        DeclareLaunchArgument(
            'use_intensity',
            default_value='true',
            description='Include intensity information'
        ),
        DeclareLaunchArgument(
            'use_tag',
            default_value='false',
            description='Include tag information'
        ),
        DeclareLaunchArgument(
            'use_line',
            default_value='false',
            description='Include line information'
        ),
        
        Node(
            package='livox_pointcloud_converter',
            executable='livox_to_pointcloud2_node',
            name='livox_to_pointcloud2_converter',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'output_frame_id': LaunchConfiguration('output_frame_id'),
                'use_intensity': LaunchConfiguration('use_intensity'),
                'use_tag': LaunchConfiguration('use_tag'),
                'use_line': LaunchConfiguration('use_line'),
            }],
            output='screen'
        )
    ])