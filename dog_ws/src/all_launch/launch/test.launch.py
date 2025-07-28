import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch_ros
from dash import Output
from traitlets import default
import launch
import launch.launch_description_sources
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    map_file = os.path.join('~', '/home/hzy/study/ros2/projects/lio_sam_ws/lio_sam_demo/data/pgm/out.yaml')
    mapserver_node = launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False}, 
                        {'yaml_filename':map_file}]
        )
    lifecycle_node = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': False},
                        {'node_names': ['map_server']}]  
        )


    default_model_path = os.path.join('~', '/home/hzy/study/ros2/projects/lio_sam_ws/lio_sam_demo/hzy_ws/src/LIO-SAM-MID360-ROS2/config/robot_test.urdf.xacro')
    print(f"Default model path: {default_model_path}")

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher'
        # parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    start_action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=0.5, actions=[mapserver_node]),
        launch.actions.TimerAction(period=1.0, actions=[lifecycle_node]),
        launch.actions.TimerAction(period=2.0, actions=[robot_state_publisher_node]),
        launch.actions.TimerAction(period=2.5, actions=[joint_state_publisher_node]),
    ])

    return launch.LaunchDescription([
        # launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path, 
 		# 					description='Absolute path to robot urdf file'),
        start_action_group
    ])


