from dash import Output
import launch
import launch.launch_description_sources
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Launch
    # bag record 数据录制
    bag_record_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory('bag_record') + '/launch/bag.launch.py'
        )
    )
    # bag playback 数据回放
    bag_playback_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory('bag_record') + '/launch/bag_playback.launch.py'
        )
    )
    # driver 驱动1
    driver_launch1 = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory('livox_ros_driver2') + '/launch_ROS2/msg_MID360_launch.py'
        )
    )
    # driver 驱动2
    driver_launch2 = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory('livox_ros_driver2') + '/launch_ROS2/rviz_MID360_launch.py'
        )
    )
    # lio-sam 算法
    lio_sam_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory('lio_sam') + '/launch/run.launch.py'
        )
    )

    '''
    录制
        启动录制
        启动驱动
        voxel_filter 点云降采样文件
        启动pcd2pgm点云转换
        pointcloud_to_laserscan 点云转激光雷达
        启动 lio-sam 算法
    '''
    start_action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=0.5, actions=[bag_record_launch]),
        # launch.actions.TimerAction(period=1.0, actions=[driver_launch2]),
        launch.actions.TimerAction(period=2.5, actions=[lio_sam_launch]),
    ])

    # 回放
    play_action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=0.1, actions=[bag_playback_launch]),
        launch.actions.TimerAction(period=2.0, actions=[lio_sam_launch]),
    ])

    return launch.LaunchDescription([
        # start_action_group
        play_action_group
    ])