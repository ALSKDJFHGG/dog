from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
import datetime
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 使用绝对路径指定 bag 目录
    bag_data_dir = '/home/hzy/study/dog/data/bags/sicence_build_around'
    print(f"Bag data dir: {bag_data_dir}")
    if not os.path.exists(bag_data_dir):
        raise RuntimeError(f"Bag directory does not exist: {bag_data_dir}")
    bag_datas = os.listdir(bag_data_dir)
    # 选择第一个bag文件（或根据需要选择特定文件）
    if not bag_datas:
        raise RuntimeError(f"No bag files found in {bag_data_dir}")
    bag_data_path = os.path.join(bag_data_dir, bag_datas[0])
    print(f"Bag data path: {bag_data_path}")
    
    # 配置回放命令
    record_command = [
        'ros2', 'bag', 'play',
        bag_data_dir,
    ]

    return LaunchDescription([
        ExecuteProcess(
            cmd=record_command,
            output='screen',
            shell=False,
            name='bag_player'
        )
        # # 添加一个节点显示录制状态
        # Node(
        #     package='rqt_console',
        #     executable='rqt_console',
        #     name='rqt_console'
        # )
    ])