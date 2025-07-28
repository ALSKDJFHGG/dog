from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
import datetime
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取当前工作空间路径
    workspace_dir = os.path.join(os.path.expanduser('~'), os.getcwd())
    
    # 在源包中创建bags目录（如果不存在）
    bag_dir = os.path.join(workspace_dir, 'src/bag_record/bags')
    os.makedirs(bag_dir, exist_ok=True)
    
    # 生成带时间戳的bag文件名
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = f"sensor_data_{timestamp}"
    
    # 完整bag路径
    full_bag_path = os.path.join(bag_dir, bag_name)
    
    # 配置录制命令
    record_command = [
        'ros2', 'bag', 'record',
        '--output', full_bag_path,
        '/livox/imu',
        '/livox/lidar'
    ]

    return LaunchDescription([
        ExecuteProcess(
            cmd=record_command,
            output='screen',
            shell=False,
            name='bag_recorder'
        )
        # # 添加一个节点显示录制状态
        # Node(
        #     package='rqt_console',
        #     executable='rqt_console',
        #     name='rqt_console'
        # )
    ])