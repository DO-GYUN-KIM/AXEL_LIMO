"""오프라인(rosbag 재생)용 launch입니다. 드라이버 없이 노드만 띄우고 use_sim_time=true로 실행합니다.

사용 예 (터미널 2개):
  1) ros2 launch autorace_bringup offline.launch.py lane_debug:=true
  2) ros2 bag play bags/<폴더> --clock

bag 안에 실제 주행 때의 /cmd_vel이 들어 있으므로, 미션 매니저 출력은 /autorace/cmd_vel_offline으로
바꿔 내보냅니다(두 값이 섞이지 않게). rqt_plot 등으로 두 토픽을 비교할 수 있습니다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('autorace_bringup')
    nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'nodes.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'cmd_vel_topic': '/autorace/cmd_vel_offline',
            'start_mission': LaunchConfiguration('start_mission'),
            'use_yolo': LaunchConfiguration('use_yolo'),
            'lane_debug': LaunchConfiguration('lane_debug'),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('start_mission', default_value='start'),
        DeclareLaunchArgument('use_yolo', default_value='true'),
        DeclareLaunchArgument('lane_debug', default_value='true'),
        nodes,
    ])
