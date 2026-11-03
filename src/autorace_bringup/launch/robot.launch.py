"""실차용 launch입니다. 카메라, LiDAR 드라이버와 자율주행 노드를 모두 띄웁니다.

사용 예:
  ros2 launch autorace_bringup robot.launch.py
  ros2 launch autorace_bringup robot.launch.py start_mission:=m1_traffic_light
  ros2 launch autorace_bringup robot.launch.py use_camera:=false   # 드라이버를 따로 띄운 경우
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_driver(package, launch_file):
    try:
        path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    except PackageNotFoundError:
        return LogInfo(msg='[경고] %s 패키지를 찾지 못했습니다. ~/wego_ws를 source 했는지 확인하십시오.' % package)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))


def drivers(context):
    actions = []
    if LaunchConfiguration('use_camera').perform(context).lower() == 'true':
        # 주최 측 교육자료 기준 카메라 launch
        actions.append(include_driver('orbbec_camera', 'astra_stereo_u3.launch.py'))
    if LaunchConfiguration('use_lidar').perform(context).lower() == 'true':
        actions.append(include_driver('ydlidar_ros2_driver', 'ydlidar.launch.py'))
    # TODO(확인): LIMO 베이스 드라이버(/cmd_vel, /odom)를 따로 띄워야 하는지 확인. 필요하면 여기에 추가합니다.
    return actions


def generate_launch_description():
    bringup = get_package_share_directory('autorace_bringup')
    nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'nodes.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'start_mission': LaunchConfiguration('start_mission'),
            'use_yolo': LaunchConfiguration('use_yolo'),
            'lane_debug': LaunchConfiguration('lane_debug'),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('start_mission', default_value='start'),
        DeclareLaunchArgument('use_yolo', default_value='true'),
        DeclareLaunchArgument('lane_debug', default_value='false'),
        OpaqueFunction(function=drivers),
        nodes,
    ])
