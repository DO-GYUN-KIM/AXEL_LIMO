"""실차용 launch입니다. 카메라, LiDAR 드라이버와 자율주행 노드를 모두 띄웁니다.

사용 예:
  ros2 launch autorace_bringup robot.launch.py
  ros2 launch autorace_bringup robot.launch.py start_mission:=m1_traffic_light
  ros2 launch autorace_bringup robot.launch.py use_camera:=false   # 드라이버를 따로 띄운 경우
  ros2 launch autorace_bringup robot.launch.py use_base:=false     # 차체 드라이버를 따로 띄운 경우

주행은 애커먼 모드만 씁니다(팀 결정 2026-11-04). 차 앞 빨간 걸쇠를 올려 LED가 녹색인지 확인하십시오.
애커먼 드라이버는 /limo/ack_cmd만 받으므로 cmd_vel_to_ackermann 노드가 /cmd_vel을 변환합니다.
조종기 SWB를 위로 올려야 코드 명령을 받습니다.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    if LaunchConfiguration('use_base').perform(context).lower() == 'true':
        # 위키 'ROS2 Limo Driver': 애커먼 모드 전용 드라이버. /limo/ack_cmd를 받고 /odom, /imu를 냅니다.
        actions.append(include_driver('limo_base', 'limo_base_ackerman.launch.py'))
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
        DeclareLaunchArgument('use_base', default_value='true'),
        DeclareLaunchArgument('use_ackermann_bridge', default_value='true'),
        DeclareLaunchArgument('start_mission', default_value='start'),
        DeclareLaunchArgument('use_yolo', default_value='true'),
        DeclareLaunchArgument('lane_debug', default_value='false'),
        OpaqueFunction(function=drivers),
        nodes,
        Node(package='autorace_control', executable='cmd_vel_to_ackermann', name='cmd_vel_to_ackermann',
             output='screen', parameters=[os.path.join(bringup, 'config', 'params.yaml')],
             condition=IfCondition(LaunchConfiguration('use_ackermann_bridge'))),
    ])
