"""자율주행 노드 묶음입니다. robot.launch.py와 offline.launch.py가 이 파일을 포함합니다.

실행 노드: lane_detector, yolo_detector, lidar_processor, parking_detector, lane_follower, mission_manager
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('autorace_bringup'), 'config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    missions_file = LaunchConfiguration('missions_file')
    sim = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(cfg, 'params.yaml')),
        DeclareLaunchArgument('missions_file', default_value=os.path.join(cfg, 'missions.yaml')),
        DeclareLaunchArgument('start_mission', default_value='start',
                              description='이 미션부터 시작합니다 (예: m3_crosswalk)'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('use_yolo', default_value='true'),
        DeclareLaunchArgument('lane_debug', default_value='false',
                              description='true면 /autorace/lane_debug/compressed 를 내보냅니다'),
    ]

    def perception(name, extra=None, condition=None):
        p = [params_file, sim]
        if extra:
            p.append(extra)
        return Node(package='autorace_perception', executable=name, name=name, output='screen',
                    parameters=p, condition=condition)

    nodes = [
        perception('lane_detector', {'publish_debug': ParameterValue(LaunchConfiguration('lane_debug'),
                                                                     value_type=bool)}),
        perception('yolo_detector', condition=IfCondition(LaunchConfiguration('use_yolo'))),
        perception('lidar_processor'),
        perception('parking_detector'),
        Node(package='autorace_control', executable='lane_follower', name='lane_follower',
             output='screen', parameters=[params_file, sim]),
        Node(package='autorace_control', executable='mission_manager', name='mission_manager',
             output='screen',
             parameters=[params_file, missions_file, sim, {
                 'start_mission': LaunchConfiguration('start_mission'),
                 'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
             }]),
    ]
    return LaunchDescription(args + nodes)
