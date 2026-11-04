"""Isaac Sim과 우리 노드를 연결하는 launch입니다 (팀 데스크탑에서 실행).

Isaac Sim에서 'Limo ROS2 Ackermann (AXEL)' 예제를 Load·Play 한 뒤:
  ros2 launch ~/AXEL_LIMO/sim/isaac_sim/launch/isaac_sim.launch.py
  ros2 launch ~/AXEL_LIMO/sim/isaac_sim/launch/isaac_sim.launch.py use_nodes:=false   # 수동 조종 시험만

띄우는 것:
  1) ack_stamp_relay: /limo/ack_cmd → /sim/ackermann_cmd (Stamped)
  2) image_transport republish: /camera/color/image_raw → /camera/color/image_raw/compressed (실차와 같은 이름)
  3) cmd_vel_to_ackermann: /cmd_vel → /limo/ack_cmd (저장소 PR #1의 노드, 실차와 같은 경로)
  4) nodes.launch.py (use_sim_time:=true, YOLO는 기본 끔)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
RELAY = os.path.join(HERE, '..', 'tools', 'ack_stamp_relay.py')


def generate_launch_description():
    use_nodes = LaunchConfiguration('use_nodes')
    args = [
        DeclareLaunchArgument('use_nodes', default_value='true',
                              description='false면 변환·중계만 띄웁니다(조종 시험용)'),
        DeclareLaunchArgument('use_yolo', default_value='false'),
        DeclareLaunchArgument('start_mission', default_value='start'),
        DeclareLaunchArgument('speed_sign', default_value='1.0',
                              description='시뮬레이터에서 차가 거꾸로 가면 -1.0'),
    ]
    sim = {'use_sim_time': True}
    actions = [
        ExecuteProcess(cmd=['python3', RELAY, '--ros-args', '-p', 'use_sim_time:=true',
                            '-p', ['speed_sign:=', LaunchConfiguration('speed_sign')]], output='screen'),
        Node(package='image_transport', executable='republish', name='sim_image_compress',
             arguments=['raw', 'compressed'],
             remappings=[('in', '/camera/color/image_raw'),
                         ('out/compressed', '/camera/color/image_raw/compressed')],
             parameters=[sim]),
        Node(package='autorace_control', executable='cmd_vel_to_ackermann', name='cmd_vel_to_ackermann',
             output='screen', parameters=[sim]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('autorace_bringup'), 'launch', 'nodes.launch.py')),
            launch_arguments={'use_sim_time': 'true',
                              'use_yolo': LaunchConfiguration('use_yolo'),
                              'start_mission': LaunchConfiguration('start_mission')}.items(),
            condition=IfCondition(use_nodes)),
    ]
    return LaunchDescription(args + actions)
