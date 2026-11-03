"""미션 이름 -> 핸들러 클래스 표입니다. missions.yaml의 mission_order에는 여기 있는 이름을 씁니다."""
from .base import Cmd, MissionContext, MissionHandler, StopHold, StopLineApproach  # noqa: F401
from .fork import ForkMission
from .lidar_missions import ConeMission, ObstacleLaneMission
from .parking import ParkingMission
from .simple import FinishMission, SlowZoneMission, StartMission, ZigzagMission
from .stop_line import CrosswalkMission, TrafficLightMission

MISSION_REGISTRY = {
    'start': StartMission,
    'm5_slow_zone': SlowZoneMission,
    'm6_zigzag': ZigzagMission,
    'm3_crosswalk': CrosswalkMission,
    'm7_obstacles': ObstacleLaneMission,
    'm2_fork': ForkMission,
    'm8_cones': ConeMission,
    'm1_traffic_light': TrafficLightMission,
    'm4_parking': ParkingMission,
    'finish': FinishMission,
}

DEFAULT_ORDER = list(MISSION_REGISTRY.keys())
