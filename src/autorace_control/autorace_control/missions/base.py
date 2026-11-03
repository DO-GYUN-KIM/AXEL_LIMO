"""미션 핸들러 공통 부분입니다.

모든 미션은 MissionHandler를 상속하고 enter() / step() / is_done()을 구현합니다.
각 미션은 두 단계로 진행합니다.
  - approach : 차선을 따라가며 미션 시작 신호(표지판, LiDAR 물체 등)를 기다립니다.
  - active   : 미션 동작을 수행합니다.
단계마다 타임아웃이 있어서, 시간이 지나면 미션을 포기하고 다음 미션으로 넘어갑니다.
(규정상 15초 안에 진입/재출발하지 못하면 어차피 0점이므로, 차가 멈춰 서 있는 것이 가장 나쁩니다.)
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class Cmd:
    linear: float = 0.0
    angular: float = 0.0
    source: str = 'self'  # 'lane' = 차선 추종 결과 사용, 'self' = 미션이 직접 만든 명령


STOP = Cmd(0.0, 0.0, 'self')


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def split_names(text):
    """'red, yellow' 같은 문자열을 ['red', 'yellow']로 바꿉니다."""
    return [s.strip() for s in str(text).split(',') if s.strip()]


@dataclass
class MissionContext:
    """매 제어 주기마다 mission_manager가 만들어 미션에 넘기는 입력 묶음입니다."""
    now: float
    base_speed: float
    slow_speed: float
    lane: Optional[object] = None       # LaneInfo (오래되었으면 None)
    lane_angular: float = 0.0           # lane_follower가 계산한 각속도
    detections: List[object] = field(default_factory=list)  # Detection 목록 (오래되었으면 빈 목록)
    image_width: int = 0
    image_height: int = 0
    lidar: Optional[object] = None      # LidarInfo (오래되었으면 None)
    pose: Optional[Tuple[float, float, float]] = None  # (x, y, yaw) from /odom
    traveled: float = 0.0               # 누적 주행 거리 [m] (odom 없으면 명령 속도 적분)

    def follow_lane(self, speed):
        return Cmd(speed, self.lane_angular, 'lane')

    def best_detection(self, names, min_conf=0.0, min_area=0.0, x_range=(0.0, 1.0)):
        """names 중 조건을 만족하는 가장 신뢰도 높은 검출을 돌려줍니다. 없으면 None.

        min_area: bbox 넓이 [px^2]. 멀리 있는 표지판을 거르는 용도입니다.
        x_range : bbox 중심 x가 이미지 폭 대비 이 범위 안에 있어야 합니다(광고 모니터 거르기 등).
        """
        best = None
        for d in self.detections:
            if d.class_name not in names or d.confidence < min_conf:
                continue
            if (d.x_max - d.x_min) * (d.y_max - d.y_min) < min_area:
                continue
            if self.image_width > 0:
                cx = (d.x_min + d.x_max) / 2.0 / self.image_width
                if not (x_range[0] <= cx <= x_range[1]):
                    continue
            if best is None or d.confidence > best.confidence:
                best = d
        return best


class StopHold:
    """N초 동안 정지를 유지하는 도우미입니다 (M3 보행자, M1 신호등)."""

    def __init__(self, hold_s):
        self.hold_s = hold_s
        self.t0 = None

    def start(self, now):
        if self.t0 is None:
            self.t0 = now

    @property
    def started(self):
        return self.t0 is not None

    def done(self, now):
        return self.t0 is not None and (now - self.t0) >= self.hold_s

    def reset(self):
        self.t0 = None


class StopLineApproach:
    """정지선 앞 정지 도우미입니다.

    카메라로 정지선 거리를 볼 수 있을 때는 그 값으로 '멈출 누적 주행거리'를 계산해 두고,
    가까워져서 정지선이 카메라에서 사라져도 주행거리(odom)로 남은 거리를 계산해 멈춥니다.
    """

    def __init__(self, margin_m, creep_speed, approach_speed, gain):
        self.margin_m = margin_m        # 브레이크 지연 보정: 이만큼 일찍 멈춥니다.
        self.creep_speed = creep_speed
        self.approach_speed = approach_speed
        self.gain = gain
        self.target = None

    def reset(self):
        self.target = None

    def update(self, ctx):
        """도착하면 None, 아니면 주행 명령(Cmd)을 돌려줍니다."""
        lane = ctx.lane
        if lane is not None and lane.stop_line_detected:
            self.target = ctx.traveled + lane.stop_line_distance - self.margin_m
        if self.target is None:
            return ctx.follow_lane(self.approach_speed)
        remaining = self.target - ctx.traveled
        if remaining <= 0.0:
            return None
        return ctx.follow_lane(clamp(self.gain * remaining, self.creep_speed, self.approach_speed))


class MissionHandler:
    """미션 공통 클래스입니다. 하위 클래스는 DEFAULTS로 파라미터 기본값을 추가합니다.

    파라미터 이름은 '<미션이름>.<키>' 형식이며 missions.yaml에서 바꿉니다.
    """
    DEFAULTS = {
        'approach_timeout_s': 60.0,  # 시작 신호를 이 시간 안에 못 찾으면 다음 미션으로
        'active_timeout_s': 14.0,    # 동작이 이 시간 안에 안 끝나면 포기하고 다음 미션으로
    }

    def __init__(self, node, name):
        self.node = node
        self.name = name
        self.p = {}
        merged = {}
        for klass in reversed(type(self).__mro__):
            merged.update(klass.__dict__.get('DEFAULTS', {}))
        for key, default in merged.items():
            pname = '%s.%s' % (name, key)
            if not node.has_parameter(pname):
                node.declare_parameter(pname, default)
            self.p[key] = node.get_parameter(pname).value
        self.phase = 'idle'
        self.result = ''
        self.note = ''
        self.speed_cap = None  # 이 미션 동안 최대 속도 (None = 전역 한계만 적용)
        self.t_enter = 0.0
        self.t_active = 0.0

    # ---- mission_manager가 부르는 함수 ----
    def start(self, ctx):
        self.phase = 'approach'
        self.result = ''
        self.note = ''
        self.speed_cap = None
        self.t_enter = ctx.now
        self.enter(ctx)

    def tick(self, ctx):
        if self.phase == 'approach' and ctx.now - self.t_enter > self.p['approach_timeout_s']:
            self.on_timeout(ctx, 'approach_timeout')
        elif self.phase == 'active' and ctx.now - self.t_active > self.p['active_timeout_s']:
            self.on_timeout(ctx, 'active_timeout')
        if self.phase == 'done':
            return ctx.follow_lane(ctx.base_speed)
        return self.step(ctx)

    def is_done(self):
        return self.phase == 'done'

    def elapsed(self, ctx):
        return ctx.now - (self.t_active if self.phase == 'active' else self.t_enter)

    # ---- 하위 클래스가 쓰는 함수 ----
    def activate(self, ctx, why=''):
        self.phase = 'active'
        self.t_active = ctx.now
        self.log('active 진입 %s' % why)

    def finish(self, result='ok'):
        self.phase = 'done'
        self.result = result
        self.log('종료: %s' % result)

    def on_timeout(self, ctx, reason):
        """타임아웃 기본 동작: 포기하고 다음 미션으로 넘어갑니다. 필요하면 하위 클래스에서 바꿉니다."""
        self.node.get_logger().warn('[%s] %s -> 다음 미션으로 넘어갑니다' % (self.name, reason))
        self.finish(reason)

    def log(self, text):
        self.node.get_logger().info('[%s] %s' % (self.name, text))

    # ---- 하위 클래스가 구현 ----
    def enter(self, ctx):
        pass

    def step(self, ctx):
        return ctx.follow_lane(ctx.base_speed)
