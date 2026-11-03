"""M4 T자 주차 미션입니다(뼈대).

지금은 YAML의 steps_* 배열을 순서대로 실행하는 시간 기반 개루프 동작입니다.
    step i: linear = steps_linear[i], angular = steps_angular[i], steps_duration[i] 초 동안
실차에서 주차 구역 위치를 재고 숫자를 맞춘 뒤, 가능하면 LiDAR(양옆 정차 LIMO 거리)로
진입 시점과 정렬을 보정하도록 바꾸십시오. 네 모서리가 모두 구역 안이면 15점, 한 곳 걸치면 7점입니다.

담당: C (LiDAR 미션 + 주차)
"""
from .base import STOP, Cmd, MissionHandler, split_names


class ParkingMission(MissionHandler):
    DEFAULTS = {
        'trigger_classes': 'parking',  # TODO(확인): 주차 표지판이 있는지. 없으면 trigger_distance_m 사용
        'min_conf': 0.5,
        'min_area': 1500.0,
        'trigger_distance_m': -1.0,  # 0 이상이면 미션 진입 후 이 거리만큼 가서 시작합니다.
        'side_trigger_m': -1.0,  # TODO(확인): 0 이상이면 옆 차로(LIMO)까지 거리가 이 값 이하일 때 시작
        # TODO(확인): 아래 동작은 자리표시용 숫자입니다. 실제 주차 구역에 맞게 다시 정해야 합니다.
        'steps_linear': [0.15, 0.0, -0.12, -0.10, 0.0],
        'steps_angular': [0.0, 0.0, 1.2, 0.0, 0.0],
        'steps_duration': [1.0, 0.5, 1.6, 1.5, 0.5],
        'final_hold_s': 3.0,
        'approach_timeout_s': 60.0,
        'active_timeout_s': 40.0,
    }

    def enter(self, ctx):
        self.enter_dist = ctx.traveled
        self.step_idx = 0
        self.step_t0 = None
        n = min(len(self.p['steps_linear']), len(self.p['steps_angular']), len(self.p['steps_duration']))
        self.n_steps = n

    def triggered(self, ctx):
        if self.p['trigger_distance_m'] >= 0.0:
            return ctx.traveled - self.enter_dist >= self.p['trigger_distance_m']
        if self.p['side_trigger_m'] >= 0.0 and ctx.lidar is not None:
            lid = ctx.lidar
            return min(lid.left_lane_min_distance, lid.right_lane_min_distance) <= self.p['side_trigger_m']
        names = split_names(self.p['trigger_classes'])
        return ctx.best_detection(names, self.p['min_conf'], self.p['min_area']) is not None

    def step(self, ctx):
        if self.phase == 'approach':
            if not self.triggered(ctx):
                return ctx.follow_lane(ctx.base_speed)
            self.activate(ctx)
            self.step_t0 = ctx.now
        self.speed_cap = 0.2
        if self.step_idx < self.n_steps:
            if ctx.now - self.step_t0 >= self.p['steps_duration'][self.step_idx]:
                self.step_idx += 1
                self.step_t0 = ctx.now
                self.note = 'step %d/%d' % (self.step_idx, self.n_steps)
            if self.step_idx < self.n_steps:
                i = self.step_idx
                return Cmd(self.p['steps_linear'][i], self.p['steps_angular'][i])
        if ctx.now - self.step_t0 >= self.p['final_hold_s']:
            self.finish('ok')
        return STOP
