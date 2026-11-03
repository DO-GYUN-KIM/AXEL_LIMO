"""정지선에서 멈추는 미션: M3 보행자 표지판, M1 신호등입니다.

공통: StopLineApproach로 정지선 앞(판정 기준 = 앞바퀴, ±5 cm)까지 천천히 다가가 멈추고,
StopHold로 정지 시간을 셉니다.
담당: A (정지 제어) + B (표지판/신호등 검출)
"""
from .base import STOP, MissionHandler, StopHold, StopLineApproach, split_names


class _StopLineMission(MissionHandler):
    DEFAULTS = {
        'min_conf': 0.5,
        'min_area': 1500.0,  # TODO(확인): 표지판/신호등이 가까울 때의 bbox 크기
        'approach_speed': 0.15,
        'creep_speed': 0.05,
        'approach_gain': 1.0,
        'stop_margin_m': 0.02,  # TODO(확인): 실차에서 정지 오차를 재서 맞춥니다(±5 cm 판정).
        'leave_distance_m': 0.4,  # 출발 후 이만큼 지나면 미션 완료
    }

    def enter(self, ctx):
        self.stopper = StopLineApproach(self.p['stop_margin_m'], self.p['creep_speed'],
                                        self.p['approach_speed'], self.p['approach_gain'])
        self.leave_from = None

    def go_and_leave(self, ctx):
        """정지 후 출발. leave_distance_m 지나면 완료."""
        if self.leave_from is None:
            self.leave_from = ctx.traveled
            self.log('출발')
        if ctx.traveled - self.leave_from >= self.p['leave_distance_m']:
            self.finish('ok')
        return ctx.follow_lane(ctx.base_speed)


class CrosswalkMission(_StopLineMission):
    """M3: 보행자 표지판을 보면 정지선에서 완전 정지, hold_s(≥3 s) 대기 후 출발합니다."""
    DEFAULTS = {
        'trigger_classes': 'pedestrian',
        'hold_s': 3.5,  # 규정 3 s 이상. 여유 0.5 s
        'approach_timeout_s': 40.0,
        'active_timeout_s': 14.0,  # 15 s 규정보다 짧게 잡아 반드시 출발합니다.
    }

    def enter(self, ctx):
        super().enter(ctx)
        self.hold = StopHold(self.p['hold_s'])

    def step(self, ctx):
        if self.phase == 'approach':
            det = ctx.best_detection(split_names(self.p['trigger_classes']),
                                     self.p['min_conf'], self.p['min_area'])
            if det is None:
                return ctx.follow_lane(ctx.base_speed)
            self.activate(ctx, '(%s %.2f)' % (det.class_name, det.confidence))
        if self.leave_from is not None:
            return self.go_and_leave(ctx)
        if not self.hold.started:
            cmd = self.stopper.update(ctx)
            if cmd is not None:
                return cmd
            self.hold.start(ctx.now)
            self.log('정지선 도착, %.1f s 대기' % self.p['hold_s'])
        if self.hold.done(ctx.now):
            return self.go_and_leave(ctx)
        return STOP
    # 참고: active_timeout_s가 지나면 기본 on_timeout()이 미션을 끝내고, 다음 미션이 차선 추종으로
    # 바로 출발합니다. 즉 정지 중에 타임아웃이 나도 차는 멈춰 있지 않습니다.


class TrafficLightMission(_StopLineMission):
    """M1: 빨강/노랑이면 정지선에서 멈추고, 초록이 green_confirm_s 동안 보이면 출발합니다.

    옆에 광고 모니터가 있으므로 bbox 위치(x_min_ratio~x_max_ratio)와 크기로 걸러냅니다.
    초록을 끝내 못 보면 max_wait_s 뒤에 출발합니다(멈춰 있는 것보다 낫다고 판단).
    """
    DEFAULTS = {
        'stop_classes': 'red,yellow',
        'go_classes': 'green',
        'x_min_ratio': 0.0,  # TODO(확인): 광고 모니터가 화면 어느 쪽에 보이는지 보고 범위를 좁힙니다.
        'x_max_ratio': 1.0,
        'green_confirm_s': 0.3,
        'stop_even_if_green': False,  # TODO(확인): 초록일 때도 정지선 정지가 필요한지 규정 확인
        'max_wait_s': 60.0,  # TODO(확인): 신호 주기
        'approach_timeout_s': 60.0,
        'active_timeout_s': 90.0,
    }

    def enter(self, ctx):
        super().enter(ctx)
        self.waiting = StopHold(self.p['max_wait_s'])
        self.green_since = None

    def light(self, ctx):
        xr = (self.p['x_min_ratio'], self.p['x_max_ratio'])
        stop = ctx.best_detection(split_names(self.p['stop_classes']), self.p['min_conf'], self.p['min_area'], xr)
        go = ctx.best_detection(split_names(self.p['go_classes']), self.p['min_conf'], self.p['min_area'], xr)
        if go is not None and (stop is None or go.confidence > stop.confidence):
            self.green_since = self.green_since if self.green_since is not None else ctx.now
            return 'green'
        self.green_since = None
        return 'stop' if stop is not None else 'none'

    def green_confirmed(self, ctx):
        return self.green_since is not None and ctx.now - self.green_since >= self.p['green_confirm_s']

    def step(self, ctx):
        state = self.light(ctx)
        if self.phase == 'approach':
            if state == 'none':
                return ctx.follow_lane(ctx.base_speed)
            self.activate(ctx, '(신호 %s)' % state)
        if self.leave_from is not None:
            return self.go_and_leave(ctx)
        if not self.waiting.started:
            if state == 'green' and not self.p['stop_even_if_green'] and self.green_confirmed(ctx):
                self.log('초록 신호, 정지 없이 통과')
                return self.go_and_leave(ctx)
            cmd = self.stopper.update(ctx)
            if cmd is not None:
                return cmd
            self.waiting.start(ctx.now)
            self.log('정지선 도착, 초록 대기')
        if self.green_confirmed(ctx):
            return self.go_and_leave(ctx)
        if self.waiting.done(ctx.now):
            self.node.get_logger().warn('[%s] 초록을 못 봤지만 max_wait_s가 지나 출발합니다' % self.name)
            return self.go_and_leave(ctx)
        return STOP
