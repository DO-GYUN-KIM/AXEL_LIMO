"""출발/도착, M5 서행 구간, M6 지그재그 서행 유도선 미션입니다.

담당: A (차선/제어/통합)
"""
from .base import STOP, Cmd, MissionHandler, clamp, split_names, wrap_angle


class StartMission(MissionHandler):
    """출발: start_delay_s 만큼 기다린 뒤 다음 미션으로 넘어갑니다."""
    DEFAULTS = {
        'start_delay_s': 1.0,  # TODO(확인): 출발 신호 방식(심판 구두 신호 등)을 주최 측에 확인
    }

    def enter(self, ctx):
        self.activate(ctx, '(출발 대기)')

    def step(self, ctx):
        if ctx.now - self.t_active >= self.p['start_delay_s']:
            self.finish('ok')
        return STOP


class FinishMission(MissionHandler):
    """도착: 계속 정지합니다. 타임아웃으로 끝나지 않습니다."""

    def tick(self, ctx):
        self.phase = 'active'
        return STOP


class SlowZoneMission(MissionHandler):
    """M5 서행 구간(방지턱): 서행 표지/노면 표시를 보면 section_length_m 동안 0.2 m/s 이하로 달립니다."""
    DEFAULTS = {
        'trigger_classes': 'slow',  # TODO(확인): 노면 마름모/'천천히' 표시를 별도 클래스로 학습할지 결정
        'min_conf': 0.5,
        'min_area': 1500.0,  # TODO(확인): 표지판이 이 크기 이상일 때(충분히 가까울 때)만 반응
        'section_length_m': 1.5,  # TODO(확인): 서행 구간 실제 길이
        'speed': 0.15,  # 육안 판정이므로 0.2보다 넉넉히 낮춥니다.
        'approach_timeout_s': 40.0,
        'active_timeout_s': 30.0,
    }

    def enter(self, ctx):
        self.start_dist = None

    def step(self, ctx):
        if self.phase == 'approach':
            det = ctx.best_detection(split_names(self.p['trigger_classes']),
                                     self.p['min_conf'], self.p['min_area'])
            if det is None:
                return ctx.follow_lane(ctx.base_speed)
            self.activate(ctx, '(%s %.2f)' % (det.class_name, det.confidence))
            self.start_dist = ctx.traveled
        self.speed_cap = self.p['speed']
        if ctx.traveled - self.start_dist >= self.p['section_length_m']:
            self.speed_cap = None
            self.finish('ok')
        return ctx.follow_lane(self.p['speed'])


class ZigzagMission(MissionHandler):
    """M6 지그재그 서행 유도선: 차선이 지그재그이므로 차선을 믿지 않고 직전 헤딩(odom yaw)을 유지합니다.

    start_after_m 만큼 차선을 따라간 뒤, 그 순간의 yaw를 기준으로 section_length_m 동안 직진합니다.
    odom이 없으면 차선 추종 조향을 lane_blend 비율로 약하게 줄여서 달립니다.
    """
    DEFAULTS = {
        'trigger_classes': '',  # 비우면 거리(start_after_m)로 시작합니다.
        'min_conf': 0.5,
        'start_after_m': 0.3,  # TODO(확인): M5 끝에서 지그재그 시작점까지 거리
        'section_length_m': 1.7,  # 규정집: 지그재그 구간 1677 mm
        'speed': 0.15,
        'k_yaw': 1.5,
        'lane_blend': 0.0,  # 0 = 헤딩 유지만, 1 = 차선 추종을 그대로 더함
        'approach_timeout_s': 20.0,
        'active_timeout_s': 20.0,
    }

    def enter(self, ctx):
        self.enter_dist = ctx.traveled
        self.start_dist = None
        self.yaw_ref = None

    def step(self, ctx):
        if self.phase == 'approach':
            names = split_names(self.p['trigger_classes'])
            hit = (ctx.best_detection(names, self.p['min_conf']) is not None) if names else \
                (ctx.traveled - self.enter_dist >= self.p['start_after_m'])
            if not hit:
                return ctx.follow_lane(ctx.base_speed)
            self.activate(ctx)
            self.start_dist = ctx.traveled
            self.yaw_ref = ctx.pose[2] if ctx.pose is not None else None
        self.speed_cap = self.p['speed']
        if ctx.traveled - self.start_dist >= self.p['section_length_m']:
            self.speed_cap = None
            self.finish('ok')
            return ctx.follow_lane(ctx.base_speed)
        blend = self.p['lane_blend'] * ctx.lane_angular
        if self.yaw_ref is None or ctx.pose is None:
            return Cmd(self.p['speed'], 0.5 * ctx.lane_angular + blend, 'self')
        w = self.p['k_yaw'] * wrap_angle(self.yaw_ref - ctx.pose[2]) + blend
        return Cmd(self.p['speed'], clamp(w, -1.0, 1.0), 'self')
