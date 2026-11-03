"""M2 갈림길 A(왼쪽)/B(오른쪽) 미션입니다.

표지판은 매 주행 무작위이므로 YOLO 검출을 여러 프레임 모아 투표(신뢰도 합)로 정합니다.
정한 뒤에는 bias_start_m 만큼 더 가서, bias_length_m 동안 차선 추종 조향에 좌/우 bias를 더해
원하는 갈래로 들어갑니다. 표지판을 끝내 못 보면 default_branch로 갑니다(멈추지 않음).

담당: B (표지판 검출) + A (조향)
"""
from .base import Cmd, MissionHandler


class ForkMission(MissionHandler):
    DEFAULTS = {
        'left_class': 'left_A',
        'right_class': 'right_B',
        'min_conf': 0.5,
        'min_area': 1000.0,
        'vote_threshold': 2.0,  # 신뢰도 합이 이 값을 넘으면 결정
        'decide_timeout_s': 20.0,  # 이 시간 안에 못 정하면 default_branch
        'default_branch': 'A',
        'bias_start_m': 0.3,  # TODO(확인): 표지판 위치에서 갈림길 분기점까지 거리
        'bias_length_m': 0.8,
        'bias_angular': 0.6,  # TODO(확인): 분기 곡률에 맞게 조정
        'speed': 0.25,
        'approach_timeout_s': 40.0,
        'active_timeout_s': 15.0,
    }

    def enter(self, ctx):
        self.votes = {'A': 0.0, 'B': 0.0}
        self.branch = None
        self.decided_at = None

    def decide(self, branch, ctx, why):
        self.branch = branch
        self.decided_at = ctx.traveled
        self.note = 'branch=' + branch
        self.activate(ctx, '(%s, %s, votes=%s)' % (branch, why, self.votes))

    def on_timeout(self, ctx, reason):
        if self.phase == 'approach':
            # 표지판을 못 봤어도 갈래는 골라야 하므로 기본값으로 진행합니다.
            self.decide(self.p['default_branch'], ctx, reason)
            return
        super().on_timeout(ctx, reason)

    def step(self, ctx):
        if self.phase == 'approach':
            for key, cls in (('A', self.p['left_class']), ('B', self.p['right_class'])):
                det = ctx.best_detection([cls], self.p['min_conf'], self.p['min_area'])
                if det is not None:
                    self.votes[key] += det.confidence
            best = max(self.votes, key=self.votes.get)
            if self.votes[best] >= self.p['vote_threshold']:
                self.decide(best, ctx, 'vote')
            elif ctx.now - self.t_enter > self.p['decide_timeout_s']:
                self.decide(self.p['default_branch'], ctx, 'decide_timeout')
            else:
                return ctx.follow_lane(ctx.base_speed)
        d = ctx.traveled - self.decided_at
        if d < self.p['bias_start_m']:
            return ctx.follow_lane(self.p['speed'])
        if d < self.p['bias_start_m'] + self.p['bias_length_m']:
            sign = 1.0 if self.branch == 'A' else -1.0
            return Cmd(self.p['speed'], ctx.lane_angular + sign * self.p['bias_angular'], 'lane')
        self.finish('ok')
        return ctx.follow_lane(ctx.base_speed)
