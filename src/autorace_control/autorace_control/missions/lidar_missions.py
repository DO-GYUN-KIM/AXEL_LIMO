"""LiDAR 미션: M7 3차선 정적 장애물 회피, M8 라바콘 통과입니다.

담당: C (LiDAR 미션 + 주차)
"""
from .base import STOP, Cmd, MissionHandler, clamp


class ObstacleLaneMission(MissionHandler):
    """M7: 현재 차로 앞에 정차 LIMO가 있으면 비어 있는 옆 차로로 S자 차선 변경을 합니다.

    차선 변경은 시간 기반 개루프(turn -> straight -> counter turn)입니다. 끝나면 다시 차선 추종.
    lane_index로 현재 몇 번째 차로인지 기억해서 트랙 밖(-2, +2)으로는 바꾸지 않습니다.
    """
    DEFAULTS = {
        'trigger_distance_m': 1.0,  # 현재 차로 장애물이 이 거리 안에 오면 차선 변경 시작
        'start_lane_index': 0,  # TODO(확인): 3차선 구간 진입 시 차로 (-1 왼쪽, 0 가운데, 1 오른쪽)
        'prefer_side': 'left',
        'speed': 0.2,
        # 애커먼 최대 조향(0.42 rad)에서 0.2 m/s일 때 낼 수 있는 각속도는 약 0.447 rad/s입니다.
        # 0.44 rad/s로 2.65 s씩 두 번 꺾으면 옆으로 약 0.55 m, 앞으로 약 0.85 m 갑니다(계산값).
        'turn_angular': 0.44,  # TODO(확인): 실차 조향이 한계까지 나오는지, 차선 폭 0.55 m에 맞는지
        'turn_time_s': 2.65,
        'straight_time_s': 0.0,
        'emergency_stop_m': 0.20,
        'section_length_m': 4.0,  # TODO(확인): 3차선 구간 길이
        'clear_time_s': 2.0,  # 장애물이 이 시간 동안 안 보이고 구간 길이를 지나면 완료
        'approach_timeout_s': 40.0,
        'active_timeout_s': 45.0,  # 7대를 피해야 하므로 길게 잡습니다.
    }

    def enter(self, ctx):
        self.lane_index = int(self.p['start_lane_index'])
        self.maneuver = None  # (방향 부호, 시작 시각)
        self.start_dist = None
        self.last_seen = ctx.now

    def blocked(self, ctx):
        lid = ctx.lidar
        return (lid is not None and lid.center_lane_occupied
                and lid.center_lane_min_distance < self.p['trigger_distance_m'])

    def choose_side(self, ctx):
        """+1 = 왼쪽, -1 = 오른쪽, 0 = 갈 곳 없음."""
        lid = ctx.lidar
        options = []
        if self.lane_index > -1 and not lid.left_lane_occupied:
            options.append(1)
        if self.lane_index < 1 and not lid.right_lane_occupied:
            options.append(-1)
        if not options:
            return 0
        pref = 1 if self.p['prefer_side'] == 'left' else -1
        return pref if pref in options else options[0]

    def run_maneuver(self, ctx):
        sign, t0 = self.maneuver
        t = ctx.now - t0
        tt, ts = self.p['turn_time_s'], self.p['straight_time_s']
        w = self.p['turn_angular']
        if t < tt:
            return Cmd(self.p['speed'], sign * w)
        if t < tt + ts:
            return Cmd(self.p['speed'], 0.0)
        if t < 2 * tt + ts:
            return Cmd(self.p['speed'], -sign * w)
        self.maneuver = None
        # 왼쪽(+1)으로 가면 lane_index가 -1 방향으로 바뀝니다.
        self.lane_index -= sign
        self.log('차선 변경 완료, lane_index=%d' % self.lane_index)
        return ctx.follow_lane(self.p['speed'])

    def step(self, ctx):
        if ctx.lidar is not None and ctx.lidar.front_min_distance < self.p['emergency_stop_m']:
            return STOP  # 너무 가까우면 정지(타임아웃이 결국 다음 미션으로 넘겨 줍니다)
        if self.phase == 'approach':
            if not self.blocked(ctx):
                return ctx.follow_lane(ctx.base_speed)
            self.activate(ctx)
            self.start_dist = ctx.traveled
        self.speed_cap = self.p['speed']
        if self.maneuver is not None:
            return self.run_maneuver(ctx)
        if self.blocked(ctx):
            self.last_seen = ctx.now
            side = self.choose_side(ctx)
            if side != 0:
                self.log('차선 변경 %s' % ('왼쪽' if side > 0 else '오른쪽'))
                self.maneuver = (side, ctx.now)
                return self.run_maneuver(ctx)
            return Cmd(0.05, 0.0)  # 양쪽 다 막힘: 아주 천천히 다가가며 다시 판단
        if (ctx.traveled - self.start_dist >= self.p['section_length_m']
                and ctx.now - self.last_seen >= self.p['clear_time_s']):
            self.speed_cap = None
            self.finish('ok')
        return ctx.follow_lane(self.p['speed'])


class ConeMission(MissionHandler):
    """M8: 라바콘 사이(왼쪽/오른쪽 가장 가까운 콘의 중점)로 조향합니다. 이 구간은 차선 판정이 없습니다."""
    DEFAULTS = {
        'min_cones': 2,
        'trigger_distance_m': 0.8,
        'speed': 0.18,
        'k_gap': 1.5,
        'max_angular': 1.2,
        'clear_time_s': 1.5,
        'min_section_m': 1.0,  # TODO(확인): 라바콘 구간 길이
        'approach_timeout_s': 40.0,
        'active_timeout_s': 40.0,
    }

    def enter(self, ctx):
        self.start_dist = None
        self.last_seen = ctx.now

    def cones_ahead(self, ctx):
        lid = ctx.lidar
        # TODO(확인): 갈림길 벽/표지판 기둥이 콘으로 잡히지 않는지 bag으로 확인
        return (lid is not None and lid.num_clusters >= self.p['min_cones']
                and lid.nearest_cluster_distance < self.p['trigger_distance_m'])

    def step(self, ctx):
        if self.phase == 'approach':
            if not self.cones_ahead(ctx):
                return ctx.follow_lane(ctx.base_speed)
            self.activate(ctx)
            self.start_dist = ctx.traveled
        self.speed_cap = self.p['speed']
        lid = ctx.lidar
        if lid is not None and lid.num_clusters > 0:
            self.last_seen = ctx.now
        if (ctx.traveled - self.start_dist >= self.p['min_section_m']
                and ctx.now - self.last_seen >= self.p['clear_time_s']):
            self.speed_cap = None
            self.finish('ok')
            return ctx.follow_lane(ctx.base_speed)
        if lid is not None and lid.gap_found:
            m = self.p['max_angular']
            return Cmd(self.p['speed'], clamp(self.p['k_gap'] * lid.gap_angle, -m, m))
        return ctx.follow_lane(self.p['speed'])
