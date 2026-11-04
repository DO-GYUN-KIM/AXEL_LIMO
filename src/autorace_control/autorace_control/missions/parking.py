"""M4 T자 주차 미션입니다. 카메라 주, 라이다 보조, 앞으로 들어가는(전면 진입) 방식입니다.

도면상 주차칸은 위쪽 안쪽 막힌 차로의 오른쪽 옆에 수직으로 붙어 있습니다(도면 판독, 추정).
진행 단계:
  approach : 차선을 따라가며 주차 표지판(또는 거리/옆 LIMO)으로 시작 지점을 찾습니다.
  pass     : 시작 지점에서 pass_distance_m 만큼 더 갑니다(칸 입구를 꺾기 좋은 위치로).
  turn     : odom yaw 기준으로 turn_angle_deg 만큼 꺾습니다(오른쪽이면 음수 각속도).
             도는 중에 칸 선이 보이고 방향 오차가 작아지면 바로 enter로 넘어갑니다.
  enter    : 카메라 칸 중심/방향 + 라이다 양옆 거리로 조향하며 천천히 들어갑니다.
             끝 선 거리가 보이면 "멈출 누적 주행거리"를 계산하고, 선이 화면에서 사라져도
             주행거리로 남은 거리만큼 가서 멈춥니다(정지선 정지와 같은 방식).
  hold     : final_hold_s 동안 정지 후 종료.
채점: 네 모서리 모두 칸 안 15점, 한 곳 걸침 7점, 두 곳 이상 0점. 옆 LIMO 접촉 1회 -1점.

담당: C (주차)
"""
import math

from .base import STOP, Cmd, MissionHandler, clamp, split_names, wrap_angle


class ParkingMission(MissionHandler):
    DEFAULTS = {
        'trigger_classes': 'parking',  # TODO(확인): 주차 표지판 모양/유무
        'min_conf': 0.5,
        'min_area': 1500.0,
        'trigger_distance_m': -1.0,  # 0 이상이면 미션 진입 후 이 거리에서 시작
        'side_trigger_m': -1.0,      # 0 이상이면 옆 물체(정차 LIMO)가 이 거리 이하일 때 시작
        'pass_distance_m': 0.10,     # TODO(확인): 시작 지점에서 꺾기 전 더 갈 거리
        'turn_direction': -1.0,      # -1 = 오른쪽, +1 = 왼쪽 (도면상 오른쪽으로 추정)
        'turn_angle_deg': 90.0,
        'turn_linear': 0.12,         # 애커먼은 전진하며 꺾어야 합니다
        'turn_angular': 0.8,         # TODO(확인): 최소 회전 반경 안에서 칸에 들어가는지 실차 확인
        'turn_handoff_deg': 25.0,    # 남은 회전이 이 값 이하이고 칸 선이 보이면 enter로
        'enter_speed': 0.08,
        'creep_speed': 0.04,
        'k_offset': 2.0,             # 칸 중심 가로 오차 -> 각속도
        'k_heading': 1.2,            # 칸 방향 오차 -> 각속도
        'k_side': 1.5,               # 라이다 양옆 거리 차 -> 각속도 (0이면 사용 안 함)
        'side_valid_max_m': 0.40,    # 이보다 먼 옆 거리는 무시
        'stop_before_end_m': 0.03,   # TODO(확인): 끝 선 앞 몇 m에서 멈출지 (차 길이/칸 길이로 계산)
        'max_enter_m': 0.70,         # 끝 선을 못 봐도 이 거리 들어가면 멈춤 (칸 길이 근처)
        'final_hold_s': 3.0,         # TODO(확인): 주차 후 정지 유지 시간 규정
        'approach_timeout_s': 60.0,
        'active_timeout_s': 40.0,
    }

    def enter(self, ctx):
        self.enter_dist = ctx.traveled
        self.sub = 'approach'
        self.mark = 0.0
        self.yaw0 = None
        self.turned = 0.0
        self.last_yaw = None
        self.stop_target = None
        self.t_hold = None

    def triggered(self, ctx):
        if self.p['trigger_distance_m'] >= 0.0:
            return ctx.traveled - self.enter_dist >= self.p['trigger_distance_m']
        if self.p['side_trigger_m'] >= 0.0 and ctx.lidar is not None:
            side = min(ctx.lidar.side_left_distance, ctx.lidar.side_right_distance)
            return side <= self.p['side_trigger_m']
        names = split_names(self.p['trigger_classes'])
        return ctx.best_detection(names, self.p['min_conf'], self.p['min_area']) is not None

    def go(self, sub, ctx):
        self.sub = sub
        self.mark = ctx.traveled
        self.note = sub
        self.log('-> %s' % sub)

    def slot_steer(self, ctx):
        s = ctx.parking
        w = self.p['k_offset'] * s.center_offset + self.p['k_heading'] * s.heading_error
        lid = ctx.lidar
        if self.p['k_side'] > 0.0 and lid is not None:
            l, r = lid.side_left_distance, lid.side_right_distance
            m = self.p['side_valid_max_m']
            if l < m and r < m:
                w += self.p['k_side'] * (l - r) / 2.0  # 왼쪽이 더 멀면 왼쪽으로
        return w

    def step(self, ctx):
        if self.sub == 'approach':
            if not self.triggered(ctx):
                return ctx.follow_lane(ctx.base_speed)
            self.activate(ctx, 'trigger')
            self.speed_cap = 0.2
            self.go('pass', ctx)

        if self.sub == 'pass':
            if ctx.traveled - self.mark < self.p['pass_distance_m']:
                return ctx.follow_lane(self.p['turn_linear'])
            self.go('turn', ctx)

        if self.sub == 'turn':
            target = math.radians(self.p['turn_angle_deg'])
            if ctx.pose is not None:
                if self.last_yaw is not None:
                    self.turned += abs(wrap_angle(ctx.pose[2] - self.last_yaw))
                self.last_yaw = ctx.pose[2]
            else:
                # odom이 없으면 명령 각속도를 적분한 값으로 대신합니다(부정확).
                self.turned += abs(self.p['turn_angular']) * 0.05  # TODO(확인): 제어 주기 50 ms 가정
            left = target - self.turned
            s = ctx.parking
            if s is not None and s.slot_valid and left <= math.radians(self.p['turn_handoff_deg']):
                self.go('enter', ctx)
            elif left <= 0.0:
                self.go('enter', ctx)
            else:
                return Cmd(self.p['turn_linear'], self.p['turn_direction'] * self.p['turn_angular'])

        if self.sub == 'enter':
            s = ctx.parking
            if s is not None and s.end_line_found:
                self.stop_target = ctx.traveled + s.end_line_distance - self.p['stop_before_end_m']
            entered = ctx.traveled - self.mark
            done = entered >= self.p['max_enter_m']
            if self.stop_target is not None:
                remaining = self.stop_target - ctx.traveled
                done = done or remaining <= 0.0
                speed = clamp(2.0 * remaining, self.p['creep_speed'], self.p['enter_speed'])
            else:
                speed = self.p['enter_speed']
            if done:
                self.go('hold', ctx)
                self.t_hold = ctx.now
            else:
                w = self.slot_steer(ctx) if (s is not None and s.slot_valid) else 0.0
                return Cmd(speed, w)

        if self.sub == 'hold':
            if ctx.now - self.t_hold >= self.p['final_hold_s']:
                self.finish('ok')
            return STOP
        return STOP
