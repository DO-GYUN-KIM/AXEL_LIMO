"""Twist(선속도 v, 각속도 w) → 애커먼 조향각 변환 계산입니다. ROS 없이 시험할 수 있게 따로 둡니다.

자전거 모델: w = v * tan(조향각) / 축거  →  조향각 = atan(축거 * w / v)
LIMO 위키 기준 값: 축거 0.2 m, 조향 한계 ±0.42 rad, 최소 회전 반경 0.4 m.
"""
import math


def twist_to_ackermann(v, w, wheelbase=0.2, max_steer=0.42, min_speed=0.01):
    """(speed, steering_angle, saturated)를 돌려줍니다.

    - 정지(|v| < min_speed)면 제자리 회전을 할 수 없으므로 속도 0, 조향은 w 방향 끝까지 둡니다(바퀴만 미리 꺾음).
    - 후진(v < 0)도 같은 식이 맞습니다(atan의 부호가 같이 바뀜).
    - 조향이 한계를 넘으면 잘라내고 saturated=True를 돌려줍니다.
    """
    if abs(v) < min_speed:
        if abs(w) < 1e-6:
            return 0.0, 0.0, False
        return 0.0, math.copysign(max_steer, w), False
    steer = math.atan(wheelbase * w / v)
    if abs(steer) > max_steer:
        return v, math.copysign(max_steer, steer), True
    return v, steer, False


def max_angular(v, wheelbase=0.2, max_steer=0.42):
    """속도 v에서 낼 수 있는 최대 각속도(rad/s)입니다."""
    return abs(v) * math.tan(max_steer) / wheelbase
