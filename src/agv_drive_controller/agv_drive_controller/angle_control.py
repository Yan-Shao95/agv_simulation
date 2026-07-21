import math

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def optimize_target(target_angle, target_speed, current_angle):
    error = normalize_angle(target_angle - current_angle)
    if abs(error) > math.pi / 2:
        target_angle = normalize_angle(target_angle + math.pi)
        target_speed = -target_speed
    return target_angle, target_speed

def angle_velocity(target, current, kp, max_velocity):
    if not all(math.isfinite(v) for v in (target, current, kp, max_velocity)):
        raise ValueError('控制参数必须为有限数')
    command = kp * normalize_angle(target - current)
    return max(-max_velocity, min(max_velocity, command))
