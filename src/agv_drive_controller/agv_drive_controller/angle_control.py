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


def bounded_angle_velocity(target, current, kp, max_velocity):
    if not all(math.isfinite(value) for value in (target, current, kp, max_velocity)):
        raise ValueError('控制参数必须为有限数')
    if abs(target) > math.pi / 2 + 1e-12:
        raise ValueError('目标舵角必须在正负90度内')
    command = kp * (target - current)
    return max(-max_velocity, min(max_velocity, command))

def rate_limit(target, current, maximum_delta):
    values = (target, current, maximum_delta)
    if not all(math.isfinite(value) for value in values) or maximum_delta < 0:
        raise ValueError('限速参数必须有效')
    delta = max(-maximum_delta, min(maximum_delta, target - current))
    return current + delta


def safe_drive_velocity(target, current, angle_error, ready_tolerance,
                        abort_tolerance, acceleration_delta, deceleration_delta):
    values = (target, current, angle_error, ready_tolerance, abort_tolerance,
              acceleration_delta, deceleration_delta)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('安全驱动参数必须为有限数')
    if ready_tolerance < 0 or abort_tolerance < ready_tolerance:
        raise ValueError('角度容差无效')
    if abs(angle_error) >= abort_tolerance:
        return 0.0
    desired = target if abs(angle_error) <= ready_tolerance else 0.0
    if target * current < 0.0:
        desired = 0.0
    speeding_up = abs(desired) > abs(current)
    maximum_delta = acceleration_delta if speeding_up else deceleration_delta
    return rate_limit(desired, current, maximum_delta)


def coordinated_speeds(targets, currents, aligned, acceleration_delta, deceleration_delta):
    targets = tuple(targets)
    currents = tuple(currents)
    if len(targets) != 4 or len(currents) != 4:
        raise ValueError('必须提供四组驱动速度')
    if not aligned:
        return [0.0] * 4
    return [
        rate_limit(
            target, current,
            acceleration_delta if abs(target) > abs(current) else deceleration_delta)
        for target, current in zip(targets, currents)
    ]

def alignment_ready(errors, was_ready, enter_tolerance, leave_tolerance,
                    abort_tolerance):
    errors = tuple(abs(error) for error in errors)
    values = errors + (enter_tolerance, leave_tolerance, abort_tolerance)
    if not errors or not all(math.isfinite(value) for value in values):
        raise ValueError('对正参数必须为有限数')
    if not 0 <= enter_tolerance <= leave_tolerance <= abort_tolerance:
        raise ValueError('对正容差顺序无效')
    limit = leave_tolerance if was_ready else enter_tolerance
    return all(error <= limit for error in errors)
