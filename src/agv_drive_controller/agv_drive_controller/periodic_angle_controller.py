import math

from .angle_control import (
    adaptive_steering_limit,
    bounded_angle_velocity,
    coordinated_speeds,
    drive_scale,
    validate_lock_parameters,
)


def compute_periodic_commands(
        targets, angles, current_speeds, *, kp, low_limit, high_limit,
        slowdown_error, stop_error, acceleration_delta, deceleration_delta):
    if targets is None or angles is None:
        return [0.0] * 4, [0.0] * 4

    targets = tuple(targets)
    angles = tuple(angles)
    current_speeds = tuple(current_speeds)
    if len(targets) != 4 or len(angles) != 4 or len(current_speeds) != 4:
        raise ValueError('周期控制必须提供四组目标、角度和当前速度')

    flattened_targets = tuple(value for target in targets for value in target)
    values = flattened_targets + angles + current_speeds + (
        kp, low_limit, high_limit, slowdown_error, stop_error,
        acceleration_delta, deceleration_delta)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('周期控制输入必须为有限数')
    if acceleration_delta < 0.0 or deceleration_delta < 0.0:
        raise ValueError('周期控制加减速步长必须非负')

    validate_lock_parameters(
        kp, low_limit, high_limit, slowdown_error, stop_error)
    errors = [target_angle - angle for (target_angle, _), angle in zip(targets, angles)]
    scale = drive_scale(
        max(abs(error) for error in errors), slowdown_error, stop_error)
    drive_commands = coordinated_speeds(
        [speed for _, speed in targets], current_speeds, scale,
        acceleration_delta, deceleration_delta)
    steering_commands = []
    for (target_angle, _), angle, error in zip(targets, angles, errors):
        limit = adaptive_steering_limit(
            abs(error), low_limit, high_limit, slowdown_error, stop_error)
        steering_commands.append(
            bounded_angle_velocity(target_angle, angle, kp, limit))
    return steering_commands, drive_commands
