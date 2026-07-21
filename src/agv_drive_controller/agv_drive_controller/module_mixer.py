import math

def mix_module(drive_velocity, steering_velocity, wheel_radius, half_track,
               left_sign=1.0, right_sign=-1.0):
    values = (drive_velocity, steering_velocity, wheel_radius, half_track, left_sign, right_sign)
    if not all(math.isfinite(v) for v in values) or wheel_radius <= 0 or half_track <= 0:
        raise ValueError('混控参数无效')
    left_surface = drive_velocity - steering_velocity * half_track
    right_surface = drive_velocity + steering_velocity * half_track
    return left_sign * left_surface / wheel_radius, right_sign * right_surface / wheel_radius
