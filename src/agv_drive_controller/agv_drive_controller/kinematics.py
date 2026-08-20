from dataclasses import dataclass
import math

EPSILON = 1e-12
LATERAL_ANGLES = (-math.pi / 2, math.pi / 2, -math.pi / 2, math.pi / 2)
LATERAL_SIGNS = (-1.0, 1.0, -1.0, 1.0)

def equivalent_target(angle, speed):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    if angle > math.pi / 2:
        return angle - math.pi, -speed
    if angle < -math.pi / 2:
        return angle + math.pi, -speed
    return angle, speed

def stop_targets(previous):
    return tuple(
        ModuleTarget(target.module_id, target.steering_angle, 0.0)
        for target in previous
    )

@dataclass(frozen=True)
class ChassisTwist:
    vx: float
    vy: float
    wz: float

@dataclass(frozen=True)
class ModuleTarget:
    module_id: int
    steering_angle: float
    drive_velocity: float

class SwerveKinematics:
    def __init__(self, module_positions):
        if len(module_positions) != 4:
            raise ValueError('必须配置四个驱动总成')
        self.positions = tuple(module_positions)

    def inverse(self, twist):
        if not all(math.isfinite(v) for v in (twist.vx, twist.vy, twist.wz)):
            raise ValueError("速度必须为有限数")
        if abs(twist.vx) <= EPSILON and abs(twist.vy) <= EPSILON and abs(twist.wz) <= EPSILON:
            return tuple(ModuleTarget(i, 0.0, 0.0) for i in range(1, 5))
        if abs(twist.vy) <= EPSILON and abs(twist.wz) <= EPSILON:
            return tuple(ModuleTarget(i, 0.0, twist.vx) for i in range(1, 5))
        if abs(twist.vx) <= EPSILON and abs(twist.wz) <= EPSILON:
            magnitude = abs(twist.vy)
            direction = 1.0 if twist.vy > 0.0 else -1.0
            return tuple(
                ModuleTarget(i, angle, magnitude * sign * direction)
                for i, (angle, sign) in enumerate(zip(LATERAL_ANGLES, LATERAL_SIGNS), 1)
            )
        result = []
        for index, (x, y) in enumerate(self.positions, 1):
            wheel_vx = twist.vx - twist.wz * y
            wheel_vy = twist.vy + twist.wz * x
            speed = math.hypot(wheel_vx, wheel_vy)
            angle = math.atan2(wheel_vy, wheel_vx) if speed > EPSILON else 0.0
            angle, speed = equivalent_target(angle, speed)
            result.append(ModuleTarget(index, angle, speed))
        return tuple(result)
