from dataclasses import dataclass
import math

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
            raise ValueError('速度必须为有限数')
        result = []
        for index, (x, y) in enumerate(self.positions, 1):
            wheel_vx = twist.vx - twist.wz * y
            wheel_vy = twist.vy + twist.wz * x
            speed = math.hypot(wheel_vx, wheel_vy)
            angle = math.atan2(wheel_vy, wheel_vx) if speed > 1e-12 else 0.0
            result.append(ModuleTarget(index, angle, speed))
        return tuple(result)
