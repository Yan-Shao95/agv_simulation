"""带动作准备阶段的四差动总成、八电机直接控制器。"""
import math
import time

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState
    from agv_interfaces.msg import MotorCommandArray
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object

from .angle_control import angle_velocity, normalize_angle, optimize_target, drive_scale, rate_limit
from .kinematics import ChassisTwist, SwerveKinematics
from .module_mixer import mix_module

POSITIONS = ((0.45, 0.325), (0.45, -0.325), (-0.45, -0.325), (-0.45, 0.325))


class PreparedMotorControl:
    """先调整四个总成方向，再输出八个独立电机速度。"""

    def __init__(self, positions=POSITIONS, wheel_radius=0.1, half_track=0.05,
                 kp=4.0, max_steering_velocity=3.0,
                 steering_tolerance=math.radians(2.0), max_motor_velocity=30.0,
                 acceleration=0.15, deceleration=0.5, steering_ki=1.0):
        self.kinematics = SwerveKinematics(positions)
        self.wheel_radius = wheel_radius
        self.half_track = half_track
        self.kp = kp
        self.max_steering_velocity = max_steering_velocity
        self.steering_tolerance = steering_tolerance
        self.max_motor_velocity = max_motor_velocity
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.steering_ki = steering_ki
        self.alignment_integral = [0.0] * 4
        self.previous_errors = [0.0] * 4
        self.speeds = [0.0] * 4
        self.ready = False

    def calculate(self, twist, angles, dt=0.02):
        if len(angles) != 4:
            raise ValueError('需要四个总成转角')
        values = (twist.vx, twist.vy, twist.wz, dt, *angles)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('控制输入必须为有限数')
        if dt < 0.0:
            raise ValueError('控制周期必须非负')
        if max(abs(twist.vx), abs(twist.vy), abs(twist.wz)) < 1e-6:
            self.speeds = [0.0] * 4
            self.ready = False
            self.alignment_integral = [0.0] * 4
            self.previous_errors = [0.0] * 4
            return [0.0] * 8, True

        targets = self.kinematics.inverse(twist)
        optimized = [optimize_target(target.steering_angle,
                                     target.drive_velocity, angles[index])
                     for index, target in enumerate(targets)]
        max_error = max(abs(normalize_angle(target_angle - angles[index]))
                        for index, (target_angle, _) in enumerate(optimized))
        # 对正后持续纠偏；较宽的退出阈值避免在2度边界反复启停。
        leave_tolerance = max(math.radians(12.0), self.steering_tolerance)
        self.ready = max_error <= (leave_tolerance if self.ready else self.steering_tolerance)
        scale = drive_scale(max_error, math.radians(4.0), leave_tolerance) if self.ready else 0.0

        motors = []
        for index, (target_angle, drive_velocity) in enumerate(optimized):
            # 未对正时减速，并以限幅积分克服接触造成的残余转角误差。
            error = normalize_angle(target_angle - angles[index])
            if self.ready or error * self.previous_errors[index] <= 0.0:
                self.alignment_integral[index] = 0.0
            if not self.ready:
                self.alignment_integral[index] = max(-0.6, min(
                    0.6, self.alignment_integral[index] + self.steering_ki * error * dt))
            self.previous_errors[index] = error
            steering_velocity = angle_velocity(
                target_angle, angles[index], self.kp, self.max_steering_velocity)
            steering_velocity = max(-self.max_steering_velocity, min(
                self.max_steering_velocity, steering_velocity + self.alignment_integral[index]))
            desired = drive_velocity * scale
            delta = (self.acceleration if abs(desired) > abs(self.speeds[index])
                     else self.deceleration) * dt
            self.speeds[index] = rate_limit(desired, self.speeds[index], delta)
            # 轮组相对地面的角速度还包含车体自转；仅用舵角误差会漏掉内外轮速差。
            if abs(drive_velocity) > 1e-9:
                steering_velocity += twist.wz * self.speeds[index] / drive_velocity
            drive_velocity = self.speeds[index]
            motors.extend(mix_module(drive_velocity, steering_velocity,
                                     self.wheel_radius, self.half_track, 1.0, -1.0))
        return [max(-self.max_motor_velocity, min(self.max_motor_velocity, value))
                for value in motors], self.ready


class DirectMotorController(Node):
    def __init__(self):
        super().__init__('direct_motor_controller')
        self.angles = [0.0] * 4
        self.have_joint_state = False
        self.twist = ChassisTwist(0.0, 0.0, 0.0)
        self.last_command = 0.0
        self.timeout = self.declare_parameter('command_timeout', 0.5).value
        self.control = PreparedMotorControl(
            wheel_radius=self.declare_parameter('wheel_radius', 0.1).value,
            half_track=self.declare_parameter('module_half_track', 0.05).value,
            kp=self.declare_parameter('kp', 4.0).value,
            max_steering_velocity=self.declare_parameter('max_steering_velocity', 3.0).value,
            steering_tolerance=math.radians(
                self.declare_parameter('steering_tolerance_deg', 2.0).value),
            max_motor_velocity=self.declare_parameter('max_motor_velocity', 30.0).value,
            acceleration=self.declare_parameter('max_drive_acceleration', 0.15).value,
            deceleration=self.declare_parameter('max_drive_deceleration', 0.5).value,
            steering_ki=self.declare_parameter('steering_ki', 1.0).value)
        self.last_update = self.get_clock().now()
        self.publisher = self.create_publisher(
            MotorCommandArray, 'drive/motor_velocity_target', 10)
        self.create_subscription(Twist, 'drive/twist_command', self._twist, 10)
        self.create_subscription(JointState, 'joint_states', self._joints, 10)
        self.create_timer(0.02, self._update)

    def _twist(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if all(math.isfinite(value) for value in values):
            self.twist = ChassisTwist(*values)
            self.last_command = time.monotonic()

    def _joints(self, msg):
        by_name = dict(zip(msg.name, msg.position))
        names = [f'module_{index}_steering_joint' for index in range(1, 5)]
        if all(name in by_name for name in names):
            self.angles = [by_name[name] for name in names]
            self.have_joint_state = True

    def _update(self):
        now = self.get_clock().now()
        dt = max(0.0, min((now - self.last_update).nanoseconds / 1e9, 0.2))
        self.last_update = now
        stopped = time.monotonic() - self.last_command > self.timeout
        twist = ChassisTwist(0.0, 0.0, 0.0) if stopped else self.twist
        velocity = [0.0] * 8
        if self.have_joint_state:
            velocity, _ = self.control.calculate(twist, self.angles, dt)
        self.publisher.publish(MotorCommandArray(velocity=velocity))


def main():
    if rclpy is None:
        raise RuntimeError('需要 ROS 2 Jazzy 环境')
    rclpy.init()
    node = DirectMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
