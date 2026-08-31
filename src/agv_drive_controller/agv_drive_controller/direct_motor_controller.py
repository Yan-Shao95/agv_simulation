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

from .angle_control import angle_velocity, normalize_angle, optimize_target
from .kinematics import ChassisTwist, SwerveKinematics
from .module_mixer import mix_module

POSITIONS = ((0.45, 0.325), (0.45, -0.325), (-0.45, -0.325), (-0.45, 0.325))


class PreparedMotorControl:
    """先调整四个总成方向，再输出八个独立电机速度。"""

    def __init__(self, positions=POSITIONS, wheel_radius=0.1, half_track=0.05,
                 kp=4.0, max_steering_velocity=3.0,
                 steering_tolerance=math.radians(2.0), max_motor_velocity=30.0):
        self.kinematics = SwerveKinematics(positions)
        self.wheel_radius = wheel_radius
        self.half_track = half_track
        self.kp = kp
        self.max_steering_velocity = max_steering_velocity
        self.steering_tolerance = steering_tolerance
        self.max_motor_velocity = max_motor_velocity

    def calculate(self, twist, angles):
        if len(angles) != 4:
            raise ValueError('需要四个总成转角')
        values = (twist.vx, twist.vy, twist.wz, *angles)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('控制输入必须为有限数')
        if max(abs(twist.vx), abs(twist.vy), abs(twist.wz)) < 1e-6:
            return [0.0] * 8, True

        targets = self.kinematics.inverse(twist)
        optimized = [optimize_target(target.steering_angle,
                                     target.drive_velocity, angles[index])
                     for index, target in enumerate(targets)]
        ready = all(abs(normalize_angle(target_angle - angles[index]))
                    <= self.steering_tolerance
                    for index, (target_angle, _) in enumerate(optimized))

        motors = []
        for index, (target_angle, drive_velocity) in enumerate(optimized):
            # 准备阶段只让左右轮差动，以免轮组未对正时车体窜动。
            steering_velocity = 0.0 if ready else angle_velocity(
                target_angle, angles[index], self.kp, self.max_steering_velocity)
            drive_velocity = drive_velocity if ready else 0.0
            motors.extend(mix_module(drive_velocity, steering_velocity,
                                     self.wheel_radius, self.half_track, 1.0, -1.0))
        return [max(-self.max_motor_velocity, min(self.max_motor_velocity, value))
                for value in motors], ready


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
            max_motor_velocity=self.declare_parameter('max_motor_velocity', 30.0).value)
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
        stopped = time.monotonic() - self.last_command > self.timeout
        twist = ChassisTwist(0.0, 0.0, 0.0) if stopped else self.twist
        velocity = [0.0] * 8
        if self.have_joint_state:
            velocity, _ = self.control.calculate(twist, self.angles)
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
