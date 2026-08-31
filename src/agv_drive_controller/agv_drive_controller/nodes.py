"""可独立运行的 ROS 2 控制节点。ROS 不存在时仍允许导入纯数学模块。"""
import math
import time

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    from agv_interfaces.msg import ModuleCommandArray, MotorCommandArray, DriveStatus
    from agv_interfaces.srv import SetControlMode
except ImportError:  # pragma: no cover - 仅用于非 ROS 开发机静态检查
    rclpy = None
    Node = object

from .kinematics import ChassisTwist, ModuleTarget, SwerveKinematics, stop_targets
from .angle_control import (
    adaptive_steering_limit,
    bounded_angle_velocity,
    coordinated_speeds,
    drive_scale,
    validate_lock_parameters,
)
from .module_mixer import mix_module
from .periodic_angle_controller import compute_periodic_commands

POSITIONS = ((0.45, 0.325), (0.45, -0.325), (-0.45, -0.325), (-0.45, 0.325))

class CommandArbiter(Node):
    def __init__(self):
        super().__init__('command_arbiter')
        self.mode = DriveStatus.STOP
        self.timeout = self.declare_parameter('command_timeout', 0.5).value
        self.last_command = time.monotonic()
        self.twist_pub = self.create_publisher(Twist, 'drive/twist_command', 10)
        self.module_pub = self.create_publisher(ModuleCommandArray, 'drive/module_command_selected', 10)
        self.motor_pub = self.create_publisher(MotorCommandArray, 'drive/motor_command_selected', 10)
        self.create_subscription(Twist, 'cmd_vel', self._twist, 10)
        self.create_subscription(ModuleCommandArray, 'drive/module_command_direct', self._module, 10)
        self.create_subscription(MotorCommandArray, 'drive/motor_command_direct', self._motor, 10)
        self.create_service(SetControlMode, 'drive/set_control_mode', self._set_mode)
        self.create_timer(0.05, self._watchdog)
    def _finite(self, values): return all(math.isfinite(v) for v in values)
    def _twist(self, msg):
        values=(msg.linear.x,msg.linear.y,msg.angular.z)
        if self.mode == DriveStatus.TWIST and self._finite(values): self.last_command=time.monotonic(); self.twist_pub.publish(msg)
    def _module(self, msg):
        if self.mode == DriveStatus.MODULE: self.last_command=time.monotonic(); self.module_pub.publish(msg)
    def _motor(self, msg):
        if self.mode == DriveStatus.MOTOR and self._finite(msg.velocity): self.last_command=time.monotonic(); self.motor_pub.publish(msg)
    def _set_mode(self, req, res):
        if req.mode not in (0,1,2,3): res.success=False; res.message='无效模式'; return res
        self._stop(); self.mode=req.mode; self.last_command=time.monotonic(); res.success=True; res.message='模式已切换'; return res
    def _stop(self): self.motor_pub.publish(MotorCommandArray(velocity=[0.0]*8))
    def _watchdog(self):
        if self.mode != DriveStatus.STOP and time.monotonic()-self.last_command > self.timeout: self._stop()

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('swerve_kinematics')
        self.kin=SwerveKinematics(POSITIONS); self.last_targets=tuple(ModuleTarget(i, 0.0, 0.0) for i in range(1, 5)); self.pub=self.create_publisher(ModuleCommandArray,'drive/module_target',10)
        self.create_subscription(Twist,'drive/twist_command',self._callback,10)
    def _callback(self,msg):
        from agv_interfaces.msg import ModuleCommand
        twist=ChassisTwist(msg.linear.x,msg.linear.y,msg.angular.z)
        if abs(twist.vx) < 1e-12 and abs(twist.vy) < 1e-12 and abs(twist.wz) < 1e-12:
            targets=stop_targets(self.last_targets)
        else:
            targets=self.kin.inverse(twist)
            self.last_targets=targets
        self.pub.publish(ModuleCommandArray(modules=[ModuleCommand(module_id=t.module_id,steering_angle=t.steering_angle,drive_velocity=t.drive_velocity) for t in targets]))

class ModuleAngleController(Node):
    def __init__(self):
        super().__init__('module_angle_controller'); self.angles=[0.0]*4; self.targets=None; self.angles_valid=False
        self.kp=float(self.declare_parameter('kp',4.0).value)
        self.low_maximum=float(self.declare_parameter('low_steering_velocity',0.6).value)
        self.high_maximum=float(self.declare_parameter('high_steering_velocity',3.0).value)
        self.slowdown_error=math.radians(float(self.declare_parameter('slowdown_error_deg',4.0).value))
        self.stop_error=math.radians(float(self.declare_parameter('stop_error_deg',12.0).value))
        validate_lock_parameters(
            self.kp,self.low_maximum,self.high_maximum,
            self.slowdown_error,self.stop_error)
        self.acceleration=float(self.declare_parameter('max_drive_acceleration',0.15).value)
        self.deceleration=float(self.declare_parameter('max_drive_deceleration',0.50).value)
        self.control_rate=float(self.declare_parameter('control_rate_hz',100.0).value)
        if not math.isfinite(self.control_rate) or self.control_rate <= 0.0:
            raise ValueError('控制频率必须为正有限数')
        self.speeds=[0.0]*4
        self.last_update=time.monotonic()
        self.pub=self.create_publisher(ModuleCommandArray,'drive/module_velocity_target',10)
        self.create_subscription(JointState,'joint_states',self._joints,10); self.create_subscription(ModuleCommandArray,'drive/module_target',self._targets,10)
        self.create_subscription(MotorCommandArray,'drive/motor_command_selected',self._motor_override,10)
        self.create_timer(1.0/self.control_rate,self._control)
    def _joints(self,msg):
        by_name=dict(zip(msg.name,msg.position))
        names=[f'module_{i}_steering_joint' for i in range(1,5)]
        if all(name in by_name and math.isfinite(by_name[name]) for name in names):
            self.angles=[by_name[name] for name in names]
            self.angles_valid=True
    def _motor_override(self,msg):
        if all(abs(speed)<1e-9 for speed in msg.velocity):
            self.speeds=[0.0]*4
            if self.targets is not None:
                self.targets=[(angle,0.0) for angle,_ in self.targets]
    def _targets(self,msg):
        if len(msg.modules) != 4:
            self.get_logger().error('模块目标必须恰好包含四组')
            return
        targets=[(target.steering_angle,target.drive_velocity) for target in msg.modules]
        if not all(math.isfinite(value) for target in targets for value in target):
            self.get_logger().error('模块目标必须为有限数')
            return
        self.targets=targets
    def _control(self):
        from agv_interfaces.msg import ModuleCommand
        now=time.monotonic()
        dt=max(0.0,min(now-self.last_update,0.2))
        self.last_update=now
        steering,self.speeds=compute_periodic_commands(
            self.targets,self.angles if self.angles_valid else None,self.speeds,
            kp=self.kp,low_limit=self.low_maximum,high_limit=self.high_maximum,
            slowdown_error=self.slowdown_error,stop_error=self.stop_error,
            acceleration_delta=self.acceleration*dt,
            deceleration_delta=self.deceleration*dt)
        out=[ModuleCommand(
            module_id=i+1,steering_angle=steering[i],drive_velocity=self.speeds[i])
            for i in range(4)]
        self.pub.publish(ModuleCommandArray(modules=out))

class DifferentialModuleMixer(Node):
    def __init__(self):
        super().__init__('differential_module_mixer'); self.pub=self.create_publisher(MotorCommandArray,'drive/motor_velocity_target',10)
        self.create_subscription(ModuleCommandArray,'drive/module_velocity_target',self._callback,10)
    def _callback(self,msg):
        values=[]
        for t in msg.modules: values.extend(mix_module(t.drive_velocity,t.steering_angle,0.1,0.05,1.0,-1.0))
        self.pub.publish(MotorCommandArray(velocity=values))

class MotorCommandBridge(Node):
    def __init__(self):
        super().__init__('motor_command_bridge'); self.pub=self.create_publisher(Float64MultiArray,'wheel_velocity_controller/commands',10)
        self.create_subscription(MotorCommandArray,'drive/motor_velocity_target',lambda m:self.pub.publish(Float64MultiArray(data=list(m.velocity))),10)
        self.create_subscription(MotorCommandArray,'drive/motor_command_selected',lambda m:self.pub.publish(Float64MultiArray(data=list(m.velocity))),10)

def run(cls):
    if rclpy is None: raise RuntimeError('需要 ROS 2 Jazzy 环境')
    rclpy.init(); node=cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
