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

from .kinematics import ChassisTwist, SwerveKinematics
from .angle_control import angle_velocity, optimize_target
from .module_mixer import mix_module

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
        self.kin=SwerveKinematics(POSITIONS); self.pub=self.create_publisher(ModuleCommandArray,'drive/module_target',10)
        self.create_subscription(Twist,'drive/twist_command',self._callback,10)
    def _callback(self,msg):
        from agv_interfaces.msg import ModuleCommand
        targets=self.kin.inverse(ChassisTwist(msg.linear.x,msg.linear.y,msg.angular.z))
        self.pub.publish(ModuleCommandArray(modules=[ModuleCommand(module_id=t.module_id,steering_angle=t.steering_angle,drive_velocity=t.drive_velocity) for t in targets]))

class ModuleAngleController(Node):
    def __init__(self):
        super().__init__('module_angle_controller'); self.angles=[0.0]*4; self.targets=None
        self.kp=self.declare_parameter('kp',4.0).value; self.maximum=self.declare_parameter('max_steering_velocity',3.0).value
        self.pub=self.create_publisher(ModuleCommandArray,'drive/module_velocity_target',10)
        self.create_subscription(JointState,'joint_states',self._joints,10); self.create_subscription(ModuleCommandArray,'drive/module_target',self._targets,10)
    def _joints(self,msg):
        by_name=dict(zip(msg.name,msg.position)); self.angles=[by_name.get(f'module_{i}_steering_joint',self.angles[i-1]) for i in range(1,5)]
    def _targets(self,msg):
        from agv_interfaces.msg import ModuleCommand
        out=[]
        for i,t in enumerate(msg.modules):
            angle,speed=optimize_target(t.steering_angle,t.drive_velocity,self.angles[i])
            out.append(ModuleCommand(module_id=i+1,steering_angle=angle_velocity(angle,self.angles[i],self.kp,self.maximum),drive_velocity=speed))
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
