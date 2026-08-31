import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions


HELP = '''
阿克曼状态保持式键盘控制
---------------------------
i : 保持前进       , : 保持后退
j : 左转 +0.1      l : 右转 -0.1
k : 停车并回正

松开 j/l 后转向自动回正，行驶速度继续保持。
Ctrl-C 退出。
'''


class AckermannKeyboardState:
    def __init__(self, speed=0.5, steering_step=0.1,
                 steering_limit=1.0, steering_timeout=0.15):
        self.speed = float(speed)
        self.steering_step = float(steering_step)
        self.steering_limit = float(steering_limit)
        self.steering_timeout = float(steering_timeout)
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_steering_time = None

    def handle_key(self, key, now):
        if key == 'i':
            self.linear_x = self.speed
        elif key == ',':
            self.linear_x = -self.speed
        elif key == 'j':
            self.angular_z = min(
                self.steering_limit, self.angular_z + self.steering_step)
            self.last_steering_time = now
        elif key == 'l':
            self.angular_z = max(
                -self.steering_limit, self.angular_z - self.steering_step)
            self.last_steering_time = now
        elif key == 'k' or key == '\x03':
            self.linear_x = 0.0
            self.angular_z = 0.0
            self.last_steering_time = None
        return key == '\x03'

    def command(self, now):
        if (self.last_steering_time is not None and
                now - self.last_steering_time > self.steering_timeout):
            self.angular_z = 0.0
            self.last_steering_time = None
        return self.linear_x, self.angular_z


def make_twist(linear_x, angular_z):
    message = Twist()
    message.linear.x = float(linear_x)
    message.angular.z = float(angular_z)
    return message


def main():
    terminal_settings = termios.tcgetattr(sys.stdin.fileno())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('ackermann_keyboard')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    state = AckermannKeyboardState(
        speed=node.declare_parameter('speed', 0.5).value,
        steering_step=node.declare_parameter('steering_step', 0.1).value,
        steering_limit=node.declare_parameter('steering_limit', 1.0).value,
        steering_timeout=node.declare_parameter('steering_timeout', 0.15).value,
    )
    publish_rate = float(node.declare_parameter('publish_rate', 20.0).value)
    publish_period = 1.0 / publish_rate
    next_publish = time.monotonic()

    print(HELP, flush=True)
    tty.setcbreak(sys.stdin.fileno())
    try:
        should_exit = False
        while rclpy.ok() and not should_exit:
            now = time.monotonic()
            timeout = max(0.0, min(0.02, next_publish - now))
            readable, _, _ = select.select([sys.stdin], [], [], timeout)
            if readable:
                should_exit = state.handle_key(sys.stdin.read(1), time.monotonic())

            now = time.monotonic()
            if now >= next_publish:
                linear_x, angular_z = state.command(now)
                publisher.publish(make_twist(linear_x, angular_z))
                next_publish = now + publish_period
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.publish(make_twist(0.0, 0.0))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        node.destroy_node()
        rclpy.shutdown()
