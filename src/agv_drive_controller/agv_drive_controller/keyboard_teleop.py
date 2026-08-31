"""通过交互式终端向 AGV 发布速度指令。"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


MOTION_KEYS = {
    'q': (1.0, 1.0, 0.0),
    'w': (1.0, 0.0, 0.0),
    'e': (1.0, -1.0, 0.0),
    'a': (0.0, 1.0, 0.0),
    'd': (0.0, -1.0, 0.0),
    'z': (-1.0, 1.0, 0.0),
    'x': (-1.0, 0.0, 0.0),
    'c': (-1.0, -1.0, 0.0),
    'f': (0.0, 0.0, 1.0),
    'g': (0.0, 0.0, -1.0),
}

HELP = """
AGV 键盘控制（请保持此终端处于焦点）
---------------------------------------
  q  w  e       左前 / 前进 / 右前
  a  s  d       左移 / 减速停车 / 右移
  z  x  c       左后 / 后退 / 右后

  f / g         左转 / 右转
  + / -         增加 / 减小线速度
  ] / [         增加 / 减小角速度
  空格          急速停车
  Ctrl-C        停车并退出
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('key_timeout', 0.25)
        self.declare_parameter('speed_step', 0.1)
        self.declare_parameter('brake_factor', 0.6)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.key_timeout = float(self.get_parameter('key_timeout').value)
        self.speed_step = float(self.get_parameter('speed_step').value)
        self.brake_factor = float(self.get_parameter('brake_factor').value)
        publish_rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        if not sys.stdin.isatty():
            raise RuntimeError('键盘控制需要交互式终端（TTY）')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.motion = (0.0, 0.0, 0.0)
        self.last_key_time = self.get_clock().now()
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.timer = self.create_timer(1.0 / publish_rate, self._tick)
        print(HELP)
        self._print_speed()

    def _print_speed(self):
        print(f'线速度: {self.linear_speed:.2f} m/s | '
              f'角速度: {self.angular_speed:.2f} rad/s', flush=True)

    def _read_key(self):
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        return sys.stdin.read(1).lower() if ready else None

    def _tick(self):
        key = self._read_key()
        if key == '\x03':
            raise KeyboardInterrupt
        if key in MOTION_KEYS:
            self.motion = MOTION_KEYS[key]
            self.last_key_time = self.get_clock().now()
        elif key == 's':
            self.motion = tuple(value * self.brake_factor for value in self.motion)
            self.last_key_time = self.get_clock().now()
            if max(abs(value) for value in self.motion) < 0.05:
                self.motion = (0.0, 0.0, 0.0)
        elif key == ' ':
            self.motion = (0.0, 0.0, 0.0)
        elif key in ('+', '='):
            self.linear_speed += self.speed_step
            self._print_speed()
        elif key == '-':
            self.linear_speed = max(0.0, self.linear_speed - self.speed_step)
            self._print_speed()
        elif key == ']':
            self.angular_speed += self.speed_step
            self._print_speed()
        elif key == '[':
            self.angular_speed = max(0.0, self.angular_speed - self.speed_step)
            self._print_speed()

        age = (self.get_clock().now() - self.last_key_time).nanoseconds / 1e9
        motion = self.motion if age <= self.key_timeout else (0.0, 0.0, 0.0)
        msg = Twist()
        msg.linear.x = motion[0] * self.linear_speed
        msg.linear.y = motion[1] * self.linear_speed
        msg.angular.z = motion[2] * self.angular_speed
        self.publisher.publish(msg)

    def stop(self):
        self.publisher.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleop()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as exc:
        if str(exc):
            print(exc, file=sys.stderr)
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
