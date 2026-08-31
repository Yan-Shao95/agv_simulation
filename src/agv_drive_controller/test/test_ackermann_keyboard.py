import sys
import unittest
from pathlib import Path


PACKAGE = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE))

from agv_drive_controller.ackermann_keyboard import AckermannKeyboardState, make_twist


class AckermannKeyboardStateTest(unittest.TestCase):
    def setUp(self):
        self.state = AckermannKeyboardState()

    def test_steering_step_preserves_forward_speed(self):
        self.state.handle_key('i', 0.0)
        self.state.handle_key('j', 0.1)
        self.assertEqual(self.state.command(0.1), (0.5, 0.1))

    def test_steering_is_limited_in_both_directions(self):
        for index in range(20):
            self.state.handle_key('j', index * 0.01)
        self.assertEqual(self.state.command(0.2)[1], 1.0)
        for index in range(30):
            self.state.handle_key('l', 1.0 + index * 0.01)
        self.assertEqual(self.state.command(1.3)[1], -1.0)

    def test_steering_timeout_returns_to_zero_without_stopping(self):
        self.state.handle_key('i', 0.0)
        self.state.handle_key('j', 0.1)
        self.assertEqual(self.state.command(0.249), (0.5, 0.1))
        self.assertEqual(self.state.command(0.251), (0.5, 0.0))

    def test_reverse_speed_is_held_while_steering(self):
        self.state.handle_key(',', 0.0)
        self.state.handle_key('l', 0.1)
        self.assertEqual(self.state.command(0.1), (-0.5, -0.1))

    def test_stop_clears_speed_and_steering(self):
        self.state.handle_key('i', 0.0)
        self.state.handle_key('j', 0.1)
        self.state.handle_key('k', 0.2)
        self.assertEqual(self.state.command(0.2), (0.0, 0.0))

    def test_unknown_key_does_not_change_state(self):
        self.state.handle_key('i', 0.0)
        self.state.handle_key('x', 0.1)
        self.assertEqual(self.state.command(0.1), (0.5, 0.0))

    def test_ctrl_c_requests_exit_and_clears_command(self):
        self.state.handle_key('i', 0.0)
        self.assertTrue(self.state.handle_key('\x03', 0.1))
        self.assertEqual(self.state.command(0.1), (0.0, 0.0))

    def test_zero_twist_is_safe_for_shutdown(self):
        message = make_twist(0.0, 0.0)
        self.assertEqual(message.linear.x, 0.0)
        self.assertEqual(message.angular.z, 0.0)

    def test_package_installs_ackermann_keyboard_entry_point(self):
        setup_text = (PACKAGE / 'setup.py').read_text()
        self.assertIn(
            'ackermann_keyboard=agv_drive_controller.ackermann_keyboard:main',
            setup_text)

    def test_ctrl_c_keeps_ros_context_alive_until_stop_is_published(self):
        source = (
            PACKAGE / 'agv_drive_controller/ackermann_keyboard.py').read_text()
        self.assertIn('SignalHandlerOptions.NO', source)
        self.assertIn('except KeyboardInterrupt:', source)


if __name__ == '__main__':
    unittest.main()
