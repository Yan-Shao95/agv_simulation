import math
import sys
import unittest
from pathlib import Path


PACKAGE = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE))

from agv_drive_controller.periodic_angle_controller import compute_periodic_commands


PARAMETERS = dict(
    kp=4.0,
    low_limit=0.6,
    high_limit=3.0,
    slowdown_error=math.radians(4.0),
    stop_error=math.radians(12.0),
    acceleration_delta=0.0015,
    deceleration_delta=0.005,
)


class PeriodicAngleControllerTest(unittest.TestCase):
    def test_ros_node_is_configured_as_periodic_closed_loop(self):
        config = (PACKAGE / 'config/controller.yaml').read_text()
        node_source = (PACKAGE / 'agv_drive_controller/nodes.py').read_text()
        self.assertIn('control_rate_hz: 100.0', config)
        self.assertIn("self.create_timer(1.0/self.control_rate,self._control)", node_source)
        self.assertIn('def _control(self):', node_source)
        targets_body = node_source.split('def _targets(self,msg):', 1)[1].split(
            'def _control(self):', 1)[0]
        self.assertNotIn('self.pub.publish', targets_body)

    def test_command_reverses_immediately_after_crossing_target(self):
        targets = [(0.0, 0.0)] * 4
        before, _ = compute_periodic_commands(
            targets, [-0.5] * 4, [0.0] * 4, **PARAMETERS)
        after, _ = compute_periodic_commands(
            targets, [0.2] * 4, [0.0] * 4, **PARAMETERS)
        self.assertTrue(all(command > 0.0 for command in before))
        self.assertTrue(all(command < 0.0 for command in after))

    def test_missing_target_or_joint_state_is_safe_zero(self):
        self.assertEqual(
            compute_periodic_commands(None, [0.0] * 4, [0.0] * 4, **PARAMETERS),
            ([0.0] * 4, [0.0] * 4))
        self.assertEqual(
            compute_periodic_commands([(0.0, 0.2)] * 4, None, [0.0] * 4, **PARAMETERS),
            ([0.0] * 4, [0.0] * 4))

    def test_invalid_module_count_is_rejected(self):
        with self.assertRaises(ValueError):
            compute_periodic_commands(
                [(0.0, 0.2)] * 3, [0.0] * 4, [0.0] * 4, **PARAMETERS)

    def test_non_finite_input_is_rejected(self):
        with self.assertRaises(ValueError):
            compute_periodic_commands(
                [(0.0, 0.2)] * 4,
                [0.0, 0.0, 0.0, math.nan],
                [0.0] * 4,
                **PARAMETERS)

    def test_all_drive_modules_share_worst_angle_scale(self):
        steering, drive = compute_periodic_commands(
            [(0.0, 0.3)] * 4,
            [0.0, 0.0, 0.0, math.radians(8.0)],
            [0.2] * 4,
            **PARAMETERS)
        self.assertEqual(len(steering), 4)
        self.assertEqual(drive, [0.195] * 4)


if __name__ == '__main__':
    unittest.main()
