import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))

from agv_drive_controller.direct_motor_controller import PreparedMotorControl
from agv_drive_controller.kinematics import ChassisTwist


class DirectMotorControllerTest(unittest.TestCase):
    def setUp(self):
        self.control = PreparedMotorControl(acceleration=100.0)

    def test_straight_outputs_eight_independent_motor_commands(self):
        motors, ready = self.control.calculate(ChassisTwist(0.3, 0.0, 0.0), [0.0] * 4)
        self.assertTrue(ready)
        self.assertEqual(len(motors), 8)
        for actual, expected in zip(motors, [3.0, -3.0] * 4):
            self.assertAlmostEqual(actual, expected)

    def test_lateral_prepares_modules_before_driving(self):
        motors, ready = self.control.calculate(ChassisTwist(0.0, 0.3, 0.0), [0.0] * 4)
        self.assertFalse(ready)
        self.assertTrue(all(abs(motors[index] - motors[index + 1]) < 1e-9
                            for index in range(0, 8, 2)))
        motors, ready = self.control.calculate(
            ChassisTwist(0.0, 0.3, 0.0), [math.pi / 2] * 4)
        self.assertTrue(ready)
        for actual, expected in zip(motors, [3.0, -3.0] * 4):
            self.assertAlmostEqual(actual, expected)

    def test_rotation_has_four_bounded_tangent_module_targets(self):
        twist = ChassisTwist(0.0, 0.0, 0.5)
        targets = self.control.kinematics.inverse(twist)
        angles = [target.steering_angle for target in targets]
        motors, ready = self.control.calculate(twist, angles)
        self.assertTrue(ready)
        self.assertEqual(len(motors), 8)
        self.assertTrue(all(abs(value) > 0.0 for value in motors))
        self.assertTrue(all(math.isclose(motors[index] + motors[index + 1], -0.5)
                            for index in range(0, 8, 2)))
        self.assertEqual(len(targets), 4)
        self.assertTrue(all(abs(angle) <= math.pi / 2 for angle in angles))
        for target, (x, y) in zip(targets, self.control.kinematics.positions):
            self.assertAlmostEqual(
                target.drive_velocity * math.cos(target.steering_angle),
                -twist.wz * y)
            self.assertAlmostEqual(
                target.drive_velocity * math.sin(target.steering_angle),
                twist.wz * x)

    def test_keeps_correcting_steering_while_driving(self):
        motors, ready = self.control.calculate(
            ChassisTwist(0.3, 0.0, 0.0), [math.radians(1)] * 4)
        self.assertTrue(ready)
        self.assertGreater(motors[0] + motors[1], 0.0)

    def test_alignment_hysteresis_avoids_drive_chatter(self):
        twist = ChassisTwist(0.3, 0.0, 0.0)
        self.control.calculate(twist, [0.0] * 4)
        motors, ready = self.control.calculate(twist, [math.radians(3)] * 4)
        self.assertTrue(ready)
        self.assertGreater(motors[0] - motors[1], 0.0)
        _, ready = self.control.calculate(twist, [math.radians(15)] * 4)
        self.assertFalse(ready)

    def test_stalled_alignment_gets_bounded_assistance(self):
        control = PreparedMotorControl()
        twist = ChassisTwist(0.2, 0.0, 0.0)
        angles = [math.radians(5.0)] * 4
        initial, ready = control.calculate(twist, angles, dt=0.1)
        self.assertFalse(ready)
        for _ in range(100):
            later, _ = control.calculate(twist, angles, dt=0.1)
        self.assertGreater(abs(later[0]), abs(initial[0]))
        self.assertTrue(all(abs(value) <= 0.6 for value in control.alignment_integral))
        control.calculate(twist, [0.0] * 4)
        self.assertEqual(control.alignment_integral, [0.0] * 4)

    def test_acceleration_uses_simulation_period(self):
        control = PreparedMotorControl()
        twist = ChassisTwist(0.3, 0.0, 0.0)
        motors, _ = control.calculate(twist, [0.0] * 4, dt=0.1)
        self.assertAlmostEqual((motors[0] - motors[1]) * 0.05, 0.015)
        unchanged, _ = control.calculate(twist, [0.0] * 4, dt=0.0)
        self.assertEqual(motors, unchanged)

    def test_zero_command_stops_without_repositioning(self):
        motors, ready = self.control.calculate(
            ChassisTwist(0.0, 0.0, 0.0), [1.0, -1.0, 0.5, -0.5])
        self.assertTrue(ready)
        self.assertEqual(motors, [0.0] * 8)


if __name__ == '__main__':
    unittest.main()
