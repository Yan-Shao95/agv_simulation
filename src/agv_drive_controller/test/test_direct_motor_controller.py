import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))

from agv_drive_controller.direct_motor_controller import PreparedMotorControl
from agv_drive_controller.kinematics import ChassisTwist


class DirectMotorControllerTest(unittest.TestCase):
    def setUp(self):
        self.control = PreparedMotorControl()

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
        self.assertTrue(all(math.isclose(motors[index], -motors[index + 1])
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

    def test_zero_command_stops_without_repositioning(self):
        motors, ready = self.control.calculate(
            ChassisTwist(0.0, 0.0, 0.0), [1.0, -1.0, 0.5, -0.5])
        self.assertTrue(ready)
        self.assertEqual(motors, [0.0] * 8)


if __name__ == '__main__':
    unittest.main()
