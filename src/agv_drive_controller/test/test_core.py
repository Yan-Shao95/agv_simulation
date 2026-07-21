import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))

from agv_drive_controller.kinematics import ChassisTwist, SwerveKinematics
from agv_drive_controller.module_mixer import mix_module
from agv_drive_controller.angle_control import optimize_target


class CoreTest(unittest.TestCase):
    def setUp(self):
        self.kin = SwerveKinematics(((0.45, 0.325), (0.45, -0.325), (-0.45, -0.325), (-0.45, 0.325)))

    def test_horizontal_motion_points_all_modules_left(self):
        targets = self.kin.inverse(ChassisTwist(0.0, 1.0, 0.0))
        self.assertTrue(all(math.isclose(t.steering_angle, math.pi / 2) for t in targets))
        self.assertTrue(all(math.isclose(t.drive_velocity, 1.0) for t in targets))

    def test_rotation_targets_are_tangent_to_module_radius(self):
        targets = self.kin.inverse(ChassisTwist(0.0, 0.0, 1.0))
        self.assertTrue(math.isclose(targets[0].steering_angle, math.atan2(0.45, -0.325)))

    def test_optimize_reverses_drive_for_shorter_steering_path(self):
        angle, speed = optimize_target(math.pi, 1.0, 0.0)
        self.assertLess(abs(angle), 1e-9)
        self.assertEqual(speed, -1.0)

    def test_physical_motor_signs_match_reverse_installation(self):
        left, right = mix_module(1.0, 0.0, 0.1, 0.05, 1.0, -1.0)
        self.assertEqual((left, right), (10.0, -10.0))
        left_turn, right_turn = mix_module(0.0, 1.0, 0.1, 0.05, 1.0, -1.0)
        self.assertEqual(left_turn, right_turn)


if __name__ == '__main__':
    unittest.main()
