import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))

from agv_drive_controller.kinematics import ChassisTwist, SwerveKinematics, stop_targets
from agv_drive_controller.module_mixer import mix_module
from agv_drive_controller.angle_control import adaptive_steering_limit, alignment_ready, bounded_angle_velocity, coordinated_speeds, drive_scale, optimize_target, rate_limit, safe_drive_velocity, validate_lock_parameters


class CoreTest(unittest.TestCase):
    def setUp(self):
        self.kin = SwerveKinematics(((0.45, 0.325), (0.45, -0.325), (-0.45, -0.325), (-0.45, 0.325)))

    def test_heavy_vehicle_alignment_band_is_configured(self):
        config = (Path(__file__).parents[1] / 'config' / 'controller.yaml').read_text()
        self.assertIn('slowdown_error_deg: 4.0', config)
        self.assertIn('stop_error_deg: 12.0', config)

    def test_straight_modes_keep_all_modules_at_zero(self):
        forward = self.kin.inverse(ChassisTwist(0.5, 0.0, 0.0))
        reverse = self.kin.inverse(ChassisTwist(-0.5, 0.0, 0.0))
        self.assertEqual([t.steering_angle for t in forward], [0.0] * 4)
        self.assertEqual([t.drive_velocity for t in forward], [0.5] * 4)
        self.assertEqual([t.steering_angle for t in reverse], [0.0] * 4)
        self.assertEqual([t.drive_velocity for t in reverse], [-0.5] * 4)

    def test_lateral_modes_use_rotation_derived_signs(self):
        left = self.kin.inverse(ChassisTwist(0.0, 0.5, 0.0))
        right = self.kin.inverse(ChassisTwist(0.0, -0.5, 0.0))
        angles = [-math.pi / 2, math.pi / 2, -math.pi / 2, math.pi / 2]
        self.assertEqual([t.steering_angle for t in left], angles)
        self.assertEqual([t.drive_velocity for t in left], [-0.5, 0.5, -0.5, 0.5])
        self.assertEqual([t.steering_angle for t in right], angles)
        self.assertEqual([t.drive_velocity for t in right], [0.5, -0.5, 0.5, -0.5])

    def test_counterclockwise_rotation_uses_bounded_expected_angles(self):
        targets = self.kin.inverse(ChassisTwist(0.0, 0.0, 1.0))
        expected = [
            -math.atan2(0.45, 0.325),
            math.atan2(0.45, 0.325),
            -math.atan2(0.45, 0.325),
            math.atan2(0.45, 0.325),
        ]
        for target, angle in zip(targets, expected):
            self.assertAlmostEqual(target.steering_angle, angle)
            self.assertLessEqual(abs(target.steering_angle), math.pi / 2)
        self.assertEqual(
            [math.copysign(1.0, t.drive_velocity) for t in targets],
            [-1.0, 1.0, 1.0, -1.0],
        )

    def test_combined_motion_preserves_each_wheel_velocity_vector(self):
        twist = ChassisTwist(0.4, 0.2, 0.3)
        targets = self.kin.inverse(twist)
        for target, (x, y) in zip(targets, self.kin.positions):
            expected_vx = twist.vx - twist.wz * y
            expected_vy = twist.vy + twist.wz * x
            actual_vx = target.drive_velocity * math.cos(target.steering_angle)
            actual_vy = target.drive_velocity * math.sin(target.steering_angle)
            self.assertAlmostEqual(actual_vx, expected_vx)
            self.assertAlmostEqual(actual_vy, expected_vy)
            self.assertLessEqual(abs(target.steering_angle), math.pi / 2)

    def test_stop_targets_preserve_previous_angles(self):
        previous = self.kin.inverse(ChassisTwist(0.0, 0.5, 0.0))
        stopped = stop_targets(previous)
        self.assertEqual(
            [target.steering_angle for target in stopped],
            [target.steering_angle for target in previous],
        )
        self.assertEqual([target.drive_velocity for target in stopped], [0.0] * 4)

    def test_bounded_target_does_not_wrap_across_steering_limit(self):
        command = bounded_angle_velocity(
            math.pi / 2, math.radians(-100), 2.0, 1.0)
        self.assertEqual(command, 1.0)

    def test_optimize_reverses_drive_for_shorter_steering_path(self):
        angle, speed = optimize_target(math.pi, 1.0, 0.0)
        self.assertLess(abs(angle), 1e-9)
        self.assertEqual(speed, -1.0)

    def test_physical_motor_signs_match_reverse_installation(self):
        left, right = mix_module(1.0, 0.0, 0.1, 0.05, 1.0, -1.0)
        self.assertEqual((left, right), (10.0, -10.0))
        left_turn, right_turn = mix_module(0.0, 1.0, 0.1, 0.05, 1.0, -1.0)
        self.assertEqual(left_turn, right_turn)

    def test_rate_limit_prevents_speed_step(self):
        self.assertAlmostEqual(rate_limit(0.5, 0.0, 0.015), 0.015)
        self.assertAlmostEqual(rate_limit(-0.5, 0.0, 0.015), -0.015)

    def test_drive_scale_is_full_below_slowdown_error(self):
        self.assertEqual(
            drive_scale(math.radians(1), math.radians(2), math.radians(6)),
            1.0)

    def test_drive_scale_interpolates_between_thresholds(self):
        self.assertAlmostEqual(
            drive_scale(math.radians(4), math.radians(2), math.radians(6)),
            0.5)

    def test_drive_scale_is_zero_at_stop_error(self):
        self.assertEqual(
            drive_scale(math.radians(6), math.radians(2), math.radians(6)),
            0.0)

    def test_coordinated_speeds_apply_one_scale_to_all_modules(self):
        self.assertEqual(
            coordinated_speeds(
                [0.3, -0.3, 0.3, -0.3],
                [0.3, -0.3, 0.3, -0.3],
                0.5, 1.0, 1.0),
            [0.15, -0.15, 0.15, -0.15])

    def test_adaptive_limit_is_gentle_for_small_error(self):
        self.assertEqual(
            adaptive_steering_limit(
                math.radians(1), 0.6, 3.0,
                math.radians(2), math.radians(6)),
            0.6)

    def test_adaptive_limit_interpolates(self):
        self.assertAlmostEqual(
            adaptive_steering_limit(
                math.radians(4), 0.6, 3.0,
                math.radians(2), math.radians(6)),
            1.8)

    def test_adaptive_limit_reaches_full_authority(self):
        self.assertEqual(
            adaptive_steering_limit(
                math.radians(6), 0.6, 3.0,
                math.radians(2), math.radians(6)),
            3.0)

    def test_adaptive_limit_rejects_reversed_limits(self):
        with self.assertRaises(ValueError):
            adaptive_steering_limit(0.1, 3.0, 0.6, 0.1, 0.2)

    def test_lock_parameters_require_ordered_thresholds(self):
        with self.assertRaises(ValueError):
            validate_lock_parameters(
                4.0, 0.6, 3.0, math.radians(6), math.radians(2))

    def test_drive_waits_until_module_is_aligned(self):
        speed = safe_drive_velocity(
            0.5, 0.0, math.radians(3.0), math.radians(2.0),
            math.radians(8.0), 0.015, 0.05)
        self.assertEqual(speed, 0.0)

    def test_drive_ramps_after_module_is_aligned(self):
        speed = safe_drive_velocity(
            0.5, 0.0, math.radians(1.0), math.radians(2.0),
            math.radians(8.0), 0.015, 0.05)
        self.assertAlmostEqual(speed, 0.015)

    def test_misaligned_vehicle_stops_all_drive_immediately(self):
        speeds = coordinated_speeds(
            [0.3, -0.3, 0.3, -0.3],
            [0.2, -0.2, 0.2, -0.2],
            False, 0.015, 0.05)
        self.assertEqual(speeds, [0.0] * 4)

    def test_global_alignment_requires_every_module(self):
        self.assertFalse(alignment_ready(
            [math.radians(1), math.radians(1), math.radians(4), math.radians(1)],
            False, math.radians(3), math.radians(5), math.radians(10)))
        self.assertTrue(alignment_ready(
            [math.radians(1), math.radians(2), math.radians(2.5), math.radians(1)],
            False, math.radians(3), math.radians(5), math.radians(10)))

    def test_global_alignment_uses_leave_tolerance_after_entry(self):
        self.assertTrue(alignment_ready(
            [math.radians(4)] * 4, True,
            math.radians(3), math.radians(5), math.radians(10)))
        self.assertFalse(alignment_ready(
            [math.radians(6), 0.0, 0.0, 0.0], True,
            math.radians(3), math.radians(5), math.radians(10)))

    def test_large_angle_error_stops_immediately(self):
        speed = safe_drive_velocity(
            0.5, 0.3, math.radians(9.0), math.radians(2.0),
            math.radians(8.0), 0.015, 0.05)
        self.assertEqual(speed, 0.0)


if __name__ == '__main__':
    unittest.main()
