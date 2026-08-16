# AGV Three-Mode Drive Kinematics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement predictable straight, lateral, rotation, and combined-turn motion for the four differential drive modules while coordinating steering alignment across the whole vehicle.

**Architecture:** Keep the existing `/cmd_vel` pipeline and split responsibility between pure `SwerveKinematics` target generation and `ModuleAngleController` transition safety. Pure lateral motion uses the approved fixed steering signs; all other nonzero motions use vector kinematics followed by a deterministic `±90°` equivalent-angle transform. A small keyboard script launches standard `teleop_twist_keyboard` with periodic command repetition.

**Tech Stack:** Python 3.12, ROS 2 Jazzy (`rclpy`, launch), Gazebo Harmonic, `unittest`, colcon.

## Global Constraints

- Preserve `/cmd_vel`, `/drive/module_target`, `/drive/module_velocity_target`, and `/wheel_velocity_controller/commands`.
- Module order is 1 left-front, 2 right-front, 3 right-rear, 4 left-rear.
- Steering targets must remain in `[-pi/2, +pi/2]`.
- Pure left-lateral steering is `[-pi/2, +pi/2, -pi/2, +pi/2]`; right lateral keeps these angles and reverses drive velocity.
- A new steering transition must stop all four drive modules until all four are aligned.
- Zero commands stop drive motion without commanding steering back to zero.
- Keyboard commands repeat at 10 Hz and remain subject to the 0.5 second arbiter watchdog.
- Preserve unrelated local changes and untracked `move/` artifacts.

---

### Task 1: Deterministic Three-Mode Kinematics

**Files:**
- Modify: `src/agv_drive_controller/agv_drive_controller/kinematics.py`
- Modify: `src/agv_drive_controller/agv_drive_controller/nodes.py`
- Modify: `src/agv_drive_controller/test/test_core.py`

**Interfaces:**
- Consumes: `ChassisTwist(vx: float, vy: float, wz: float)` and the existing module position tuple.
- Produces: `equivalent_target(angle: float, speed: float) -> tuple[float, float]` and `SwerveKinematics.inverse(twist: ChassisTwist) -> tuple[ModuleTarget, ...]` with every target angle in `[-pi/2, pi/2]`.

- [ ] **Step 1: Write failing tests for straight and fixed-sign lateral targets**

Add these tests to `CoreTest`:

```python
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
```

- [ ] **Step 2: Run the lateral tests and verify RED**

Run:

```bash
python3 -m unittest \
  src.agv_drive_controller.test.test_core.CoreTest.test_straight_modes_keep_all_modules_at_zero \
  src.agv_drive_controller.test.test_core.CoreTest.test_lateral_modes_use_rotation_derived_signs -v
```

Expected: straight mode or lateral mode fails because the current generic `atan2` implementation produces uniform `+90°` for left lateral and `-90°` for right lateral.

- [ ] **Step 3: Write failing tests for bounded self-rotation and combined motion**

```python
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
```

- [ ] **Step 4: Run the rotation tests and verify RED**

Run:

```bash
python3 -m unittest \
  src.agv_drive_controller.test.test_core.CoreTest.test_counterclockwise_rotation_uses_bounded_expected_angles \
  src.agv_drive_controller.test.test_core.CoreTest.test_combined_motion_preserves_each_wheel_velocity_vector -v
```

Expected: rotation test fails because module 1 and module 4 currently produce angles whose absolute values exceed 90°.

- [ ] **Step 5: Implement explicit modes and the equivalent-angle transform**

In `kinematics.py`, add:

```python
EPSILON = 1e-12
LATERAL_ANGLES = (-math.pi / 2, math.pi / 2, -math.pi / 2, math.pi / 2)
LATERAL_SIGNS = (-1.0, 1.0, -1.0, 1.0)


def equivalent_target(angle, speed):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    if angle > math.pi / 2:
        return angle - math.pi, -speed
    if angle < -math.pi / 2:
        return angle + math.pi, -speed
    return angle, speed
```

Replace `inverse()` with explicit zero, longitudinal, lateral, and vector branches:

```python
def inverse(self, twist):
    if not all(math.isfinite(v) for v in (twist.vx, twist.vy, twist.wz)):
        raise ValueError('速度必须为有限数')
    if abs(twist.vx) <= EPSILON and abs(twist.vy) <= EPSILON and abs(twist.wz) <= EPSILON:
        return tuple(ModuleTarget(i, 0.0, 0.0) for i in range(1, 5))
    if abs(twist.vy) <= EPSILON and abs(twist.wz) <= EPSILON:
        return tuple(ModuleTarget(i, 0.0, twist.vx) for i in range(1, 5))
    if abs(twist.vx) <= EPSILON and abs(twist.wz) <= EPSILON:
        magnitude = abs(twist.vy)
        direction = 1.0 if twist.vy > 0.0 else -1.0
        return tuple(
            ModuleTarget(i, angle, magnitude * sign * direction)
            for i, (angle, sign) in enumerate(zip(LATERAL_ANGLES, LATERAL_SIGNS), 1)
        )
    result = []
    for index, (x, y) in enumerate(self.positions, 1):
        wheel_vx = twist.vx - twist.wz * y
        wheel_vy = twist.vy + twist.wz * x
        speed = math.hypot(wheel_vx, wheel_vy)
        angle = math.atan2(wheel_vy, wheel_vx) if speed > EPSILON else 0.0
        angle, speed = equivalent_target(angle, speed)
        result.append(ModuleTarget(index, angle, speed))
    return tuple(result)
```

- [ ] **Step 6: Preserve steering targets when a zero command stops the vehicle**

Import `stop_targets` and add the failing test:

```python
def test_stop_targets_preserve_previous_angles(self):
    previous = self.kin.inverse(ChassisTwist(0.0, 0.5, 0.0))
    stopped = stop_targets(previous)
    self.assertEqual(
        [target.steering_angle for target in stopped],
        [target.steering_angle for target in previous],
    )
    self.assertEqual([target.drive_velocity for target in stopped], [0.0] * 4)
```

Run this test and verify RED because `stop_targets` does not exist. Then add this pure helper to `kinematics.py`:

```python
def stop_targets(previous):
    return tuple(
        ModuleTarget(target.module_id, target.steering_angle, 0.0)
        for target in previous
    )
```

In `KinematicsNode`, initialize `self.last_targets` to four zero-angle stopped targets. For an all-zero `ChassisTwist`, publish `stop_targets(self.last_targets)`; for a nonzero twist, call `inverse()`, save the result in `self.last_targets`, and publish it. This keeps the last steering orientation while stopping all drive modules.

- [ ] **Step 7: Run all core tests and verify GREEN**

Run: `python3 -m unittest src/agv_drive_controller/test/test_core.py -v`

Expected: all kinematics and existing mixer/control helper tests pass. Update the obsolete `test_horizontal_motion_points_all_modules_left` expectation to the approved fixed-sign lateral values rather than retaining two contradictory lateral tests.

- [ ] **Step 8: Commit Task 1**

```bash
git add src/agv_drive_controller/agv_drive_controller/kinematics.py \
  src/agv_drive_controller/agv_drive_controller/nodes.py \
  src/agv_drive_controller/test/test_core.py
git commit -m "feat: add deterministic three-mode kinematics"
```

---

### Task 2: Whole-Vehicle Steering Alignment Gate

**Files:**
- Modify: `src/agv_drive_controller/agv_drive_controller/angle_control.py`
- Modify: `src/agv_drive_controller/agv_drive_controller/nodes.py`
- Modify: `src/agv_drive_controller/config/controller.yaml`
- Modify: `src/agv_drive_controller/test/test_core.py`

**Interfaces:**
- Consumes: four optimized `(angle, speed)` targets and four measured steering angles.
- Produces: `alignment_ready(errors, was_ready, enter_tolerance, leave_tolerance, abort_tolerance) -> bool` and coordinated module commands where drive is enabled for all modules or none.

- [ ] **Step 1: Write failing tests for global alignment hysteresis**

Import `alignment_ready` and add:

```python
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
```

- [ ] **Step 2: Run the new tests and verify RED**

Run:

```bash
python3 -m unittest \
  src.agv_drive_controller.test.test_core.CoreTest.test_global_alignment_requires_every_module \
  src.agv_drive_controller.test.test_core.CoreTest.test_global_alignment_uses_leave_tolerance_after_entry -v
```

Expected: error or failure because the current helper accepts one scalar error rather than four errors.

- [ ] **Step 3: Generalize `alignment_ready` to the whole vehicle**

Replace the scalar helper with:

```python
def alignment_ready(errors, was_ready, enter_tolerance, leave_tolerance,
                    abort_tolerance):
    errors = tuple(abs(error) for error in errors)
    values = errors + (enter_tolerance, leave_tolerance, abort_tolerance)
    if not errors or not all(math.isfinite(value) for value in values):
        raise ValueError('对正参数必须为有限数')
    if not 0 <= enter_tolerance <= leave_tolerance <= abort_tolerance:
        raise ValueError('对正容差顺序无效')
    limit = leave_tolerance if was_ready else enter_tolerance
    return all(error <= limit for error in errors)
```

- [ ] **Step 4: Refactor `ModuleAngleController` to coordinate all modules**

Declare `enter_tolerance_deg`, `leave_tolerance_deg`, and `abort_tolerance_deg`; store `self.aligned = False`. In `_targets`, first optimize all four targets, calculate all four errors, update one global alignment state, and then generate output:

```python
optimized = [
    optimize_target(target.steering_angle, target.drive_velocity, self.angles[i])
    for i, target in enumerate(msg.modules)
]
errors = [normalize_angle(angle - self.angles[i]) for i, (angle, _) in enumerate(optimized)]
self.aligned = alignment_ready(
    errors, self.aligned, self.enter_tolerance,
    self.leave_tolerance, self.abort_tolerance)
if any(abs(error) >= self.abort_tolerance for error in errors):
    self.aligned = False

out = []
for i, ((angle, requested_speed), error) in enumerate(zip(optimized, errors)):
    desired_speed = requested_speed if self.aligned else 0.0
    self.speeds[i] = rate_limit(
        desired_speed, self.speeds[i],
        (self.acceleration if abs(desired_speed) > abs(self.speeds[i])
         else self.deceleration) * dt)
    out.append(ModuleCommand(
        module_id=i + 1,
        steering_angle=angle_velocity(angle, self.angles[i], self.kp, self.maximum),
        drive_velocity=self.speeds[i]))
```

When `_motor_override` receives eight zero velocities, reset `self.speeds` and `self.aligned` so the next command must pass alignment again. Import `alignment_ready` and `rate_limit` from `angle_control`.

- [ ] **Step 5: Set hysteresis parameters**

Use these values in `controller.yaml`:

```yaml
    enter_tolerance_deg: 3.0
    leave_tolerance_deg: 5.0
    abort_tolerance_deg: 10.0
    max_drive_acceleration: 0.15
    max_drive_deceleration: 0.50
```

Remove the old `ready_tolerance_deg` key and the per-module `safe_drive_velocity` call from `nodes.py`; keep `safe_drive_velocity` only if existing direct unit tests still document a separately used pure helper, otherwise remove it and its tests as dead code.

- [ ] **Step 6: Run all core tests and verify GREEN**

Run: `python3 -m unittest src/agv_drive_controller/test/test_core.py -v`

Expected: all tests pass, including the new global hysteresis cases.

- [ ] **Step 7: Commit Task 2**

```bash
git add src/agv_drive_controller/agv_drive_controller/angle_control.py \
  src/agv_drive_controller/agv_drive_controller/nodes.py \
  src/agv_drive_controller/config/controller.yaml \
  src/agv_drive_controller/test/test_core.py
git commit -m "fix: coordinate steering alignment across modules"
```

---

### Task 3: Reliable Standard Keyboard Launcher

**Files:**
- Create: `scripts/teleop.sh`
- Modify: `README.md`

**Interfaces:**
- Consumes: installed `teleop_twist_keyboard` package and built workspace environment.
- Produces: a foreground keyboard process publishing `/cmd_vel` at `repeat_rate=10.0`.

- [ ] **Step 1: Verify the current one-shot command behavior**

Run standard teleop without parameters in one terminal and inspect `/cmd_vel` in another. Expected before the script: a key press creates one message and the arbiter watchdog stops the vehicle after 0.5 seconds.

- [ ] **Step 2: Create `scripts/teleop.sh`**

```bash
#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
if test -f install_runtime/setup.bash; then
  source install_runtime/setup.bash
else
  source install/setup.bash
fi
set -u
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p repeat_rate:=10.0
```

Mark it executable with `chmod +x scripts/teleop.sh`.

- [ ] **Step 3: Document exact keyboard startup and mappings**

Add a README section that instructs users to switch to TWIST mode, execute `./scripts/teleop.sh`, keep its terminal focused, use lowercase `j/l` for rotation and uppercase `J/L` for lateral motion, and press `k` to stop.

- [ ] **Step 4: Verify the launcher**

Run: `bash -n scripts/teleop.sh`

Expected: exit code 0.

Then run `./scripts/teleop.sh`, press `i` once, and run `ros2 topic hz /cmd_vel` from a sourced terminal.

Expected: approximately 10 Hz until `k` is pressed.

- [ ] **Step 5: Commit Task 3**

```bash
git add scripts/teleop.sh README.md
git commit -m "feat: add reliable keyboard teleop launcher"
```

---

### Task 4: Full Build, Regression, and Simulation Acceptance

**Files:**
- Modify only if verification reveals a defect: files owned by Tasks 1-3.

**Interfaces:**
- Consumes: completed kinematics, alignment controller, and keyboard launcher.
- Produces: verified build and recorded acceptance evidence.

- [ ] **Step 1: Run Python regression tests**

Run: `python3 -m unittest src/agv_drive_controller/test/test_core.py -v`

Expected: all tests pass with zero failures and zero errors.

- [ ] **Step 2: Build all eight packages in path-safe runtime directories**

Run:

```bash
source /opt/ros/jazzy/setup.bash
CC=gcc CXX=g++ colcon --log-base log_runtime build \
  --symlink-install --build-base build_runtime --install-base install_runtime
```

Expected: `Summary: 8 packages finished`.

- [ ] **Step 3: Run the workspace test suite in runtime directories**

Run:

```bash
source /opt/ros/jazzy/setup.bash
source install_runtime/setup.bash
colcon --log-base log_runtime test \
  --build-base build_runtime --install-base install_runtime
colcon test-result --test-result-base build_runtime --verbose
```

Expected: zero test failures and zero test errors.

- [ ] **Step 4: Launch simulation and verify controllers**

Run:

```bash
source /opt/ros/jazzy/setup.bash
source install_runtime/setup.bash
ros2 launch agv_bringup sim.launch.py
```

In a second terminal run `ros2 control list_controllers`.

Expected: both `joint_state_broadcaster` and `wheel_velocity_controller` are `active`.

- [ ] **Step 5: Execute the approved motion sequence**

Switch to TWIST mode, run `./scripts/teleop.sh`, and execute:

```text
i -> , -> J -> L -> i -> j -> l -> J -> u -> o -> k
```

For each transition, inspect one sample from `/drive/module_target`, `/drive/module_velocity_target`, `/wheel_velocity_controller/commands`, `/joint_states`, and `/odometry/wheel`.

Expected:

- Pure lateral angles/signs match the approved table.
- Rotation targets stay within ±90°.
- During a steering transition all four module drive velocities are zero.
- Once all four modules align, all four ramp together.
- Odometry changes in the commanded direction.
- No transition remains permanently blocked.

- [ ] **Step 6: Run final repository checks**

Run:

```bash
git diff --check
git status --short
```

Expected: no whitespace errors; only intentional implementation files and pre-existing user artifacts are listed.

- [ ] **Step 7: Commit any verification-only correction**

Only if Step 5 exposed a defect, add the failing regression test first, verify RED, apply the smallest correction, verify GREEN, and commit the exact affected files with:

```bash
git commit -m "fix: correct three-mode simulation behavior"
```
