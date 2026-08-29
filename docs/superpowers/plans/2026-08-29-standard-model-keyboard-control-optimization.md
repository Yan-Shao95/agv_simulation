# Standard Model Keyboard Control Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate steering drift and permanent drive blocking while operating the 1200 kg standard differential-swerve model with the standard keyboard controls.

**Architecture:** Preserve `/cmd_vel → kinematics → angle controller → differential mixer → wheel controller` and the passive differential-steering hardware. Add continuous coordinated slowdown and error-dependent steering authority, then calibrate the passive joint for the approved heavy model.

**Tech Stack:** ROS 2 Jazzy, Python 3 `unittest`, xacro/URDF, Gazebo Harmonic, rosbag2 MCAP.

## Global Constraints

- Keep pure differential steering; add no steering actuator or steering command interface.
- Keep standard keyboard keys and all ROS topics/services unchanged.
- Total mass is 1200 kg: 1163.5 kg body, four 5 kg carriers, eight 2 kg wheels, one 0.5 kg lidar.
- Body inertia is `ixx=73.94`, `iyy=151.52`, `izz=201.67 kg·m²`, with zero cross terms.
- Use identical parameters on all four modules and keep targets within ±90°.
- At ±0.3 m/s for 30 seconds, every straight-mode steering error stays within ±3° without permanent blocking.
- Keep `mu1=mu2=1.5` unless recorded slip data proves otherwise.
- Preserve untracked `move/` and `move.zip`.

---

### Task 1: Continuous coordinated drive scaling

**Files:**
- Modify: `src/agv_drive_controller/agv_drive_controller/angle_control.py`
- Test: `src/agv_drive_controller/test/test_core.py`

**Interfaces:**
- Produces: `drive_scale(max_error, slowdown_error, stop_error) -> float`
- Changes: `coordinated_speeds(targets, currents, scale, acceleration_delta, deceleration_delta) -> list[float]`

- [ ] **Step 1: Write failing tests**

Import `drive_scale`, then add:

```python
def test_drive_scale_is_full_below_slowdown_error(self):
    self.assertEqual(drive_scale(math.radians(1), math.radians(2), math.radians(6)), 1.0)

def test_drive_scale_interpolates_between_thresholds(self):
    self.assertAlmostEqual(drive_scale(
        math.radians(4), math.radians(2), math.radians(6)), 0.5)

def test_drive_scale_is_zero_at_stop_error(self):
    self.assertEqual(drive_scale(math.radians(6), math.radians(2), math.radians(6)), 0.0)

def test_coordinated_speeds_apply_one_scale_to_all_modules(self):
    self.assertEqual(coordinated_speeds(
        [0.3, -0.3, 0.3, -0.3], [0.3, -0.3, 0.3, -0.3],
        0.5, 1.0, 1.0), [0.15, -0.15, 0.15, -0.15])
```

- [ ] **Step 2: Verify RED**

Run: `python3 src/agv_drive_controller/test/test_core.py`

Expected: import/test failure because `drive_scale` is missing and the old function accepts a Boolean.

- [ ] **Step 3: Implement the minimum behavior**

```python
def drive_scale(max_error, slowdown_error, stop_error):
    values = (max_error, slowdown_error, stop_error)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('驱动降速参数必须为有限数')
    if max_error < 0 or slowdown_error < 0 or stop_error <= slowdown_error:
        raise ValueError('驱动降速阈值无效')
    if max_error <= slowdown_error:
        return 1.0
    if max_error >= stop_error:
        return 0.0
    return (stop_error - max_error) / (stop_error - slowdown_error)
```

Change `coordinated_speeds` to validate `0 <= scale <= 1`, set each desired speed to `target * scale`, and rate-limit it. Replace the old binary-alignment test argument with scale `0.0` while retaining its four-zero assertion.

- [ ] **Step 4: Verify GREEN**

Run: `python3 src/agv_drive_controller/test/test_core.py`

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/agv_drive_controller/agv_drive_controller/angle_control.py src/agv_drive_controller/test/test_core.py
git commit -m "feat: add continuous coordinated drive scaling"
```

### Task 2: Error-dependent steering authority

**Files:**
- Modify: `src/agv_drive_controller/agv_drive_controller/angle_control.py`
- Test: `src/agv_drive_controller/test/test_core.py`

**Interfaces:**
- Produces: `adaptive_steering_limit(error, low_limit, high_limit, slowdown_error, stop_error) -> float`

- [ ] **Step 1: Write failing tests**

```python
def test_adaptive_limit_is_gentle_for_small_error(self):
    self.assertEqual(adaptive_steering_limit(
        math.radians(1), 0.6, 3.0, math.radians(2), math.radians(6)), 0.6)

def test_adaptive_limit_interpolates(self):
    self.assertAlmostEqual(adaptive_steering_limit(
        math.radians(4), 0.6, 3.0, math.radians(2), math.radians(6)), 1.8)

def test_adaptive_limit_reaches_full_authority(self):
    self.assertEqual(adaptive_steering_limit(
        math.radians(6), 0.6, 3.0, math.radians(2), math.radians(6)), 3.0)

def test_adaptive_limit_rejects_reversed_limits(self):
    with self.assertRaises(ValueError):
        adaptive_steering_limit(0.1, 3.0, 0.6, 0.1, 0.2)
```

- [ ] **Step 2: Verify RED**

Run: `python3 src/agv_drive_controller/test/test_core.py`

Expected: failure because `adaptive_steering_limit` is missing.

- [ ] **Step 3: Implement the helper**

```python
def adaptive_steering_limit(error, low_limit, high_limit,
                            slowdown_error, stop_error):
    values = (error, low_limit, high_limit, slowdown_error, stop_error)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('舵角控制参数必须为有限数')
    if error < 0 or low_limit < 0 or high_limit < low_limit:
        raise ValueError('舵角速度限制无效')
    fraction = 1.0 - drive_scale(error, slowdown_error, stop_error)
    return low_limit + fraction * (high_limit - low_limit)
```

- [ ] **Step 4: Verify GREEN and commit**

```bash
python3 src/agv_drive_controller/test/test_core.py
git add src/agv_drive_controller/agv_drive_controller/angle_control.py src/agv_drive_controller/test/test_core.py
git commit -m "feat: adapt steering authority to angle error"
```

Expected: all tests pass before commit.

### Task 3: Wire smooth locking into the ROS controller

**Files:**
- Modify: `src/agv_drive_controller/agv_drive_controller/nodes.py:67-113`
- Modify: `src/agv_drive_controller/config/controller.yaml`
- Modify: `src/agv_drive_controller/agv_drive_controller/angle_control.py`
- Test: `src/agv_drive_controller/test/test_core.py`

**Interfaces:**
- Consumes: `drive_scale`, `adaptive_steering_limit`, `bounded_angle_velocity`, `coordinated_speeds`
- Produces: unchanged `/drive/module_velocity_target`

- [ ] **Step 1: Write the failing parameter-validation test**

```python
def test_lock_parameters_require_ordered_thresholds(self):
    with self.assertRaises(ValueError):
        validate_lock_parameters(4.0, 0.6, 3.0, math.radians(6), math.radians(2))
```

- [ ] **Step 2: Verify RED**

Run: `python3 src/agv_drive_controller/test/test_core.py`

Expected: failure because `validate_lock_parameters` is missing.

- [ ] **Step 3: Implement validation**

```python
def validate_lock_parameters(kp, low_limit, high_limit,
                             slowdown_error, stop_error):
    values = (kp, low_limit, high_limit, slowdown_error, stop_error)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('锁角参数必须为有限数')
    if kp < 0 or low_limit < 0 or high_limit < low_limit:
        raise ValueError('锁角增益或速度限制无效')
    if slowdown_error < 0 or stop_error <= slowdown_error:
        raise ValueError('锁角降速阈值无效')
```

- [ ] **Step 4: Update `ModuleAngleController`**

Declare and validate exactly these values:

```python
self.kp = float(self.declare_parameter('kp', 4.0).value)
self.low_maximum = float(self.declare_parameter('low_steering_velocity', 0.6).value)
self.high_maximum = float(self.declare_parameter('high_steering_velocity', 3.0).value)
self.slowdown_error = math.radians(float(self.declare_parameter('slowdown_error_deg', 2.0).value))
self.stop_error = math.radians(float(self.declare_parameter('stop_error_deg', 6.0).value))
validate_lock_parameters(self.kp, self.low_maximum, self.high_maximum,
                         self.slowdown_error, self.stop_error)
```

In each target callback, compute `max_error`, one shared `scale`, and per-module limits:

```python
scale = drive_scale(max(abs(error) for error in errors),
                    self.slowdown_error, self.stop_error)
self.speeds = coordinated_speeds(
    requested_speeds, self.speeds, scale,
    self.acceleration * dt, self.deceleration * dt)
limit = adaptive_steering_limit(
    abs(error), self.low_maximum, self.high_maximum,
    self.slowdown_error, self.stop_error)
```

Pass `limit` to `bounded_angle_velocity`. Remove `aligned`, `enter_tolerance_deg`, `leave_tolerance_deg`, and `abort_tolerance_deg`. Preserve STOP override resetting all drive speeds.

Set YAML values to `kp: 4.0`, `low_steering_velocity: 0.6`, `high_steering_velocity: 3.0`, `slowdown_error_deg: 2.0`, `stop_error_deg: 6.0`, `max_drive_acceleration: 0.15`, `max_drive_deceleration: 0.50`.

- [ ] **Step 5: Test and commit**

```bash
python3 src/agv_drive_controller/test/test_core.py
git add src/agv_drive_controller/agv_drive_controller/angle_control.py src/agv_drive_controller/agv_drive_controller/nodes.py src/agv_drive_controller/config/controller.yaml src/agv_drive_controller/test/test_core.py
git commit -m "fix: coordinate heavy-model steering lock control"
```

Expected: all tests pass.

### Task 4: Build the 1200 kg passive-steering model

**Files:**
- Modify: `src/agv_description/urdf/agv.urdf.xacro:5-15`
- Create: `src/agv_description/test/test_model_parameters.py`

**Interfaces:**
- Produces: unchanged `robot_description` interface with approved mass, inertia, and passive damping

- [ ] **Step 1: Write the failing model test**

```python
import re
import unittest
from pathlib import Path

XACRO = Path(__file__).parents[1] / 'urdf' / 'agv.urdf.xacro'

class ModelParametersTest(unittest.TestCase):
    def test_heavy_body_and_passive_steering_parameters(self):
        text = XACRO.read_text()
        self.assertIn('<mass value="1163.5"/>', text)
        self.assertIn('ixx="73.94"', text)
        self.assertIn('iyy="151.52"', text)
        self.assertIn('izz="201.67"', text)
        self.assertIn('damping="1.0" friction="0.05"', text)
        self.assertEqual(len(re.findall(r'<xacro:module id="[1-4]"', text)), 4)

if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: Verify RED**

Run: `python3 src/agv_description/test/test_model_parameters.py`

Expected: failure because body mass is 120 kg and damping is 0.2.

- [ ] **Step 3: Apply approved model values**

Use:

```xml
<mass value="1163.5"/>
<inertia ixx="73.94" ixy="0" ixz="0"
         iyy="151.52" iyz="0" izz="201.67"/>
```

Change the macro steering dynamics to `<dynamics damping="1.0" friction="0.05"/>`. Do not change wheel friction, component masses, joint axes, or module coordinates.

- [ ] **Step 4: Test and commit**

```bash
python3 src/agv_description/test/test_model_parameters.py
python3 src/agv_drive_controller/test/test_core.py
git add src/agv_description/urdf/agv.urdf.xacro src/agv_description/test/test_model_parameters.py
git commit -m "feat: model 1200kg passive differential swerve vehicle"
```

Expected: both suites pass.

### Task 5: Rebuild and verify standard keyboard operation

**Files:**
- Verify only: Tasks 1–4 outputs
- Preserve: `move/`, `move.zip`

**Interfaces:**
- Consumes: `./scripts/teleop.sh`, `/drive/set_control_mode`, and existing telemetry topics
- Produces: MCAP evidence for straight and keyboard-transition acceptance

- [ ] **Step 1: Stop current keyboard and simulation cleanly**

Send Ctrl+C to both active sessions, then run:

```bash
ps -eo pid,stat,etime,cmd | rg 'ros2 launch agv_bringup|gz sim|teleop_twist_keyboard|module_angle_controller'
```

Expected: no real matching process remains.

- [ ] **Step 2: Build and run automated checks**

```bash
source /opt/ros/jazzy/setup.zsh
CC=gcc CXX=g++ colcon build --symlink-install --packages-select agv_description agv_drive_controller agv_bringup --event-handlers console_direct+
python3 src/agv_drive_controller/test/test_core.py
python3 src/agv_description/test/test_model_parameters.py
git diff --check
```

Expected: zero failed packages, all tests pass, and `git diff --check` is silent.

- [ ] **Step 3: Start clean simulation and TWIST mode**

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch agv_bringup sim.launch.py
```

In a second terminal:

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 service call /drive/set_control_mode agv_interfaces/srv/SetControlMode "{mode: 1}"
```

Expected: robot creation and both controller activations succeed; service returns `success=True`.

- [ ] **Step 4: Record 30-second forward and reverse runs**

Start recording:

```bash
ros2 bag record -s mcap -o /tmp/agv_standard_model_acceptance \
  /drive/module_target /drive/module_velocity_target \
  /drive/motor_velocity_target /wheel_velocity_controller/commands \
  /joint_states /odometry/wheel
```

In another terminal run forward, stop, reset the Gazebo model to the same initial pose, then run reverse:

```bash
timeout 30s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0}, angular: {z: 0.0}}" -r 10
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}" --once
timeout 30s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.3, y: 0.0}, angular: {z: 0.0}}" -r 10
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}" --once
```

Expected: all `module_{1..4}_steering_joint` positions stay within ±0.05236 rad, odometry moves in the requested direction, and drive output does not remain zero after errors return below 2°.

- [ ] **Step 5: Run standard keyboard transition sequence**

Run `./scripts/teleop.sh`, then execute:

```text
i -> , -> Shift+J -> Shift+L -> i -> j -> l -> Shift+J -> u -> o -> k
```

Expected: each commanded motion starts after alignment, every target remains within ±90°, and no assembly oscillates or remains blocked.

- [ ] **Step 6: Perform one-variable fallback only if evidence fails**

If ±3° fails, keep control parameters fixed, change only steering friction from 0.05 to 0.2, update the model test expectation, rebuild `agv_description`, and repeat Steps 3–5 with a new bag. Do not change wheel friction or add per-module compensation.

- [ ] **Step 7: Final verification**

```bash
python3 src/agv_drive_controller/test/test_core.py
python3 src/agv_description/test/test_model_parameters.py
git diff --check
git status --short
```

Expected: tests pass; only intentional files and pre-existing `move/`, `move.zip` appear. Commit a validated fallback alone as `fix: calibrate passive steering holding friction`.
