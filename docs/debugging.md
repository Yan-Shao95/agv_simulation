# 独立调试说明

模块编号：1 左前、2 右前、3 右后、4 左后；每模块电机顺序为左、右。

## 节点链路

`command_arbiter → swerve_kinematics → module_angle_controller → differential_module_mixer → motor_command_bridge`

每个节点均有独立 launch：

- `command_arbiter_standalone.launch.py`
- `swerve_kinematics_standalone.launch.py`
- `module_angle_controller_standalone.launch.py`
- `differential_module_mixer_standalone.launch.py`
- `motor_command_bridge_standalone.launch.py`

MODULE 模式向 `/drive/module_command_direct` 发布 4 个总成命令；MOTOR 模式向 `/drive/motor_command_direct` 发布固定 8 元素电机速度数组。模式切换先发送零速度，指令超过 0.5 秒自动停车。

## 扩展传感器

- IMU：`/sensors/imu/data`，`sensor_msgs/msg/Imu`，frame 为 `imu_link`。
- GPS：`/sensors/gps/fix`，`sensor_msgs/msg/NavSatFix`，frame 为 `gps_link`。
- 轮式里程计：`/odometry/wheel`。
- 外部里程计：`/odometry/external`。
- 融合结果：`/odometry/filtered`。

启用 `agv_localization` 时，应关闭轮式里程计节点的 `publish_odom_tf`，确保只有 EKF 发布 `odom → base_link`。
