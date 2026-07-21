# 四差动驱动总成 AGV 仿真

这是一个面向仓储场景的 ROS 2 全向 AGV 仿真项目，运行环境为 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Harmonic。

车辆由 4 个差动驱动总成构成，每个总成包含一个被动转向关节和两个反向安装的驱动轮，共计 4 个转向关节、8 个驱动轮。左右轮的差速用于调整总成方向，两轮的平均速度用于驱动车辆，因此整车能够完成：

- 前进与后退
- 左右横移
- 任意方向斜行
- 原地旋转
- 平移与旋转组合运动

项目同时提供二维激光雷达、轮式里程计、仓库世界、SLAM Toolbox、AMCL、Nav2，以及可选的 EKF、IMU、GPS 和外部里程计接入配置。

## 当前状态

以下内容已经在 Ubuntu 24.04 原生环境中实际验证：

- ROS 2 Jazzy 全工作空间构建，8 个软件包全部成功。
- Xacro 展开和 URDF 结构检查。
- Gazebo Harmonic 仓库世界和 AGV 实体生成。
- 4 个转向关节与 8 个轮关节的 Gazebo/ros2_control 硬件接口。
- 关节状态广播器和 8 轮速度控制器自动加载。
- ROS 与 Gazebo 的 `/clock`、`/scan` 桥接。
- 720 点、360°、0.10～20 m 的二维激光雷达。
- `/joint_states` 和 `/odometry/wheel` 反馈。
- 前进、后退、横移、斜行和原地旋转。
- 指令超时自动停车和 ROS 节点正常退出。
- 5 项控制与运动学自动测试。

以下功能已经提供配置，但尚未完成完整端到端验收：

- SLAM Toolbox 实际建图和地图质量验收。
- AMCL 长时间定位稳定性。
- Nav2 多目标点自主导航。
- GPS、IMU 和外部里程计的实测融合。

仓库自带的 `warehouse.pgm` 是低分辨率示意地图。正式导航应通过 SLAM 生成约 0.05 m/像素的地图。

## 系统架构

速度控制链路如下：

```text
/cmd_vel
   │
   ▼
command_arbiter
   │  /drive/twist_command
   ▼
swerve_kinematics
   │  /drive/module_target
   ▼
module_angle_controller ◀── /joint_states
   │  /drive/module_velocity_target
   ▼
differential_module_mixer
   │  /drive/motor_velocity_target
   ▼
motor_command_bridge
   │  /wheel_velocity_controller/commands
   ▼
ros2_control / Gazebo
   │
   ├── /joint_states
   └── /odometry/wheel
```

控制链分层设计，既可以使用标准 `geometry_msgs/msg/Twist` 控制整车，也可以绕过上层运动学，直接调试总成或 8 个电机。

## 软件包说明

| 软件包 | 作用 |
|---|---|
| `agv_interfaces` | 自定义总成命令、电机命令、状态消息和控制模式服务 |
| `agv_drive_controller` | 指令仲裁、四总成运动学、舵向闭环、差动混控和轮式里程计 |
| `agv_description` | AGV Xacro/URDF、雷达和 ros2_control 配置 |
| `agv_gazebo` | Gazebo 启动、车辆生成、控制器加载和 ros_gz_bridge |
| `agv_worlds` | 40 m × 50 m 仓库世界 |
| `agv_localization` | robot_localization、EKF 和 GPS 扩展配置 |
| `agv_navigation` | SLAM Toolbox、AMCL、Nav2 和示意地图 |
| `agv_bringup` | 仿真、建图和导航的统一启动入口 |

## 环境要求

推荐环境：

- Ubuntu 24.04 LTS，x86_64
- ROS 2 Jazzy
- Gazebo Harmonic
- 至少 4 核 CPU、8 GB 内存
- 推荐 8 核 CPU、16 GB 内存和可用 OpenGL 显卡

没有 GPU 设备权限时，Gazebo 可以回退到软件渲染，但仿真实时因子会降低。此时用墙钟统计的 `/scan` 频率可能低于 10 Hz，而仿真时间中的传感器配置仍为 10 Hz。

## 获取项目

```bash
git clone git@github.com:Yan-Shao95/agv_simulation.git
cd agv_simulation
```

如果当前网络禁止 GitHub SSH 22 端口，可以使用 HTTPS：

```bash
git clone https://github.com/Yan-Shao95/agv_simulation.git
cd agv_simulation
```

## 安装依赖

首次使用时运行：

```bash
./scripts/install_native.sh
```

该脚本会安装 ROS 2 桌面环境、Gazebo ROS 集成、ros2_control、Nav2、SLAM Toolbox、robot_localization、Colcon 和 rosdep 等依赖。脚本包含 `sudo apt-get`，执行时需要输入当前 Ubuntu 用户密码。

如果脚本没有执行权限：

```bash
chmod +x scripts/*.sh docker/entrypoint.sh
```

## 构建

在项目根目录执行：

```bash
./scripts/build.sh
```

等价的手动构建流程：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys ament_python
CC=gcc CXX=g++ colcon build --symlink-install
```

构建完成后加载工作空间：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

每个新终端都需要重新执行这两条 `source` 命令。

## 运行测试

验证 Xacro 和 URDF：

```bash
./scripts/validate_xacro.sh
```

运行全部自动测试：

```bash
./scripts/test.sh
```

当前预期结果：

```text
8 packages finished
5 tests, 0 errors, 0 failures, 0 skipped
```

## 启动仿真

终端 1：

```bash
cd /path/to/agv_simulation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch agv_bringup sim.launch.py
```

启动过程会自动完成：

1. 加载 40 m × 50 m 仓库世界。
2. 展开 AGV Xacro 并启动 `robot_state_publisher`。
3. 在 Gazebo 中生成车辆。
4. 建立 `/clock` 和 `/scan` 桥接。
5. 启动完整驱动控制链。
6. 加载 `joint_state_broadcaster`。
7. 加载 `wheel_velocity_controller`。

看到两个控制器均为 `active` 后即可控制车辆：

```bash
ros2 control list_controllers
```

结束仿真时在终端 1 按 `Ctrl+C`。

## 控制车辆

打开终端 2，并加载环境：

```bash
cd /path/to/agv_simulation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 1. 切换到 TWIST 模式

系统启动后默认处于 STOP 模式，不接受 `/cmd_vel`。先执行：

```bash
ros2 service call /drive/set_control_mode \
  agv_interfaces/srv/SetControlMode "{mode: 1}"
```

返回 `success=True` 表示切换成功。

### 2. 发送运动命令

前进：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0}, angular: {z: 0.0}}" -r 10
```

后退：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.3, y: 0.0}, angular: {z: 0.0}}" -r 10
```

向左横移：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.3}, angular: {z: 0.0}}" -r 10
```

向右横移：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: -0.3}, angular: {z: 0.0}}" -r 10
```

左前方斜行：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.2}, angular: {z: 0.0}}" -r 10
```

原地逆时针旋转：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.3}}" -r 10
```

原地顺时针旋转：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: -0.3}}" -r 10
```

按 `Ctrl+C` 停止命令发布。控制器超过 0.5 秒未收到有效指令时会自动发送零轮速。

### 3. 停止并锁定控制

```bash
ros2 service call /drive/set_control_mode \
  agv_interfaces/srv/SetControlMode "{mode: 0}"
```

## 控制模式

| 模式 | 编号 | 输入 | 用途 |
|---|---:|---|---|
| STOP | 0 | 无 | 停车，忽略运动命令 |
| TWIST | 1 | `/cmd_vel` | 常规整车控制、Nav2 和上层调度 |
| MODULE | 2 | `/drive/module_command_direct` | 直接调试 4 个总成 |
| MOTOR | 3 | `/drive/motor_command_direct` | 直接调试 8 个电机 |

模式切换时控制器会先发送一次零速度命令。

## 主要 ROS 2 接口

| 名称 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 输入 | TWIST 模式整车速度 |
| `/drive/set_control_mode` | `agv_interfaces/srv/SetControlMode` | 服务 | 设置 STOP/TWIST/MODULE/MOTOR |
| `/drive/module_command_direct` | `agv_interfaces/msg/ModuleCommandArray` | 输入 | MODULE 模式直接总成命令 |
| `/drive/motor_command_direct` | `agv_interfaces/msg/MotorCommandArray` | 输入 | MOTOR 模式 8 轮速度命令 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 输出 | 4 个转向关节和 8 个轮关节状态 |
| `/odometry/wheel` | `nav_msgs/msg/Odometry` | 输出 | 四总成正运动学轮式里程计 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 输出 | 360° 二维激光雷达 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 输出 | Gazebo 仿真时钟 |
| `/wheel_velocity_controller/commands` | `std_msgs/msg/Float64MultiArray` | 内部 | 8 轮控制器目标速度 |

完整控制链接口见 [docs/interfaces.md](docs/interfaces.md)，独立节点调试方法见 [docs/debugging.md](docs/debugging.md)。

## 查看运行状态

控制器状态：

```bash
ros2 control list_controllers
ros2 control list_hardware_components
```

车辆关节反馈：

```bash
ros2 topic echo /joint_states --once
```

轮式里程计：

```bash
ros2 topic echo /odometry/wheel
```

雷达数据和频率：

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
```

列出节点、话题和服务：

```bash
ros2 node list
ros2 topic list
ros2 service list
```

## 建图

先启动基础仿真并切换到 TWIST 模式，然后在新终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch agv_bringup mapping.launch.py
```

遥控车辆覆盖仓库通道后保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f warehouse
```

保存结果通常包含 `warehouse.yaml` 和 `warehouse.pgm`。正式使用前应检查地图分辨率、闭环一致性和障碍物边界。

## 定位与导航

使用已有地图启动 AMCL 和 Nav2：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch agv_bringup navigation.launch.py
```

Nav2 配置允许 `vx`、`vy` 和 `wz`，可利用底盘的全向能力。当前仓库附带地图仅用于配置联调；建议先完成实际 SLAM 建图，再进行导航验收。

## 可选定位融合

预留接口：

- IMU：`/sensors/imu/data`，`sensor_msgs/msg/Imu`
- GPS：`/sensors/gps/fix`，`sensor_msgs/msg/NavSatFix`
- 外部里程计：`/odometry/external`
- 融合里程计：`/odometry/filtered`

启动融合：

```bash
ros2 launch agv_localization localization_fusion.launch.py
```

启用 GPS：

```bash
ros2 launch agv_localization localization_fusion.launch.py use_gps:=true
```

启用 EKF 时，应关闭轮式里程计节点的 `publish_odom_tf`，确保只有一个节点发布 `odom → base_link`。

## 独立调试控制节点

每层控制节点都提供独立 launch：

```bash
ros2 launch agv_drive_controller command_arbiter_standalone.launch.py
ros2 launch agv_drive_controller swerve_kinematics_standalone.launch.py
ros2 launch agv_drive_controller module_angle_controller_standalone.launch.py
ros2 launch agv_drive_controller differential_module_mixer_standalone.launch.py
ros2 launch agv_drive_controller motor_command_bridge_standalone.launch.py
```

适合逐层检查消息、运动学目标、转向误差和电机混控输出。

## Docker

构建镜像：

```bash
docker compose -f docker/docker-compose.yml build
```

运行：

```bash
docker compose -f docker/docker-compose.yml run --rm agv-sim
```

Gazebo GUI 需要正确传递 `DISPLAY`、X11/Wayland 套接字和 GPU 设备。桌面 Ubuntu 环境优先推荐原生安装方式。

## 常见问题

### 启动后车辆不响应 `/cmd_vel`

系统默认处于 STOP 模式。确认已经切换到 TWIST：

```bash
ros2 service call /drive/set_control_mode \
  agv_interfaces/srv/SetControlMode "{mode: 1}"
```

### 找不到项目软件包

当前终端没有加载工作空间：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 控制器未激活

```bash
ros2 control list_controllers
```

正常情况下 `joint_state_broadcaster` 和 `wheel_velocity_controller` 都应为 `active`。

### 出现 EGL 或 `/dev/dri` 警告

当前用户无法访问 GPU 渲染设备。Gazebo 通常会使用软件渲染继续运行，但速度较慢。可以检查：

```bash
ls -l /dev/dri
groups
```

实体机可根据系统策略将用户加入 `render` 或 `video` 组，重新登录后生效。

### 从其他电脑复制后构建失败

不要复用其他路径或系统生成的 `build/`、`install/`、`log/`。先备份或删除这些生成目录，再重新运行：

```bash
./scripts/build.sh
```

### 雷达墙钟频率低于 10 Hz

先观察 Gazebo 的实时因子。软件渲染或 CPU 负载过高时，整个仿真都会慢于现实时间；传感器在仿真时间中的更新率仍配置为 10 Hz。

## 项目许可证

项目软件包声明使用 Apache-2.0 许可证。
