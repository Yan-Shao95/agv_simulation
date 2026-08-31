# 多车型 AGV 仿真

这是一个面向仓储场景的 ROS 2 AGV 仿真项目，运行环境为 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Harmonic。项目通过统一的 `vehicle_type` 参数支持三种底盘：

| `vehicle_type` | 车型 | 主要运动能力 |
|---|---|---|
| `differential_swerve` | 四差动驱动总成 AGV（默认） | 前后、横移、斜行、自转及组合运动 |
| `mecanum` | 四驱麦克纳姆轮小车 | 前后、左右横移、自转及组合运动 |
| `ackermann` | 标准阿克曼小车 | 前后行驶和随车速转向 |

默认差动舵轮车型由 4 个差动驱动总成构成，每个总成包含一个转向关节和两个反向安装的驱动轮，共计 4 个转向关节、8 个驱动轮。标准物理模型按约 1200 kg 整车质量配置，并包含周期舵角闭环和停车角度保持优化。

项目同时提供二维激光雷达、轮式里程计、40 m × 50 m 仓库世界、SLAM Toolbox、AMCL、Nav2，以及 EKF、IMU、GPS 和外部里程计接入配置。

## 当前状态

以下内容已经在 Ubuntu 24.04 原生环境中实际验证：

- ROS 2 Jazzy 全工作空间构建，8 个软件包全部成功。
- 三种车型的 Xacro 展开、URDF 结构检查和 Gazebo 实体生成。
- `vehicle_type` 在仿真、建图和导航启动入口中的透传。
- 默认差动舵轮车型约 1200 kg 物理模型、4 个转向关节、8 个轮关节及完整 ros2_control 控制链。
- 麦克纳姆轮前后、左右横移和正反自转；四轮摩擦方向固定于车体坐标，不随轮子旋转。
- 麦克纳姆仿真 GPS 已通过 `/sensors/gps/fix` 发布有效 `NavSatFix` 数据。
- 阿克曼前后行驶、组合转向，以及纵向速度保持、转向松开自动回正的专用键盘控制。
- ROS 与 Gazebo 的 `/clock`、`/scan`、`/cmd_vel`、`/odometry/wheel` 桥接。
- 720 点、360°、0.10～20 m 的二维激光雷达。
- 默认车型的 `/joint_states` 和 `/odometry/wheel` 反馈。
- 三车型受控速度指令和世界真实位姿动态验收。
- 指令超时自动停车和 ROS 节点正常退出。
- 控制、运动学、车型选择、麦克纳姆接触配置和阿克曼键盘自动测试。

以下功能已经提供配置，但尚未完成完整端到端验收：

- SLAM Toolbox 实际建图和地图质量验收。
- AMCL 长时间定位稳定性。
- Nav2 多目标点自主导航。
- GPS、IMU 和外部里程计的完整融合定位验收。

仓库自带的 `warehouse.pgm` 是低分辨率示意地图。正式导航应通过 SLAM 生成约 0.05 m/像素的地图。

## 系统架构

三种车型共用 `/cmd_vel`、Gazebo 世界和传感器桥接。麦克纳姆和阿克曼由各自的 Gazebo 驱动插件直接消费 `/cmd_vel`；以下分层控制链仅用于默认差动舵轮车型：

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
| `agv_drive_controller` | 默认车型控制链、阿克曼状态保持键盘和轮式里程计 |
| `agv_description` | 三车型 Xacro/URDF、雷达和 ros2_control 配置 |
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

测试脚本会检查控制与运动学逻辑、三车型选择、Xacro、麦克纳姆接触方向和阿克曼状态保持键盘。测试数量会随功能增长，以脚本退出码和测试报告中的零失败为准。

## 启动仿真

终端 1 先加载环境：

```bash
cd /path/to/agv_simulation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

然后选择车型启动。未指定 `vehicle_type` 时默认加载 `differential_swerve`。

四差动总成 AGV（默认）：

```bash
ros2 launch agv_bringup sim.launch.py vehicle_type:=differential_swerve
```

四驱麦克纳姆轮小车：

```bash
ros2 launch agv_bringup sim.launch.py vehicle_type:=mecanum
```

标准阿克曼小车：

```bash
ros2 launch agv_bringup sim.launch.py vehicle_type:=ackermann
```

启动过程会自动完成：

1. 加载 40 m × 50 m 仓库世界。
2. 展开 AGV Xacro 并启动 `robot_state_publisher`。
3. 在 Gazebo 中生成车辆。
4. 建立 `/clock`、`/scan`、`/cmd_vel` 和 `/odometry/wheel` 桥接。
5. 仅在 `differential_swerve` 下启动完整差动舵轮控制链。
6. 仅在 `differential_swerve` 下加载 `joint_state_broadcaster` 和 `wheel_velocity_controller`。
7. 在麦克纳姆模型中加载 GPS 传感器和固定于车体的轮地摩擦方向。

默认差动舵轮车型看到两个控制器均为 `active` 后即可控制：

```bash
ros2 control list_controllers
```

麦克纳姆和阿克曼使用 Gazebo 驱动插件，不会出现上述两个 ros2_control 控制器。结束仿真时在终端 1 按 `Ctrl+C`。

## 控制车辆

打开终端 2，并加载环境：

```bash
cd /path/to/agv_simulation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 默认差动舵轮车型：切换到 TWIST 模式

只有 `differential_swerve` 使用 STOP/TWIST/MODULE/MOTOR 仲裁。该车型启动后默认处于 STOP 模式，需要先执行：

```bash
ros2 service call /drive/set_control_mode \
  agv_interfaces/srv/SetControlMode "{mode: 1}"
```

返回 `success=True` 表示切换成功。

### 默认差动舵轮与麦克纳姆：通用键盘控制

在项目根目录运行：

```bash
./scripts/teleop.sh
```

保持该终端获得键盘焦点。`differential_swerve` 启动键盘前必须完成上述 TWIST 模式切换；`mecanum` 由 Gazebo 插件直接接收 `/cmd_vel`，无需调用模式服务。键位如下：

- `i` / `,`：前进 / 后退
- 小写 `j` / `l`：逆时针 / 顺时针自转
- 大写 `J` / `L`（按住 Shift）：向左 / 向右横移
- `u` / `o` / `m` / `.`：带转向的前进或后退组合运动
- `k`：立即停止驱动，并保持当前舵轮角度

麦克纳姆模型的轮轴方向和四轮各向异性摩擦方向已按 Gazebo MecanumDrive 约定配置。横移和自转应同时观察 Gazebo 世界真实姿态，不能只依赖插件积分得到的理想轮式里程计。

### 阿克曼状态保持式键盘控制

`ackermann` 不使用通用键盘节点，因为通用节点按 `j/l` 时会把线速度清零，而阿克曼车辆不能原地转向。请使用独立的状态保持式键盘节点：

```bash
./scripts/ackermann_teleop.sh
```

- `i`：保持前进，后续转向操作不会清除前进速度。
- `,`：保持后退。
- `j` / `l`：每个按键事件将转向指令增加 / 减少 `0.1 rad/s`，最大为 `±1.0 rad/s`。
- 停止输入 `j/l` 超过 `0.15` 秒后，转向自动恢复为零，纵向速度继续保持。
- `k`：停车并回正。
- `Ctrl-C`：发布零速度后退出。

### 直接发送运动命令

三种车型都接收 `/cmd_vel`。阿克曼只使用 `linear.x` 与 `angular.z`；`linear.y` 横移命令仅适用于全向车型。

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

按 `Ctrl+C` 停止命令发布。默认差动舵轮控制器超过 0.5 秒未收到有效指令时会自动发送零轮速。

### 默认差动舵轮车型：停止并锁定控制

```bash
ros2 service call /drive/set_control_mode \
  agv_interfaces/srv/SetControlMode "{mode: 0}"
```

## 默认差动舵轮控制模式

以下模式只属于 `differential_swerve`，麦克纳姆和阿克曼不提供 `/drive/set_control_mode` 服务。

| 模式 | 编号 | 输入 | 用途 |
|---|---:|---|---|
| STOP | 0 | 无 | 停车，忽略运动命令 |
| TWIST | 1 | `/cmd_vel` | 常规整车控制、Nav2 和上层调度 |
| MODULE | 2 | `/drive/module_command_direct` | 直接调试 4 个总成 |
| MOTOR | 3 | `/drive/motor_command_direct` | 直接调试 8 个电机 |

模式切换时控制器会先发送一次零速度命令。

## 主要 ROS 2 接口

| 名称 | 类型 | 方向 | 车型范围 | 说明 |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 输入 | 三车型 | 整车速度指令 |
| `/odometry/wheel` | `nav_msgs/msg/Odometry` | 输出 | 三车型 | 轮式/驱动插件里程计 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 输出 | 三车型 | 360° 二维激光雷达 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 输出 | 三车型 | Gazebo 仿真时钟 |
| `/sensors/gps/fix` | `sensor_msgs/msg/NavSatFix` | 输出 | 麦克纳姆 | 已验证发布的仿真 GPS 测量 |
| `/drive/set_control_mode` | `agv_interfaces/srv/SetControlMode` | 服务 | 差动舵轮 | 设置 STOP/TWIST/MODULE/MOTOR |
| `/drive/module_command_direct` | `agv_interfaces/msg/ModuleCommandArray` | 输入 | 差动舵轮 | MODULE 模式直接总成命令 |
| `/drive/motor_command_direct` | `agv_interfaces/msg/MotorCommandArray` | 输入 | 差动舵轮 | MOTOR 模式 8 轮速度命令 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 输出 | 差动舵轮 | 4 个转向关节和 8 个轮关节状态 |
| `/wheel_velocity_controller/commands` | `std_msgs/msg/Float64MultiArray` | 内部 | 差动舵轮 | 8 轮控制器目标速度 |

完整控制链接口见 [docs/interfaces.md](docs/interfaces.md)，独立节点调试方法见 [docs/debugging.md](docs/debugging.md)。

## 查看运行状态

默认差动舵轮控制器状态：

```bash
ros2 control list_controllers
ros2 control list_hardware_components
```

默认差动舵轮关节反馈：

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

当前 Nav2 配置允许 `vx`、`vy` 和 `wz`，适用于差动舵轮和麦克纳姆的全向能力。阿克曼使用前向速度和曲率转向，正式导航前需要针对非完整约束重新校准规划器参数。当前仓库附带地图仅用于配置联调；建议先完成实际 SLAM 建图，再进行导航验收。

## 可选定位融合

融合配置预留接口如下，其中麦克纳姆仿真已经发布 GPS 测量，其他传感器及完整融合仍需按应用接入和验收：

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

## 关闭仿真和键盘后台

先在键盘控制窗口按 `k` 停车，再按 `Ctrl+C` 退出键盘节点；随后在启动仿真的终端按 `Ctrl+C`。正常退出会停止 Gazebo、车辆节点和 ros_gz_bridge。

如果终端窗口已经关闭，可先用只读命令检查残留进程：

```bash
ps -eo pid,stat,etime,cmd | \
  grep -E 'ros2 launch agv_bringup|gz sim|teleop_twist_keyboard|ackermann_keyboard|parameter_bridge'
```

确认具体 PID 和命令属于本项目后再终止，避免按模糊名称误杀其他 ROS 2 任务。

## 常见问题

### 启动后车辆不响应 `/cmd_vel`

如果使用 `differential_swerve`，确认已经从默认 STOP 模式切换到 TWIST：

```bash
ros2 service call /drive/set_control_mode \
  agv_interfaces/srv/SetControlMode "{mode: 1}"
```

如果使用 `mecanum` 或 `ackermann`，它们不提供该模式服务。请检查 `/cmd_vel` 是否恰好有一个发布者和一个订阅者：

```bash
ros2 topic info /cmd_vel
```

### 阿克曼按转向键后停止

不要为阿克曼使用通用 `teleop_twist_keyboard`。通用节点的 `j/l` 会发送零线速度，阿克曼无法原地旋转。请运行：

```bash
./scripts/ackermann_teleop.sh
```

该节点让 `i` 保持前进，`j/l` 只调整转向，并在停止转向输入后自动回正。

### 找不到项目软件包

当前终端没有加载工作空间：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 默认差动舵轮控制器未激活

```bash
ros2 control list_controllers
```

使用 `differential_swerve` 时，`joint_state_broadcaster` 和 `wheel_velocity_controller` 都应为 `active`。麦克纳姆和阿克曼由 Gazebo 插件驱动，不加载这两个控制器。

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
