# 四差动驱动总成 AGV 仿真实施计划

> **执行者必读：** 实施时必须使用 `superpowers:executing-plans`，按任务顺序一次性执行。由于用户要求一次性生成且未授权子代理，不使用子代理。所有步骤使用复选框跟踪。

**目标：** 构建可在 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic 上运行的四差动驱动总成仓储 AGV 仿真，支持分层独立调试、360° 雷达、SLAM、AMCL、Nav2、可选多源定位及原生/Docker 环境。

**架构：** 8 个接地轮由 ros2_control 速度接口驱动，4 个总成的被动垂直关节由差动轮地作用转向。控制链拆成仲裁、逆运动学、舵向闭环、差动混控、电机桥接、里程计和诊断节点，节点通过明确的 ROS 2 接口解耦。Gazebo、仓库、定位导航和传感器融合分别位于独立功能包中。

**技术栈：** Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic、ros_gz、ros2_control、Python 3/rclpy、CMake/ament、Xacro、SLAM Toolbox、Nav2、AMCL、robot_localization、Docker Compose。

## 全局约束

- 所有说明、文档、注释性文本和测试说明使用中文；ROS 2 标识符使用常规英文 snake_case。
- 总成编号固定为 1 左前、2 右前、3 右后、4 左后。
- 模型包含 4 个被动转向关节、8 个接地驱动轮和 8 个反向安装电机语义。
- 默认车体 1.2 m × 0.8 m × 0.35 m，轴距 0.9 m，轮距 0.65 m，轮径 0.20 m，雷达高度 0.65 m。
- 雷达 360°、0.10～20.0 m、默认 720 点/圈、10 Hz。
- 仓库世界固定为 40 m × 50 m，资源本地化，不依赖运行时网络。
- 所有自研 ROS 2 节点均提供独立可执行入口、参数文件和独立 launch。
- 可选 GPS、IMU、外部里程计缺失时不得阻塞默认系统。
- 未经用户明确要求，不执行 `git commit`、`git push` 或 `git rebase`；各任务末尾只执行变更检查。

## 文件结构

```text
agv_sim/
├── README.md
├── docker/{Dockerfile,docker-compose.yml,entrypoint.sh}
├── scripts/{install_native.sh,build.sh,test.sh}
├── docs/{architecture.md,interfaces.md,debugging.md}
└── src/
    ├── agv_interfaces/{msg,srv,CMakeLists.txt,package.xml}
    ├── agv_drive_controller/{agv_drive_controller,config,launch,test,setup.py,package.xml}
    ├── agv_description/{urdf,config,launch,test,CMakeLists.txt,package.xml}
    ├── agv_gazebo/{config,launch,CMakeLists.txt,package.xml}
    ├── agv_worlds/{worlds,models,launch,CMakeLists.txt,package.xml}
    ├── agv_localization/{config,launch,CMakeLists.txt,package.xml}
    ├── agv_navigation/{config,launch,maps,rviz,CMakeLists.txt,package.xml}
    └── agv_bringup/{config,launch,CMakeLists.txt,package.xml}
```

---

### 任务 1：建立接口包和工作空间骨架

**文件：**
- 创建：`src/agv_interfaces/msg/ModuleCommand.msg`
- 创建：`src/agv_interfaces/msg/ModuleCommandArray.msg`
- 创建：`src/agv_interfaces/msg/MotorCommandArray.msg`
- 创建：`src/agv_interfaces/msg/DriveStatus.msg`
- 创建：`src/agv_interfaces/srv/SetControlMode.srv`
- 创建：`src/agv_interfaces/CMakeLists.txt`
- 创建：`src/agv_interfaces/package.xml`
- 创建：`.gitignore`

**接口：**
- `ModuleCommand`：`uint8 module_id`、`float64 steering_angle`、`float64 drive_velocity`。
- `ModuleCommandArray`：`std_msgs/Header header`、`ModuleCommand[4] modules`。
- `MotorCommandArray`：`std_msgs/Header header`、`float64[8] velocity`。
- `DriveStatus`：模式常量 `STOP=0/TWIST=1/MODULE=2/MOTOR=3`，当前模式、超时状态、4 个总成角度和 8 个电机速度。
- `SetControlMode`：请求 `uint8 mode`，响应 `bool success`、`string message`。

- [ ] 写出上述完整消息、服务、ament_cmake 配置和依赖声明。
- [ ] 运行 `source /opt/ros/jazzy/setup.bash && colcon build --packages-select agv_interfaces --symlink-install`；预期退出码 0。
- [ ] 运行 `source install/setup.bash && ros2 interface show agv_interfaces/msg/MotorCommandArray`；预期显示固定长度 `float64[8] velocity`。
- [ ] 运行 `git status --short`，确认只有本任务文件变更，不提交。

### 任务 2：以测试驱动实现纯运动学与混控库

**文件：**
- 创建：`src/agv_drive_controller/agv_drive_controller/models.py`
- 创建：`src/agv_drive_controller/agv_drive_controller/kinematics.py`
- 创建：`src/agv_drive_controller/agv_drive_controller/angle_control.py`
- 创建：`src/agv_drive_controller/agv_drive_controller/module_mixer.py`
- 创建：`src/agv_drive_controller/test/test_kinematics.py`
- 创建：`src/agv_drive_controller/test/test_angle_control.py`
- 创建：`src/agv_drive_controller/test/test_module_mixer.py`
- 创建：`src/agv_drive_controller/setup.py`
- 创建：`src/agv_drive_controller/setup.cfg`
- 创建：`src/agv_drive_controller/package.xml`

**接口：**
- `ChassisTwist(vx: float, vy: float, wz: float)`。
- `ModuleTarget(module_id: int, steering_angle: float, drive_velocity: float)`。
- `SwerveKinematics(module_positions: tuple[tuple[float,float],...]).inverse(twist) -> tuple[ModuleTarget,...]`。
- `optimize_target(target_angle, target_speed, current_angle) -> tuple[float,float]`，允许翻转轮速以减少超过 π/2 的转角。
- `angle_velocity(target, current, kp, max_velocity) -> float`。
- `mix_module(drive_velocity, steering_velocity, wheel_radius, half_track, left_sign, right_sign) -> tuple[float,float]`。

- [ ] 先编写失败测试：零速保持当前角度、纵移四模块同向、横移四模块转至 π/2、自转形成切向方向、最短角优化、直行/纯转向/叠加混控及反向安装符号。
- [ ] 运行 `colcon test --packages-select agv_drive_controller --pytest-args -q`；预期因模块尚不存在而失败。
- [ ] 实现不可变数据模型、角度归一化、逆运动学、方向优化、P 控制限幅和双轮混控；所有计算拒绝非有限数。
- [ ] 再次运行同一测试；预期全部通过且 `colcon test-result --verbose` 无失败。
- [ ] 运行 `git status --short` 检查变更，不提交。

### 任务 3：实现分层独立 ROS 2 控制节点

**文件：**
- 创建：`src/agv_drive_controller/agv_drive_controller/command_arbiter.py`
- 创建：`src/agv_drive_controller/agv_drive_controller/swerve_kinematics_node.py`
- 创建：`src/agv_drive_controller/agv_drive_controller/module_angle_controller.py`
- 创建：`src/agv_drive_controller/agv_drive_controller/differential_module_mixer.py`
- 创建：`src/agv_drive_controller/agv_drive_controller/motor_command_bridge.py`
- 创建：`src/agv_drive_controller/agv_drive_controller/drive_diagnostics.py`
- 创建：`src/agv_drive_controller/config/*.yaml`
- 创建：`src/agv_drive_controller/launch/*_standalone.launch.py`
- 创建：`src/agv_drive_controller/test/test_nodes.py`
- 修改：`src/agv_drive_controller/setup.py`

**接口：**
- 仲裁输入：`/cmd_vel`、`/drive/module_command_direct`、`/drive/motor_command_direct`；输出分别为 `/drive/twist_command`、`/drive/module_command_selected`、`/drive/motor_command_selected`。
- 模式服务：`/drive/set_control_mode`；切换时发布一次零指令，默认 `STOP`。
- 运动学输出：`/drive/module_target`。
- 舵向控制输入：`/joint_states`；输出：`/drive/module_velocity_target`。
- 混控输出：`/drive/motor_velocity_target`。
- 电机桥输出：8 个 `forward_command_controller/commands` 顺序固定为模块 1～4、每模块左后右。

- [ ] 编写 launch_testing/pytest 测试：各节点可单独启动、参数可覆盖、模式切换归零、500 ms 默认超时停车、NaN 拒绝、STOP 抢占和输出数组长度检查。
- [ ] 运行节点测试并确认首次失败。
- [ ] 实现 6 个节点、入口点、逐节点 YAML 和 standalone launch；回调只做单一职责，QoS 对命令使用 reliable/keep_last(1)。
- [ ] 运行 `colcon test --packages-select agv_drive_controller`；预期全部通过。
- [ ] 逐个执行 `ros2 launch agv_drive_controller <node>_standalone.launch.py`，用 `timeout 3` 检查节点能独立存活；预期无异常退出。
- [ ] 检查变更，不提交。

### 任务 4：实现轮式里程计及可选 TF 发布

**文件：**
- 创建：`src/agv_drive_controller/agv_drive_controller/forward_kinematics.py`
- 创建：`src/agv_drive_controller/agv_drive_controller/swerve_odometry.py`
- 创建：`src/agv_drive_controller/config/odometry.yaml`
- 创建：`src/agv_drive_controller/launch/odometry_standalone.launch.py`
- 创建：`src/agv_drive_controller/test/test_forward_kinematics.py`
- 创建：`src/agv_drive_controller/test/test_odometry_node.py`

**接口：**
- `estimate_twist(module_angles, wheel_velocities, geometry) -> ChassisTwist` 使用最小二乘求解。
- 发布 `/odometry/wheel` (`nav_msgs/Odometry`)；参数 `publish_odom_tf` 默认 true。
- frame 参数默认 `odom` 与 `base_link`，协方差为可配置 6×6 对角线。

- [ ] 编写直行、横移、自转、混合运动、奇异输入和 TF 开关测试并确认失败。
- [ ] 实现正运动学、时间积分、协方差、里程计消息与互斥 TF 开关。
- [ ] 运行本包全部测试；预期通过。
- [ ] 检查变更，不提交。

### 任务 5：建立车辆 Xacro、差动总成和 ros2_control

**文件：**
- 创建：`src/agv_description/urdf/agv.urdf.xacro`
- 创建：`src/agv_description/urdf/differential_module.xacro`
- 创建：`src/agv_description/urdf/lidar.xacro`
- 创建：`src/agv_description/urdf/ros2_control.xacro`
- 创建：`src/agv_description/config/ros2_controllers.yaml`
- 创建：`src/agv_description/launch/description.launch.py`
- 创建：`src/agv_description/test/test_description.py`
- 创建：`src/agv_description/CMakeLists.txt`
- 创建：`src/agv_description/package.xml`

**接口：**
- 4 个连续型转向关节：`module_{1..4}_steering_joint`，仅状态反馈、无执行器命令。
- 8 个轮关节：`module_{1..4}_{left,right}_wheel_joint`，速度命令和位置/速度状态。
- 雷达 frame：`laser_frame`；预留固定 frame：`imu_link`、`gps_link`。

- [ ] 编写 Xacro 展开测试：关节数量与名称、模块安装坐标、轮半径、惯性非零、转向关节连续、8 个速度接口、雷达范围和预留 frame。
- [ ] 运行测试确认模型缺失导致失败。
- [ ] 实现参数化车体和总成宏；为左右轮设置正确轴向、摩擦、碰撞和电机安装符号参数。
- [ ] 添加 Gazebo Sim ros2_control 系统插件和 8 路 forward command controller。
- [ ] 运行 `xacro .../agv.urdf.xacro > /tmp/agv.urdf` 与 `check_urdf /tmp/agv.urdf`；预期解析成功。
- [ ] 运行本包测试并检查变更，不提交。

### 任务 6：集成 Gazebo Harmonic、车辆生成和 ROS 桥接

**文件：**
- 创建：`src/agv_gazebo/config/bridge.yaml`
- 创建：`src/agv_gazebo/launch/simulation.launch.py`
- 创建：`src/agv_gazebo/launch/robot_spawn.launch.py`
- 创建：`src/agv_gazebo/CMakeLists.txt`
- 创建：`src/agv_gazebo/package.xml`
- 创建：`src/agv_gazebo/test/test_bridge_config.py`

**接口：**
- 桥接 `/scan`、`/clock` 和必要的 Gazebo 状态；`use_sim_time=true`。
- 生成位置参数 `x/y/z/yaw`，默认位于装卸区。

- [ ] 编写桥接 YAML、launch 参数和控制器启动顺序的静态测试并确认失败。
- [ ] 实现 Gazebo 启动、robot_state_publisher、create、controller_manager spawner 和桥接的事件顺序。
- [ ] 运行静态测试；预期通过。
- [ ] 在 Ubuntu 环境运行 `ros2 launch agv_gazebo simulation.launch.py headless:=true`，检查 `/clock`、`/scan`、`/joint_states` 和 8 路控制器；预期均存在。
- [ ] 检查变更，不提交。

### 任务 7：创建 40 m × 50 m 本地仓库世界

**文件：**
- 创建：`src/agv_worlds/worlds/warehouse_40x50.sdf`
- 创建：`src/agv_worlds/models/{rack,pallet,box,loading_zone}/model.config`
- 创建：`src/agv_worlds/models/{rack,pallet,box,loading_zone}/model.sdf`
- 创建：`src/agv_worlds/launch/warehouse.launch.py`
- 创建：`src/agv_worlds/test/test_world.py`
- 创建：`src/agv_worlds/CMakeLists.txt`
- 创建：`src/agv_worlds/package.xml`

**接口：**
- 世界边界 40×50 m；出生区保持至少 4×4 m 无障碍。
- 所有墙、架、托盘、箱体同时具有 visual 和 collision。

- [ ] 编写 SDF XML 测试：尺寸、模型数量、唯一名称、本地 URI、碰撞体、灯光和出生区净空。
- [ ] 运行测试确认失败。
- [ ] 实现地面、墙、装卸区、重复货架、通道、托盘和箱体；collision 使用简单盒体。
- [ ] 使用 `gz sdf -k warehouse_40x50.sdf` 验证；预期无错误。
- [ ] 运行世界测试并检查变更，不提交。

### 任务 8：实现可选多源定位接口

**文件：**
- 创建：`src/agv_localization/config/ekf.yaml`
- 创建：`src/agv_localization/config/ekf_gps.yaml`
- 创建：`src/agv_localization/config/navsat_transform.yaml`
- 创建：`src/agv_localization/launch/localization_fusion.launch.py`
- 创建：`src/agv_localization/test/test_localization_config.py`
- 创建：`src/agv_localization/CMakeLists.txt`
- 创建：`src/agv_localization/package.xml`

**接口：**
- 输入 `/odometry/wheel`、可选 `/sensors/imu/data`、`/sensors/gps/fix`、`/odometry/external`。
- 输出 `/odometry/filtered`；启用融合时由 EKF 唯一发布 `odom → base_link`。
- launch 参数 `use_imu/use_gps/use_external_odom` 默认 false。

- [ ] 编写 YAML 维数、话题、frame、开关和 TF 唯一发布者测试并确认失败。
- [ ] 实现默认轮式 EKF、IMU/外部里程计融合和 GPS+navsat_transform 配置。
- [ ] 确保 launch 关闭融合时不启动任何融合节点，打开时向底盘里程计覆盖 `publish_odom_tf:=false`。
- [ ] 运行测试；预期通过并检查变更，不提交。

### 任务 9：配置 SLAM、AMCL 与全向 Nav2

**文件：**
- 创建：`src/agv_navigation/config/slam_toolbox.yaml`
- 创建：`src/agv_navigation/config/nav2.yaml`
- 创建：`src/agv_navigation/launch/mapping.launch.py`
- 创建：`src/agv_navigation/launch/localization.launch.py`
- 创建：`src/agv_navigation/launch/navigation.launch.py`
- 创建：`src/agv_navigation/maps/warehouse.yaml`
- 创建：`src/agv_navigation/maps/warehouse.pgm`
- 创建：`src/agv_navigation/rviz/navigation.rviz`
- 创建：`src/agv_navigation/test/test_navigation_config.py`
- 创建：`src/agv_navigation/CMakeLists.txt`
- 创建：`src/agv_navigation/package.xml`

**接口：**
- Nav2 输出 `/cmd_vel`；输入 `/scan`、TF 和选择后的里程计。
- 控制器启用 holonomic 速度采样，`min_vel_y < 0 < max_vel_y`，足迹匹配 1.2×0.8 m 车体并留安全余量。

- [ ] 编写配置测试：SLAM/AMCL frame 一致、Nav2 全向 y 速度非零、scan 观察源、地图路径存在、生命周期节点完整。
- [ ] 运行测试确认失败。
- [ ] 实现建图、定位、导航三个独立 launch；导航控制器设置保守速度/加速度和舵向建立约束。
- [ ] 生成与 SDF 固定几何一致的初始占据栅格地图，分辨率 0.05 m，原点覆盖 40×50 m。
- [ ] 运行配置测试；预期通过并检查变更，不提交。

### 任务 10：统一 Bringup、键盘调试与模式示例

**文件：**
- 创建：`src/agv_bringup/launch/sim.launch.py`
- 创建：`src/agv_bringup/launch/mapping.launch.py`
- 创建：`src/agv_bringup/launch/navigation.launch.py`
- 创建：`src/agv_bringup/config/defaults.yaml`
- 创建：`src/agv_bringup/CMakeLists.txt`
- 创建：`src/agv_bringup/package.xml`
- 修改：各控制节点 standalone launch，统一参数名和命名空间传递。

**接口：**
- `sim.launch.py` 参数：`world/gui/headless/use_rviz/namespace/use_localization_fusion`。
- mapping/navigation 复用仿真入口，不复制车辆启动逻辑。

- [ ] 编写 launch 描述静态测试，检查参数透传、节点唯一性和模式默认 STOP。
- [ ] 实现统一入口和独立调试入口。
- [ ] 用 `ros2 service call /drive/set_control_mode ...` 切换 TWIST/MODULE/MOTOR/STOP，并分别给出可复制的 `ros2 topic pub` 命令。
- [ ] 运行 launch 测试并检查变更，不提交。

### 任务 11：提供原生 Ubuntu 与 Docker 环境

**文件：**
- 创建：`scripts/install_native.sh`
- 创建：`scripts/build.sh`
- 创建：`scripts/test.sh`
- 创建：`docker/Dockerfile`
- 创建：`docker/docker-compose.yml`
- 创建：`docker/entrypoint.sh`
- 创建：`.dockerignore`

**接口：**
- 镜像基础为 `ubuntu:24.04`，安装 ROS 2 Jazzy、Gazebo Harmonic、ros_gz、Nav2、SLAM Toolbox、robot_localization 和测试工具。
- Compose 挂载源码并提供 X11/Wayland 所需环境变量，默认命令为 bash。

- [ ] 编写 ShellCheck 和 Compose 配置验证步骤。
- [ ] 实现幂等原生安装、rosdep、colcon 构建和测试脚本。
- [ ] 运行 `shellcheck scripts/*.sh docker/entrypoint.sh`；预期无 error。
- [ ] 运行 `docker compose -f docker/docker-compose.yml config`；预期输出有效配置。
- [ ] 若允许构建，运行 `docker compose -f docker/docker-compose.yml build`；预期成功。
- [ ] 检查变更，不提交。

### 任务 12：编写中文文档与独立调试手册

**文件：**
- 创建：`README.md`
- 创建：`docs/architecture.md`
- 创建：`docs/interfaces.md`
- 创建：`docs/debugging.md`

**内容：**
- 原生与 Docker 安装、构建、启动、建图、保存地图、定位、导航完整命令。
- 每个节点的职责、输入输出、参数、独立启动和注入测试命令。
- 4 总成/8 电机编号、正方向、控制模式和 TF 树。
- GPS、IMU、外部里程计接入及 TF 发布权切换示例。

- [ ] 编写文档链接检查脚本或使用 `lychee`（若环境已有）；至少用 `rg` 检查所有 launch 和话题名与源码一致。
- [ ] 补齐所有中文文档，不保留 TBD/TODO。
- [ ] 运行 `rg -n 'TBD|TODO|待补充|XXX' README.md docs src`；预期无结果。
- [ ] 检查变更，不提交。

### 任务 13：执行全量验证与验收场景

**文件：**
- 创建：`src/agv_bringup/test/test_sim_smoke.py`
- 创建：`src/agv_bringup/test/test_motion_acceptance.py`
- 创建：`src/agv_bringup/test/test_navigation_smoke.py`
- 修改：`scripts/test.sh`

**验收动作：**
- [ ] 运行 `rosdep install --from-paths src --ignore-src -r -y`；预期依赖解析完成。
- [ ] 运行 `colcon build --symlink-install --event-handlers console_direct+`；预期所有包构建成功。
- [ ] 运行 `colcon test --event-handlers console_direct+ && colcon test-result --verbose`；预期 0 个失败。
- [ ] 无界面启动仿真，验证 `/scan` 的 `angle_min≈-π`、`angle_max≈π`、`range_max=20.0`、720 点和约 10 Hz。
- [ ] TWIST 模式依次发送直行、倒车、横移、斜向和自转指令；比较 10 秒前后真值/里程计，预期主方向正确、非目标漂移在配置容差内。
- [ ] MODULE 模式直接控制 4 个总成，确认各模块方向与速度独立可观测。
- [ ] MOTOR 模式逐一驱动 8 个轮，确认编号和安装符号完全符合规格。
- [ ] 验证指令超时、STOP 抢占、模式切换先归零和非法数值拒绝。
- [ ] 启动 SLAM，确认地图更新并可保存；启动 AMCL，确认位姿收敛；启动 Nav2，向至少 3 个跨通道目标导航并确认到达。
- [ ] 分别启用/禁用融合配置，确认 `/odometry/filtered` 和 `odom → base_link` 只有一个发布者；缺少 GPS/IMU 时默认系统正常运行。
- [ ] 运行 `git diff --check`；预期无空白错误。运行 `git status --short` 汇总全部生成文件，不提交。

## 实施完成条件

只有任务 1～13 全部完成、可执行环境内的构建与测试输出已记录、无法在当前 Windows 主机运行的 Ubuntu/Gazebo 验证明确标注为环境限制时，才能报告完成。不得把静态配置检查描述为真实 Gazebo 运行验证。
