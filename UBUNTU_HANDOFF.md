# Ubuntu 电脑项目迁移与继续开发说明

## 1. 项目目标

本项目用于构建 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic 环境下的仓储 AGV 仿真。

车辆采用 4 个差动驱动总成，编号和位置固定为：

1. 左前
2. 右前
3. 右后
4. 左后

每个总成包含两个反向安装的电机和两个实际接地轮，共 8 个电机、8 个接地轮。两个电机的速度和旋向共同产生总成的纵向运动与绕中心轴转向。整车目标能力包括直行、倒车、横移、斜向运动和原地自转。

传感器和环境目标：

- 单线二维激光雷达，360°，0.10～20 m，默认 720 点、10 Hz。
- 40 m × 50 m 仓库场景。
- SLAM Toolbox 建图。
- AMCL 定位和 Nav2 全向导航。
- 预留 GPS、IMU 和外部里程计接口。

完整设计和实施计划见：

- `docs/superpowers/specs/2026-07-13-differential-swerve-agv-simulation-design.md`
- `docs/superpowers/plans/2026-07-13-differential-swerve-agv-simulation.md`

## 2. 当前目录结构

```text
src/
├── agv_interfaces         自定义消息和控制模式服务
├── agv_drive_controller   运动学、仲裁、舵向闭环、差动混控和里程计
├── agv_description        Xacro/URDF、雷达和 ros2_control 配置
├── agv_gazebo             Gazebo 启动和 ros_gz 桥接
├── agv_worlds             40×50 m 仓库 SDF 世界
├── agv_localization       robot_localization/EKF 和 GPS 扩展配置
├── agv_navigation         SLAM、AMCL、Nav2 和初始地图
└── agv_bringup            仿真、建图和导航统一启动入口
```

其他重要文件：

- `docker/Dockerfile`：基于官方 `ros:jazzy-ros-base-noble`。
- `docker/docker-compose.yml`：Docker 开发环境。
- `scripts/install_native.sh`：Ubuntu 原生依赖安装。
- `scripts/build.sh`：工作空间构建。
- `scripts/test.sh`：Colcon 测试。
- `docs/debugging.md`：节点独立调试和扩展传感器说明。
- `docs/interfaces.md`：主要 ROS 2 话题接口。

## 3. 已完成并验证的内容

以下结果曾在官方 `ros:jazzy-ros-base-noble` Docker 容器中实际执行：

- `agv_interfaces` 构建成功。
- `agv_drive_controller` 构建成功。
- Colcon/pytest 实际发现并执行 5 个运动学测试。
- 测试结果为 5 passed、0 errors、0 failures。

测试覆盖：

- 横移时四个总成的目标舵向。
- 原地自转时各总成的切向目标。
- 最短舵向角优化和轮速翻转。
- 反向安装电机的速度符号。
- 四总成正运动学恢复横向速度。

此外，项目文件已进行过 Python 语法、XML/SDF 解析和 PGM 像素数量静态检查。

## 4. 尚未完成或未验证的内容

以下项目不能视为已经可用，必须在新 Ubuntu 电脑继续验证和修正：

1. Xacro 尚未在安装完整依赖的 Jazzy 环境中成功展开验证。
2. `agv_description`、`agv_gazebo`、`agv_worlds`、`agv_localization`、`agv_navigation` 和 `agv_bringup` 尚未完成全量 Colcon 构建验证。
3. Gazebo 中的 4 个被动转向关节和 8 个驱动轮尚未完成物理运行验证。
4. `ros_gz_bridge` 的 `/scan`、`/clock` 桥接尚未运行验证。
5. SLAM Toolbox、AMCL 和 Nav2 尚未完成端到端验证。
6. 当前 `warehouse.pgm` 是 20×25 像素、2 m/像素的低分辨率示意图。正式导航应通过 SLAM 生成约 0.05 m/像素的地图。
7. 驱动诊断节点、完整节点级测试和自动化运动验收仍未实现。
8. Docker 完整镜像曾因原电脑的 Docker 网络/代理速度过慢而未构建完成，不是已经验证成功的镜像。

## 5. 建议的新电脑环境

- Ubuntu 24.04 LTS，x86_64。
- ROS 2 Jazzy。
- Gazebo Harmonic。
- 建议至少 16 GB 内存、4 核 CPU；运行大型仓库和 Nav2 时推荐 8 核以上。
- 若需要 Gazebo/RViz 图形界面，建议使用支持 OpenGL 的实体机环境。

## 6. 迁移项目

把整个 `agv_sim` 目录复制到 Ubuntu，例如：

```bash
mkdir -p ~/workspace
cp -a /path/to/agv_sim ~/workspace/agv_sim
cd ~/workspace/agv_sim
```

如果从 Windows 打包，建议排除以下生成目录：

```text
build/
install/
log/
__pycache__/
```

在 Ubuntu 首次构建前确认脚本可执行：

```bash
chmod +x scripts/*.sh docker/entrypoint.sh
```

## 7. Ubuntu 原生安装与构建

先检查 `scripts/install_native.sh`，确认软件源和 sudo 操作符合新电脑策略，然后执行：

```bash
cd ~/workspace/agv_sim
./scripts/install_native.sh
```

建议先分包构建已经验证过的控制层：

```bash
source /opt/ros/jazzy/setup.bash
colcon build \
  --packages-select agv_interfaces agv_drive_controller \
  --symlink-install \
  --event-handlers console_direct+

source install/setup.bash
colcon test \
  --packages-select agv_drive_controller \
  --event-handlers console_direct+
colcon test-result --verbose
```

预期控制层测试结果：

```text
5 tests, 0 errors, 0 failures, 0 skipped
```

然后执行全工作空间构建：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

若从其他电脑复制了旧的 `build/install/log`，首次构建前应删除这些生成目录，避免 Windows 路径或旧环境缓存干扰。

## 8. Docker 构建

Dockerfile 默认使用清华 TUNA Ubuntu/ROS 2 镜像，以改善中国大陆网络环境下的下载速度：

```bash
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml run --rm agv-sim
```

若要使用官方软件源：

```bash
docker build \
  --build-arg USE_CHINA_MIRROR=0 \
  -t agv-sim:jazzy \
  -f docker/Dockerfile .
```

进入容器后：

```bash
source /opt/ros/jazzy/setup.bash
./scripts/build.sh
./scripts/test.sh
```

在 Ubuntu 主机运行 GUI 时，需要根据 X11 或 Wayland 环境调整 Compose 的 `DISPLAY` 和显示套接字挂载。

## 9. 建议的继续开发顺序

### 第一步：验证 Xacro 和 URDF

```bash
source /opt/ros/jazzy/setup.bash
xacro src/agv_description/urdf/agv.urdf.xacro -o /tmp/agv.urdf
check_urdf /tmp/agv.urdf
```

重点检查：

- 4 个 `module_N_steering_joint` 均为 continuous 被动转向关节。
- 8 个轮轴关节名称和模块编号正确。
- ros2_control 中能反馈 4 个转向关节状态并控制 8 个轮速。
- 左右电机的物理正方向符合反向安装定义。
- 雷达参数为 720 点、360°、最大 20 m。

### 第二步：全量构建

优先修复 package.xml、CMakeLists.txt、launch 文件和依赖声明问题，直到：

```bash
colcon build --symlink-install
```

完整成功且无包失败。

### 第三步：启动无界面 Gazebo

```bash
source install/setup.bash
ros2 launch agv_bringup sim.launch.py
```

检查：

```bash
ros2 topic list
ros2 control list_controllers
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 topic echo /joint_states --once
```

### 第四步：逐层调试驱动

默认控制模式是 STOP。切换到 TWIST：

```bash
ros2 service call /drive/set_control_mode \
  agv_interfaces/srv/SetControlMode "{mode: 1}"
```

直行：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0}, angular: {z: 0.0}}" -r 10
```

横移：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.3}, angular: {z: 0.0}}" -r 10
```

原地自转：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.3}}" -r 10
```

依次验证 TWIST、MODULE、MOTOR、STOP 四种模式，以及超时停车和模式切换归零。

### 第五步：建图和导航

```bash
ros2 launch agv_bringup mapping.launch.py
ros2 run nav2_map_server map_saver_cli -f warehouse
ros2 launch agv_bringup navigation.launch.py
```

用实际 SLAM 生成的地图替换低分辨率示意地图，并确认 Nav2 控制器允许 `vx`、`vy`、`wz`。

### 第六步：扩展定位传感器

标准预留接口：

- IMU：`/sensors/imu/data`，`sensor_msgs/msg/Imu`。
- GPS：`/sensors/gps/fix`，`sensor_msgs/msg/NavSatFix`。
- 轮式里程计：`/odometry/wheel`。
- 外部里程计：`/odometry/external`。
- 融合里程计：`/odometry/filtered`。

启用 EKF 时必须关闭轮式里程计节点的 `publish_odom_tf`，保证只有一个节点发布 `odom → base_link`。

## 10. 已知高风险检查项

接手后优先检查以下问题：

1. `gz_ros2_control` 插件名称和 Jazzy/Harmonic 实际安装版本是否一致。
2. 被动转向关节只有状态接口时，GazeboSystem 是否正确导出其位置和速度。
3. 轮地摩擦参数是否会导致总成无法转向或侧滑严重。
4. 车辆生成与 controller spawner 是否存在启动时序问题。
5. `ros_gz_bridge` YAML 中 Gazebo 消息类型和话题名称是否正确。
6. Nav2 配置中的插件名称、参数层级和 Jazzy 版本是否匹配。
7. 初始地图分辨率过低，不能作为最终导航验收地图。
8. 当前仓库模型主要使用简化盒体，应在物理性能稳定后再细化视觉模型。

## 11. 完成验收标准

只有满足以下条件，项目才能视为完整：

- 全部 ROS 2 包构建成功。
- 全部自动测试通过。
- Gazebo 中车辆可完成直行、倒车、横移、斜向和原地自转。
- 4 个总成角度及 8 个轮速反馈正确。
- `/scan` 为 360°、最大 20 m、约 10 Hz。
- 仓库障碍物能被碰撞系统和雷达识别。
- SLAM 可保存地图。
- AMCL 可稳定定位。
- Nav2 可跨通道完成多个目标点导航。
- GPS、IMU 和外部里程计缺失时不影响默认系统。
- 启用融合时 `odom → base_link` 只有一个发布者。

## 12. Git 注意事项

原项目目录创建时不是 Git 仓库，且此前没有执行任何 Git 提交。迁移到 Ubuntu 后如需建立版本管理，可由项目负责人明确决定是否执行：

```bash
git init
git add .
git commit -m "chore(project): 初始化四差动驱动仿真工程"
```

不要在未确认的情况下提交 `build/`、`install/`、`log/` 或其他生成文件。
