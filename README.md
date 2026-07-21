# 四差动驱动总成 AGV 仿真

面向 Ubuntu 24.04、ROS 2 Jazzy 与 Gazebo Harmonic 的仓储 AGV 项目。车辆由 4 个差动驱动总成组成，每个总成有两个反向安装的电机和两个接地轮，支持直行、横移、斜行与原地自转。

需要迁移到另一台 Ubuntu 电脑继续开发时，请先阅读 [Ubuntu 项目迁移与交接说明](UBUNTU_HANDOFF.md)。

## 快速开始

```bash
./scripts/install_native.sh
./scripts/build.sh
source install/setup.bash
ros2 launch agv_bringup sim.launch.py
```

建图与导航：

```bash
ros2 launch agv_bringup mapping.launch.py
ros2 run nav2_map_server map_saver_cli -f warehouse
ros2 launch agv_bringup navigation.launch.py
```

Docker：

```bash
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml run --rm agv-sim
```

## 控制模式

默认模式为 STOP。模式编号：STOP=0、TWIST=1、MODULE=2、MOTOR=3。

```bash
ros2 service call /drive/set_control_mode agv_interfaces/srv/SetControlMode "{mode: 1}"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0}, angular: {z: 0.0}}" -r 10
```

独立节点可使用 `ros2 launch agv_drive_controller <节点>_standalone.launch.py` 启动。详细接口见 [调试说明](docs/debugging.md)。

## 当前验证边界

已在 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Harmonic 原生环境完成全量构建、Xacro/URDF、控制器、雷达桥接及直行、倒车、横移、斜行和原地旋转验证。无 GPU 设备权限时 Gazebo 会回退到软件渲染，仿真实时因子和按墙钟统计的雷达频率会降低，但不影响仿真时间下的 10 Hz 配置。

初始地图仍是覆盖 40×50 m 场景的低分辨率示意图；正式导航建议先运行 SLAM，以 0.05 m 分辨率保存地图。SLAM、AMCL 和 Nav2 的完整多目标导航尚未作为本次基础仿真验收的一部分。
