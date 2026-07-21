# ROS 2 接口

| 层级 | 输入 | 输出 |
|---|---|---|
| 仲裁 | `/cmd_vel`、直接总成/电机命令 | 选中的分层命令 |
| 运动学 | `/drive/twist_command` | `/drive/module_target` |
| 舵向闭环 | 总成目标、`/joint_states` | `/drive/module_velocity_target` |
| 差动混控 | 总成速度目标 | `/drive/motor_velocity_target` |
| 电机桥 | 8 电机目标 | `/wheel_velocity_controller/commands` |
| 雷达 | Gazebo `/scan` | ROS `/scan` |

所有话题允许通过 ROS 2 remap 改名，节点支持命名空间部署。
