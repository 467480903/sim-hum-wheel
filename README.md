# G2 机器人仿真系统

基于 ROS2 Humble 的四舵轮全向机器人仿真系统：虚拟工业场景 + 全关节控制器 + SLAM/自主导航 + 网页 3D 控制台，开箱即用。

## 系统架构

```
┌─────────────────────────── 网页控制台 (http://localhost:8080) ───────────────────────────┐
│  three.js 3D 渲染 (机器人 STL + 8x8m 虚拟场景 + 激光可视化)                              │
│  示教模式: 轴运动(22 关节滑块) / 位置运动(末端拖拽 IK + 示教点位) / 底盘运动(摇杆+相对运动) │
│  地图导航: 占据栅格地图 + 点击/拖拽设目标 + 路径/位姿实时显示                              │
└─────────────────────────────────── rosbridge (ws://:9090) ──────────────────────────────┘
                                            │
┌─────────────────────────────────── ROS2 域 42 (本机隔离) ───────────────────────────────┐
│                                                                                         │
│  控制栈 control.launch.py                                                               │
│  ├─ controller_manager (mock 硬件) + 7 控制器                                            │
│  │   joint_state_broadcaster / {left_arm,right_arm,head,waist}_joint_controller          │
│  │   swerve_{steering,drive}_controller                                                  │
│  ├─ swerve_drive_node      4 舵轮运动学 (逆解 cmd_vel→轮, 正解轮→odom) + 里程计 + TF      │
│  ├─ laser_sim_node         虚拟激光 (对场景几何射线投射, 720 线) → /scan                  │
│  ├─ chassis_relative_move_node  里程计闭环的底盘相对位移 (dx, dy, dθ)                    │
│  └─ nav_goal_relay_node    网页目标点 → nav2 NavigateToPose action + 状态回传            │
│                                                                                         │
│  导航栈 nav.launch.py                                                                   │
│  └─ map_server / amcl / controller_server / planner_server / bt_navigator / behaviors    │
│     (lifecycle_manager 统一激活, 预置静态地图, 全向底盘 DWB 参数)                          │
└─────────────────────────────────────────────────────────────────────────────────────────┘
```

## 目录结构

```
sim/
├── start_all.sh              # 一键启动/停止/状态 (start|stop|restart|status)
├── env.sh                    # 环境入口: DDS 域隔离 + 系统 Python 3.10
├── sim_ws/                   # ROS2 工作空间
│   └── src/
│       ├── g2_description/   # 机器人与场景描述
│       │   ├── urdf/model.urdf        # 机器人 (腰部5 + 头部3 + 双臂各7 + 4舵轮, STL 外观)
│       │   ├── urdf/model_room.urdf   # 8x8m 虚拟工业场景 (墙体 + 隔间 + 桌子)
│       │   ├── meshes/                # 24 个 STL 网格
│       │   ├── maps/scene_map.*       # 预建静态地图 (160x160 @ 0.05m)
│       │   └── rviz/sim.rviz          # rviz2 配置
│       └── g2_control/       # 控制与功能节点
│           ├── config/controllers.yaml    # 控制器配置
│           ├── config/nav2_params.yaml    # nav2 参数 (全向底盘调优)
│           ├── launch/                    # control / nav / slam / sim 四套
│           ├── g2_control/                # 功能节点源码 (见架构图)
│           └── test/                      # 7 个控制器测试 + 集成测试脚本
├── web/                      # 网页控制台 (纯静态, 无构建)
│   ├── index.html / js/main.js   # 手写轻量 URDF 解析器 + three.js 渲染 + IK
│   └── libs/                     # three.js / STLLoader / TransformControls / roslib (本地化)
└── logs/                     # 各服务运行日志
```

## 快速开始

```bash
# 1. 构建工作空间 (首次)
cd ~/sim/sim_ws
source /opt/ros/humble/setup.bash
colcon build

# 2. 一键启动全套系统
~/sim/start_all.sh            # 或 ./start_all.sh status 查看状态

# 3. 打开控制台
#    http://localhost:8080
```

依赖的 ROS2 包：`ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-navigation2
ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-rosbridge-suite`

## 网页控制台功能

| 页签 | 功能 | 通信话题 |
|---|---|---|
| 轴运动 | 22 关节滑块 (腰5/头3/双臂7+7)，实时同步 + 分组回零 | `/*_joint_controller/joint_trajectory` |
| 位置运动 | 左右手末端 xyz 引导轴拖拽 (阻尼最小二乘 IK)、末端坐标显示、示教点位保存/回放 (localStorage) | 同上 |
| 底盘运动 | 平移摇杆 + 原地旋转 (直接速度)、里程计闭环相对位移 (前/后/左/右/转) | `/cmd_vel`、`/chassis_relative_move` |
| 地图导航 | 栅格地图 + 机器人位姿 (AMCL 优先)、点击设目标 / 拖拽定朝向、路径回放、取消导航 | `/nav_goal_pose`、`/nav_cancel`、`/nav_status` |

3D 视图：左键旋转 / 右键平移 / 滚轮缩放，支持视角跟随机器人、激光射线可视化。

## 底盘运动学 (4 舵轮)

- **逆解**：车体速度 (vx, vy, ω) → 每轮转向角 `atan2(vy + ω·lx, vx − ω·ly)` 与轮速；转向按最短角差 + 角速度限幅，转向误差大时轮速按 `cos(err)` 缩放防侧滑
- **正解**：4 轮 8 方程最小二乘解 (vx, vy, ω)，积分发布 `/odom` + TF `odom→base_link`
- 全向平移与旋转可同时进行，无需差速约束

## 测试

```bash
source env.sh
~/sim/sim_ws/src/g2_control/test/run_all_tests.sh   # 7 个控制器测试 (全部 PASS)
```

覆盖：双臂/头/腰轨迹跟踪、位置控制器、舵轮底盘正逆解一致性、底盘相对运动闭环、虚拟激光有效性。

## 已验证的端到端链路

- 网页滑块 → rosbridge → 控制器 → 关节到位 (误差 < 0.1 rad)
- 地图点击 → nav2 规划 → 舵轮执行 → AMCL 定位 → 到达目标 (终点误差 0.13m)
- 导航中取消 (accepted → canceling → canceled)

## 注意事项

- `env.sh` 设置 `ROS_LOCALHOST_ONLY=1` + `ROS_DOMAIN_ID=42`，与局域网内真实机器人 DDS 隔离，ros2 命令前必须先 `source env.sh`
- 必须使用系统 Python 3.10 (`/usr/bin/python3`)；miniconda 的 3.13 无 rclpy 会导致节点启动失败 (脚本已处理)
- `start_all.sh` 的启动顺序经过调优：rosbridge 必须在 nav2 激活风暴平息后启动 (12s 等待)，否则 `/joint_states` 订阅匹配失败、锁存地图收不到
- 每次启动机器人位姿重置为场景原点
