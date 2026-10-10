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
│           ├── launch/                    # control / nav / rosbridge / web / slam
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

### 分终端启动并查看日志

调试时打开**四个不同的终端**，每个终端保持一个 launch 命令运行，日志会实时显示在那里。启动前，先停止通过 `start_all.sh` 运行的旧服务，避免同名 ROS 节点和 8080/9090 端口冲突。

**每个新终端都必须执行 `source /home/wxf/sim/env.sh`。**它会加载 ROS 2 Humble 和 `sim_ws/install`，并设置本项目的 ROS 域。仅执行 `source /home/wxf/sim/sim/bin/activate` 只会激活 Python 虚拟环境，不能让 ROS 找到 `g2_description` 和 `g2_control`。若首次运行尚未构建工作空间，先按上面的「快速开始」执行 `colcon build`。以下命令直接使用源码里的 launch 文件，修改 launch 文件后无需重新构建。

**终端 1：控制栈**（mock 硬件、控制器、TF、舵轮运动学、激光及功能节点）：

```bash
cd /home/wxf/sim
source /home/wxf/sim/env.sh
ros2 pkg prefix g2_description  # 应输出 /home/wxf/sim/sim_ws/install/g2_description
ros2 launch /home/wxf/sim/sim_ws/src/g2_control/launch/control.launch.py
```

等待终端显示 7 个默认控制器已激活，再启动导航。

**终端 2：导航栈**（地图、定位、规划、导航）：

```bash
cd /home/wxf/sim
source /home/wxf/sim/env.sh
ros2 launch /home/wxf/sim/sim_ws/src/g2_control/launch/nav.launch.py
```

等待终端显示 `Managed nodes are active`，再启动 rosbridge。`start_all.sh` 在这里还会等待 12 秒，以避开 DDS 发现高峰；手动启动时也建议稍等片刻。

**终端 3：rosbridge**（网页与 ROS 通信）：

```bash
cd /home/wxf/sim
source /home/wxf/sim/env.sh
ros2 launch /home/wxf/sim/sim_ws/src/g2_control/launch/rosbridge.launch.py
```

**终端 4：网页服务**（访问日志显示在当前终端）：

```bash
cd /home/wxf/sim
source /home/wxf/sim/env.sh
ros2 launch /home/wxf/sim/sim_ws/src/g2_control/launch/web.launch.py
```

打开 `http://localhost:8080`。按 `Ctrl+C` 停止对应终端的服务组。若看到 `PackageNotFoundError: package 'g2_description' not found`，在**报错的同一个终端**重新执行 `source /home/wxf/sim/env.sh`，并运行 `ros2 pkg prefix g2_description` 检查工作空间是否已加载。单独启动 `control.launch.py` 时不会执行 `start_all.sh` 中的默认姿态设置，机器人以硬件初始姿态启动。

如需 RViz，在另一个已执行 `source /home/wxf/sim/env.sh` 的终端运行 `ros2 run rviz2 rviz2 -d /home/wxf/sim/sim_ws/src/g2_description/rviz/sim.rviz`。

`slam.launch.py` 保留用于**重新建图**：它会启动控制栈、RViz 和 `slam_toolbox`，与上述正常导航流程不同。使用时不要另外启动 `control.launch.py` 或 `nav.launch.py`，以免节点重复；在已加载 `env.sh` 的终端运行 `ros2 launch /home/wxf/sim/sim_ws/src/g2_control/launch/slam.launch.py`。

## 独立控制器启动与切换

`control.launch.py` 会先启动 `controller_manager`、TF、mock 硬件及功能节点；随后默认激活关节状态、双臂/头/腰的轨迹控制器和两个舵轮控制器。位置控制器及底盘直接控制器会被**加载为 inactive**，可按需独立激活。

> 以下命令必须在控制栈已运行时执行。若只调试控制器，可在一个终端启动控制栈：
>
> ```bash
> cd /home/wxf/sim
> source /home/wxf/sim/env.sh
> ros2 launch /home/wxf/sim/sim_ws/src/g2_control/launch/control.launch.py
> ```
>
> 另开一个终端执行下列命令；两个终端都需要先 `source ~/sim/env.sh`。不要同时执行 `start_all.sh` 和 `control.launch.py`，否则会启动两套同名 ROS 节点。

### 查看当前控制器状态

```bash
cd ~/sim
source env.sh
ros2 control list_controllers -c /controller_manager
```

默认 active 的控制器：

| 控制器 | 类型 | 控制对象 | 命令接口 |
|---|---|---|---|
| `joint_state_broadcaster` | 状态广播 | 全部关节状态 | 无 |
| `left_arm_joint_controller` | 轨迹 | 左臂 7 关节 | `/left_arm_joint_controller/joint_trajectory` |
| `right_arm_joint_controller` | 轨迹 | 右臂 7 关节 | `/right_arm_joint_controller/joint_trajectory` |
| `head_joint_controller` | 轨迹 | 头部 3 关节 | `/head_joint_controller/joint_trajectory` |
| `waist_joint_controller` | 轨迹 | 腰部 5 关节 | `/waist_joint_controller/joint_trajectory` |
| `swerve_steering_controller` | 位置组控制 | 4 个舵轮转向关节 | `/swerve_steering_controller/commands` (`Float64MultiArray`) |
| `swerve_drive_controller` | 速度组控制 | 4 个舵轮驱动关节 | `/swerve_drive_controller/commands` (`Float64MultiArray`) |

预加载但 inactive 的控制器：`left_arm_position_controller`、`right_arm_position_controller`、`waist_position_controller`、`chassis_joint_controller`、`chassis_position_controller`。

### 单独启用位置控制器

同一组关节的轨迹控制器与位置控制器不能同时 active。先停用对应轨迹控制器，再启用位置控制器；例如，切换左臂：

```bash
source ~/sim/env.sh
ros2 control switch_controllers \
  --deactivate left_arm_joint_controller \
  --activate left_arm_position_controller \
  --strict -c /controller_manager
```

右臂或腰部使用相同模式：

```bash
# 右臂
ros2 control switch_controllers \
  --deactivate right_arm_joint_controller \
  --activate right_arm_position_controller \
  --strict -c /controller_manager

# 腰部
ros2 control switch_controllers \
  --deactivate waist_joint_controller \
  --activate waist_position_controller \
  --strict -c /controller_manager
```

位置控制器通过 `/<controller_name>/commands` 接收 `std_msgs/msg/Float64MultiArray`；数组元素顺序与 `sim_ws/src/g2_control/config/controllers.yaml` 中该控制器的 `joints` 顺序完全一致。例如向左臂发送 7 个目标位置：

```bash
ros2 topic pub --once /left_arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray '{data: [0.0, 0.2, -0.3, 0.0, 0.0, 0.0, 0.0]}'
```

切回默认轨迹控制器：

```bash
ros2 control switch_controllers \
  --deactivate left_arm_position_controller \
  --activate left_arm_joint_controller \
  --strict -c /controller_manager
```

### 单独启用底盘直接控制器

`chassis_joint_controller`（8 关节轨迹）和 `chassis_position_controller`（8 关节直接位置）用于直接测试底盘关节；正常移动底盘应保持 `swerve_steering_controller`、`swerve_drive_controller` active，并向 `/cmd_vel` 发布速度，由 `swerve_drive_node` 完成运动学解算。

直接控制底盘时，为避免与舵轮控制器争用关节命令，先停用两个舵轮控制器，再激活**其中一个**底盘控制器：

```bash
# 激活底盘位置控制器
ros2 control switch_controllers \
  --deactivate swerve_steering_controller swerve_drive_controller \
  --activate chassis_position_controller \
  --strict -c /controller_manager

# 恢复正常舵轮运动学控制
ros2 control switch_controllers \
  --deactivate chassis_position_controller \
  --activate swerve_steering_controller swerve_drive_controller \
  --strict -c /controller_manager
```

如需使用 `chassis_joint_controller`，将上例中的 `chassis_position_controller` 替换为 `chassis_joint_controller`；其轨迹话题为 `/chassis_joint_controller/joint_trajectory`。不要同时激活 `chassis_joint_controller` 和 `chassis_position_controller`。

### 加载未预加载的控制器

当前 `control.launch.py` 已加载全部已配置控制器，通常只需切换 active/inactive 状态。若后续新增了 YAML 中的控制器且尚未加载，可使用：

```bash
source ~/sim/env.sh
ros2 control load_controller <controller_name> \
  --set-state inactive -c /controller_manager
```

再用 `ros2 control switch_controllers --activate <controller_name> ...` 激活它。每次切换后建议执行 `ros2 control list_controllers -c /controller_manager` 确认状态。

## 网页控制台功能

| 页签 | 功能 | 通信话题 |
|---|---|---|
| 轴运动 | 22 关节滑块 (腰5/头3/双臂7+7)，实时同步 + 分组回零 | `/*_joint_controller/joint_trajectory` |
| 位置运动 | 进入时自动将双臂切换到位置控制器；末端 xyz 引导轴拖拽 (阻尼最小二乘 IK)、步进和示教点位 | `/*_arm_position_controller/commands`；离开回到轨迹控制器 |
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
