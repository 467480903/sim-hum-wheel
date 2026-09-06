# G2 仿真环境统一入口: source 本文件后再执行 ros2 命令
# 注意: 局域网内存在其他 ROS2 系统(真实机器人), 必须隔离:
#   - ROS_LOCALHOST_ONLY=1  禁止跨机 DDS 发现
#   - ROS_DOMAIN_ID=42      独立域 ID
#   - ROS_LOG_DIR           沙箱禁止写 ~/.ros/log, 重定向到工作区
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
export ROS_LOG_DIR=/home/wxf/sim/ros_logs
# python3 必须用系统 3.10: miniconda 的 3.13 没有 rclpy, 会导致 rosbridge 等节点启动失败
export PATH=/usr/bin:$PATH
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
[ -f /home/wxf/sim/sim_ws/install/setup.bash ] && source /home/wxf/sim/sim_ws/install/setup.bash
