#!/usr/bin/env bash
# =====================================================================
# G2 仿真系统 一键启动脚本
#
# 用法:
#   ./start_all.sh            启动全部服务 (已在运行会先停止再启动, 幂等)
#   ./start_all.sh stop       停止全部服务
#   ./start_all.sh status     查看运行状态
#
# 服务链: 控制栈 -> 导航栈 -> rosbridge(:9090) -> web(:8080)
# 日志:   logs/control.log  logs/nav.log  logs/rosbridge.log  logs/web.log
#
# 说明:
#   - 所有服务经 nohup 脱离终端, 关闭终端/重启 Trae 不受影响 (重启电脑需重跑)
#   - 停止时按可执行路径精确匹配清理, 不会误杀编辑器等无关进程
#   - 每次启动机器人位姿重置为原点
# =====================================================================
set -u

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGS="$SIM_DIR/logs"
WEB_DIR="$SIM_DIR/web"
mkdir -p "$LOGS"

# ROS 环境 (setup.bash 不兼容 set -u, 加载时需临时关闭)
set +u
source "$SIM_DIR/env.sh" >/dev/null 2>&1
set -u

G='\033[32m'; R='\033[31m'; C='\033[36m'; N='\033[0m'
ok()   { printf "  ${G}[OK]${N}   %s\n" "$1"; }
bad()  { printf "  ${R}[FAIL]${N} %s\n" "$1"; }
info() { printf "  ${C}[WAIT]${N} %s\n" "$1"; }

# ---------------- 停止 ----------------
do_stop() {
  echo "==> 停止旧进程..."

  # 1) 优雅停止: SIGINT 让 ros2 launch 收尾并带走全部子进程
  pkill -INT -f 'bin/ros2 launch g2_control control\.launch\.py' 2>/dev/null
  pkill -INT -f 'bin/ros2 launch g2_control nav\.launch\.py' 2>/dev/null
  pkill -INT -f 'lib/rosbridge_server/rosbridge_websocket' 2>/dev/null
  sleep 3

  # 2) 强杀残留节点 (按可执行路径匹配, 避免误杀无关进程)
  local p
  for p in \
    '/opt/ros/humble/lib/controller_manager/ros2_control_node' \
    'sim_ws/install/g2_control/lib/g2_control/' \
    '/opt/ros/humble/lib/robot_state_publisher' \
    '/opt/ros/humble/lib/tf2_ros/static_transform_publisher' \
    '/opt/ros/humble/lib/nav2_' \
    '/opt/ros/humble/lib/rosbridge_server' \
    'ros2cli.daemon'; do
    pkill -KILL -f "$p" 2>/dev/null
  done

  # 3) 端口占用进程: web(8080) / rosbridge(9090)
  fuser -k 8080/tcp >/dev/null 2>&1
  fuser -k 9090/tcp >/dev/null 2>&1
  sleep 1

  # 4) 清理无进程引用的残留 DDS 共享内存段 (kill -9 的遗留物)
  local n=0 f
  for f in /dev/shm/fastrtps_*; do
    [ -e "$f" ] || continue
    fuser "$f" >/dev/null 2>&1 || { rm -f "$f"; n=$((n+1)); }
  done
  [ "$n" -gt 0 ] && info "清理 $n 个残留 DDS 共享内存段"

  ok "已停止"
}

# ---------------- 等待工具 ----------------
wait_until() {  # $1=描述 $2=超时秒 $3...=检查命令
  local desc=$1 tmo=$2; shift 2
  local t=0
  while :; do
    if "$@" >/dev/null 2>&1; then ok "$desc"; return 0; fi
    if [ "$t" -ge "$tmo" ]; then
      bad "$desc (超时 ${tmo}s, 请查 logs/ 日志)"
      return 1
    fi
    sleep 3; t=$((t+3))
  done
}

port_up() { ss -tln 2>/dev/null | grep -q ":$1 "; }

ctrl_ready() {   # 7 个控制器激活 (broadcaster + 4 关节轨迹 + 2 舵轮)
  local n=0
  n=$(grep -c 'Configured and activated' "$LOGS/control.log" 2>/dev/null)
  [ "$n" -ge 7 ]
}

set_start_pose() {
  info "设置机器人默认姿态 (腰部弯曲、上臂下垂、前臂前伸)..."
  ros2 topic pub --once --wait-matching-subscriptions 1 \
    /left_arm_joint_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    '{joint_names: [idx21_arm_l_joint1, idx22_arm_l_joint2, idx23_arm_l_joint3, idx24_arm_l_joint4, idx25_arm_l_joint5, idx26_arm_l_joint6, idx27_arm_l_joint7], points: [{positions: [1.5708, -1.5708, -1.5708, -1.5708, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}' || return 1
  ros2 topic pub --once --wait-matching-subscriptions 1 \
    /right_arm_joint_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    '{joint_names: [idx61_arm_r_joint1, idx62_arm_r_joint2, idx63_arm_r_joint3, idx64_arm_r_joint4, idx65_arm_r_joint5, idx66_arm_r_joint6, idx67_arm_r_joint7], points: [{positions: [-1.5708, -1.5708, 1.5708, -1.5708, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}' || return 1
  ros2 topic pub --once --wait-matching-subscriptions 1 \
    /waist_joint_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    '{joint_names: [idx01_body_joint1, idx02_body_joint2, idx03_body_joint3, idx04_body_joint4, idx05_body_joint5], points: [{positions: [-0.45, 0.7, -0.2, 0.0, 0.0], time_from_start: {sec: 3}}]}' || return 1
}

nav_ready() {    # lifecycle_manager 激活全部 nav2 节点
  grep -q 'Managed nodes are active' "$LOGS/nav.log" 2>/dev/null
}

# ---------------- 启动 ----------------
start_bg() {  # $1=服务名 $2=日志文件 $3=命令串
  nohup bash -c "$3" >"$LOGS/$2" 2>&1 &
  info "$1 启动中 (pid $!, 日志 logs/$2)"
}

do_start() {
  if [ ! -f "$SIM_DIR/sim_ws/install/setup.bash" ]; then
    bad "sim_ws 未构建, 请先: cd sim_ws && colcon build"
    exit 1
  fi
  [ -f "$WEB_DIR/index.html" ] || { bad "缺少 $WEB_DIR/index.html"; exit 1; }

  do_stop
  echo "==> 启动 G2 仿真系统..."

  start_bg 控制栈 control.log \
    "source $SIM_DIR/env.sh && ros2 launch g2_control control.launch.py"
  wait_until "控制栈就绪 (7 控制器激活)" 120 ctrl_ready || exit 1
  set_start_pose || { bad "无法设置机器人默认姿态"; exit 1; }

  start_bg 导航栈 nav.log \
    "source $SIM_DIR/env.sh && ros2 launch g2_control nav.launch.py"
  wait_until "导航栈就绪 (amcl/controller/bt_navigator)" 120 nav_ready || exit 1

  # 注意: 必须等导航栈 DDS 发现风暴平息后再启动 rosbridge,
  # 否则 rosbridge 与 controller_manager 的 /joint_states 订阅匹配
  # 会受干扰 (实测 0 包)。同时 rosbridge 晚于 map_server 启动,
  # 才能收到 transient_local 锁存的 /map 地图数据。
  info "等待导航栈 DDS 平静 (12s)..."
  sleep 12

  start_bg rosbridge rosbridge.log \
    "source $SIM_DIR/env.sh && ros2 run rosbridge_server rosbridge_websocket"
  wait_until "rosbridge 就绪 (端口 9090)" 30 port_up 9090 || exit 1

  start_bg web服务 web.log \
    "cd $WEB_DIR && python3 -m http.server 8080"
  wait_until "web 服务就绪 (端口 8080)" 15 port_up 8080 || exit 1

  echo
  echo "  =============================================="
  ok "G2 仿真系统已就绪"
  echo "      网页控制台: http://localhost:8080"
  echo "      rosbridge:  ws://localhost:9090"
  echo "      日志:       logs/{control,nav,rosbridge,web}.log"
  echo "      停止:       ./start_all.sh stop"
  echo "  =============================================="
}

# ---------------- 状态 ----------------
do_status() {
  echo "G2 仿真系统状态:"
  if pgrep -f 'controller_manager/ros2_control_node' >/dev/null 2>&1; then
    local n
    n=$(grep -c 'Configured and activated' "$LOGS/control.log" 2>/dev/null)
    ok "控制栈    运行中 (本次启动激活 ${n:-0} 个控制器)"
  else
    bad "控制栈    未运行"
  fi
  if pgrep -f '/opt/ros/humble/lib/nav2_' >/dev/null 2>&1; then
    ok "导航栈    运行中"
  else
    bad "导航栈    未运行"
  fi
  if port_up 9090; then ok "rosbridge  运行中 (:9090)"; else bad "rosbridge  未运行"; fi
  if port_up 8080; then ok "web 服务   运行中 (:8080)"; else bad "web 服务   未运行"; fi
}

# ---------------- 入口 ----------------
case "${1:-start}" in
  start|restart) do_start ;;
  stop)          do_stop ;;
  status)        do_status ;;
  *) echo "用法: $0 [start|stop|restart|status]"; exit 1 ;;
esac
