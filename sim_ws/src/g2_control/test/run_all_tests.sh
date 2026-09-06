#!/bin/bash
# 依次运行全部控制器测试 (前置: control.launch.py 已启动, 且已 source env.sh)
# 注意: 必须用 /usr/bin/python3 (ROS Humble 的 3.10), 不能用 miniconda 的 3.13
# 用法: ./run_all_tests.sh
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
PY=/usr/bin/python3
PASS=0; FAIL=0; FAILED=()

for t in test_head_joint_controller.py \
         test_arm_joint_controllers.py \
         test_waist_joint_controller.py \
         test_laser_scan.py \
         test_position_controllers.py \
         test_swerve_chassis.py \
         test_chassis_relative_move.py; do
    echo "=============================================="
    echo "== 运行 $t"
    echo "=============================================="
    if "$PY" "$DIR/$t"; then
        PASS=$((PASS+1))
    else
        FAIL=$((FAIL+1)); FAILED+=("$t")
    fi
done

echo "=============================================="
echo "总计: $PASS 通过, $FAIL 失败"
[ $FAIL -gt 0 ] && printf '失败: %s\n' "${FAILED[@]}"
exit $FAIL
