#!/usr/bin/env python3
"""准备 g2_description 的 URDF 与 mesh:

1. 规范化 mesh 路径: 大小写不匹配 -> 复制到 meshes/ 并全部小写, 改写为 package://g2_description/meshes/...
2. 删除引用不存在 convex 网格的 <collision> 块
3. 4 个舵轮转向关节 revolute -> continuous (转向 0-360)
4. 增加 SLAM 虚拟硬件: 前两轮中间向上 100mm, 直径 50mm 红色球体
5. 生成 model_control.urdf: 附加 ros2_control mock 硬件描述
"""
import os
import re
import shutil
import sys

SRC_URDF = "/home/wxf/sim/urdf/G2/model.urdf"
SRC_DIR = "/home/wxf/sim/urdf/G2"
DST_PKG = "/home/wxf/sim/sim_ws/src/g2_description"
DST_MESH = os.path.join(DST_PKG, "meshes")

os.makedirs(DST_MESH, exist_ok=True)

with open(SRC_URDF, encoding="utf-8") as f:
    xml = f.read()

# 磁盘上 mesh 文件索引 (相对路径小写 -> 实际路径)
disk = {}
for root, _dirs, files in os.walk(SRC_DIR):
    for fn in files:
        if fn.lower().endswith(".stl"):
            rel = os.path.relpath(os.path.join(root, fn), SRC_DIR)
            disk[rel.lower()] = os.path.join(root, fn)

copied = {}
missing = set()


def map_mesh(m):
    """返回 package:// 路径, 文件不存在返回 None"""
    key = m.group(1).lower()
    if key in copied:
        return copied[key]
    if key not in disk:
        missing.add(m.group(1))
        return None
    src = disk[key]
    base = os.path.basename(src).lower()
    dst = os.path.join(DST_MESH, base)
    if not os.path.exists(dst):
        shutil.copy2(src, dst)
    pkg_path = f"package://g2_description/meshes/{base}"
    copied[key] = pkg_path
    return pkg_path


# 1) 改写所有 mesh 引用
def rewrite_meshes(text):
    out = []
    last = 0
    for m in re.finditer(r'<mesh filename="([^"]+)"\s*/>', text):
        p = map_mesh(m)
        if p:
            out.append(text[last:m.start()])
            out.append(f'<mesh filename="{p}"/>')
            last = m.end()
    out.append(text[last:])
    return "".join(out)


xml = rewrite_meshes(xml)

# 2) 删除引用缺失文件的 <collision> 块 (convex 目录不存在)
def drop_bad_collisions(text):
    pattern = re.compile(r"[ \t]*<collision[\s\S]*?</collision>\n?", re.M)
    out = []
    last = 0
    for m in pattern.finditer(text):
        block = m.group(0)
        if any(p in missing for p in re.findall(r'filename="([^"]+)"', block)):
            continue  # 丢弃
        out.append(text[last:m.start()])
        out.append(block)
        last = m.end()
    out.append(text[last:])
    return "".join(out)


xml = drop_bad_collisions(xml)

# 3) 舵轮转向关节 revolute -> continuous (0-360)
STEERING_JOINTS = [
    "idx111_chassis_lwheel_front_joint1",
    "idx121_chassis_lwheel_rear_joint1",
    "idx131_chassis_rwheel_front_joint1",
    "idx141_chassis_rwheel_rear_joint1",
]
for jn in STEERING_JOINTS:
    old = f'<joint name="{jn}" type="revolute">'
    new = f'<joint name="{jn}" type="continuous">'
    if old not in xml:
        print(f"WARN: 未找到 {old}")
    xml = xml.replace(old, new)
    # continuous 关节去掉上下限, 保留 effort/velocity
    xml = re.sub(
        r'(<joint name="' + jn + r'" type="continuous">[\s\S]*?)<limit effort="15" lower="-2.97" upper="2.97" velocity="4.08"/>',
        r'\1<limit effort="15" velocity="4.08"/>',
        xml,
    )

# 4) SLAM 虚拟传感器: 前两轮中间 (x=0.23, y=0) 向上 100mm (z=0.16675+0.1), 直径50mm 红球
slam_xml = """
  <!-- ===== SLAM 虚拟硬件 (前两轮中间向上 100mm, 直径 50mm, 红色) ===== -->
  <link name="slam_sensor_link">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
      <material name="slam_red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
    </collision>
  </link>
  <joint name="slam_sensor_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0.23 0 0.26675"/>
    <parent link="chassis_link"/>
    <child link="slam_sensor_link"/>
  </joint>
"""
xml = xml.replace("</robot>", slam_xml + "</robot>")

# 写出 model.urdf
with open(os.path.join(DST_PKG, "urdf", "model.urdf"), "w", encoding="utf-8") as f:
    f.write(xml)

# 5) 生成 ros2_control 描述
BODY = [f"idx0{i}_body_joint{i}" for i in range(1, 6)]
HEAD = [f"idx1{i}_head_joint{i}" for i in range(1, 4)]
LARM = [f"idx2{i}_arm_l_joint{i}" for i in range(1, 8)]
RARM = [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)]
STEER = [j + "1" for j in ["idx111_chassis_lwheel_front_joint", "idx121_chassis_lwheel_rear_joint",
                            "idx131_chassis_rwheel_front_joint", "idx141_chassis_rwheel_rear_joint"]]
DRIVE = [j + "2" for j in ["idx112_chassis_lwheel_front_joint", "idx122_chassis_lwheel_rear_joint",
                            "idx132_chassis_rwheel_front_joint", "idx142_chassis_rwheel_rear_joint"]]

# Ready pose: bent waist, upper arms down, elbows bent 90 degrees with hands forward.
# The right arm's mounting transform mirrors the signs of joints 1 and 3.
INITIAL_POSITIONS = {
    "idx01_body_joint1": -0.45,
    "idx02_body_joint2": 0.7,
    "idx03_body_joint3": -0.2,
    "idx21_arm_l_joint1": 1.5708,
    "idx22_arm_l_joint2": -1.5708,
    "idx23_arm_l_joint3": -1.5708,
    "idx24_arm_l_joint4": -1.5708,
    "idx61_arm_r_joint1": -1.5708,
    "idx62_arm_r_joint2": -1.5708,
    "idx63_arm_r_joint3": 1.5708,
    "idx64_arm_r_joint4": -1.5708,
}


def joint_block(name, cmds):
    ifs = "\n".join(f'      <command_interface name="{c}"/>' for c in cmds)
    initial = INITIAL_POSITIONS.get(name, 0.0)
    return f"""    <joint name="{name}">
{ifs}
      <state_interface name="position">
        <param name="initial_value">{initial}</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>"""


blocks = []
for j in BODY + HEAD + LARM + RARM:
    blocks.append(joint_block(j, ["position"]))
for j in STEER:
    blocks.append(joint_block(j, ["position", "velocity"]))
for j in DRIVE:
    blocks.append(joint_block(j, ["position", "velocity"]))

rc = f"""
  <ros2_control name="g2_sim_system" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
      <param name="calculate_dynamics">false</param>
      <param name="mock_sensor_commands">false</param>
    </hardware>
{chr(10).join(blocks)}
  </ros2_control>
"""

xml_ctrl = xml.replace("</robot>", rc + "\n</robot>")
with open(os.path.join(DST_PKG, "urdf", "model_control.urdf"), "w", encoding="utf-8") as f:
    f.write(xml_ctrl)

print(f"mesh 复制: {len(copied)} 个, 缺失(已剔除collision): {len(missing)} 个")
for m in sorted(missing):
    print(f"  missing: {m}")
print("生成: model.urdf, model_control.urdf")
