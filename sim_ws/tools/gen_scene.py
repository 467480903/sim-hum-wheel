#!/usr/bin/env python3
"""从 scene.yaml 生成:
1. model_room.urdf  (虚拟场景 URDF, 供 RViz / web 显示)
2. maps/scene_map.pgm + scene_map.yaml (occupancy 地图, 供 nav2)
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, "..", "src", "g2_description")

import yaml

with open(os.path.join(PKG, "config", "scene.yaml"), encoding="utf-8") as f:
    spec = yaml.safe_load(f)

boxes = spec["boxes"]
sx, sy = spec["scene"]["size"]
ox, oy = spec["scene"]["origin"]

# ---------------- model_room.urdf ----------------
links = []
joints = []


def add_box(name, cx, cy, lx, ly, h, color):
    links.append(f"""
  <link name="{name}">
    <visual>
      <origin rpy="0 0 0" xyz="{cx} {cy} {h / 2.0}"/>
      <geometry><box size="{lx} {ly} {h}"/></geometry>
      <material name="{name}_mat"><color rgba="{color}"/></material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="{cx} {cy} {h / 2.0}"/>
      <geometry><box size="{lx} {ly} {h}"/></geometry>
    </collision>
  </link>""")
    joints.append(f"""
  <joint name="{name}_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="room_world"/>
    <child link="{name}"/>
  </joint>""")


# 地面 (比场景略大, 灰色)
add_box("room_floor", 0.0, 0.0, sx + 0.4, sy + 0.4, 0.02, "0.55 0.55 0.55 1.0")
for b in boxes:
    cx, cy = b["center"]
    lx, ly = b["size"]
    if b["name"].startswith("room_"):
        color = "0.75 0.66 0.42 1.0"   # 房间墙: 木质色
    elif b["name"].startswith("table"):
        color = "0.35 0.55 0.35 1.0"   # 桌子: 绿色
    else:
        color = "0.7 0.7 0.72 1.0"     # 边缘墙: 浅灰
    add_box(b["name"], cx, cy, lx, ly, b["height"], color)

urdf = f"""<?xml version="1.0"?>
<!-- 虚拟工业场景 8m x 8m, 自动生成自 scene.yaml, 勿手改 -->
<robot name="model_room">
  <link name="room_world"/>
{''.join(links)}
{''.join(joints)}
</robot>
"""
with open(os.path.join(PKG, "urdf", "model_room.urdf"), "w", encoding="utf-8") as f:
    f.write(urdf)

# ---------------- occupancy 地图 ----------------
res = 0.05                                   # 5cm/像素
W = int(round(sx / res))
H = int(round(sy / res))
# 0=占用 254=自由 205=未知; 初始全自由
grid = [[254] * W for _ in range(H)]


def to_px(x, y):
    # 世界 -> 像素, pgm 第 0 行在顶部 (y 最大)
    col = int((x - ox) / res)
    row = H - 1 - int((y - oy) / res)
    return row, col


for b in boxes:
    cx, cy = b["center"]
    lx, ly = b["size"]
    r0, c0 = to_px(cx - lx / 2, cy + ly / 2)   # 左上
    r1, c1 = to_px(cx + lx / 2, cy - ly / 2)   # 右下
    for r in range(max(0, r0), min(H, r1 + 1)):
        for c in range(max(0, c0), min(W, c1 + 1)):
            grid[r][c] = 0

# PGM P5
os.makedirs(os.path.join(PKG, "maps"), exist_ok=True)
with open(os.path.join(PKG, "maps", "scene_map.pgm"), "wb") as f:
    f.write(f"P5\n{W} {H}\n255\n".encode())
    f.write(bytes(v for row in grid for v in row))

map_yaml = f"""image: scene_map.pgm
resolution: {res}
origin: [{ox}, {oy}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
mode: trinary
"""
with open(os.path.join(PKG, "maps", "scene_map.yaml"), "w", encoding="utf-8") as f:
    f.write(map_yaml)

print(f"model_room.urdf: {len(boxes)} 个障碍物 + 地面")
print(f"scene_map.pgm: {W}x{H} @ {res}m")
