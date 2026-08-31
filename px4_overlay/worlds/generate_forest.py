#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import random
import math

WORLD_NAME = "random_forest.world"

# =========================
# 地图大小
# =========================

FOREST_SIZE = 80

# 原点清空区域
CLEAR_RADIUS = 3.0

# 树数量
TREE_NUM = 800

# 树间距限制
MIN_DIST = 3.0
MAX_DIST = 6.0

# 树干参数
TRUNK_RADIUS = 0.18
TRUNK_HEIGHT_MIN = 5.0
TRUNK_HEIGHT_MAX = 10.0

# 树冠参数
LEAF_RADIUS_MIN = 0.4
LEAF_RADIUS_MAX = 0.8

# =========================
# world 文件头
# =========================

world = """<?xml version="1.0" ?>
<sdf version="1.6">

<world name="forest_world">

  <include>
    <uri>model://sun</uri>
  </include>

  <include>
    <uri>model://ground_plane</uri>
  </include>

"""

# =========================
# 随机生成树位置
# =========================

tree_positions = []

attempt_limit = 100000
attempt = 0

while len(tree_positions) < TREE_NUM and attempt < attempt_limit:

    attempt += 1

    x = random.uniform(
        -FOREST_SIZE/2,
        FOREST_SIZE/2
    )

    y = random.uniform(
        -FOREST_SIZE/2,
        FOREST_SIZE/2
    )

    # 原点附近留空
    if math.sqrt(x*x + y*y) < CLEAR_RADIUS:
        continue

    valid = True

    for (tx, ty) in tree_positions:

        d = math.sqrt((x - tx)**2 + (y - ty)**2)

        # 太近
        if d < MIN_DIST:
            valid = False
            break

    if valid:
        tree_positions.append((x, y))

# =========================
# 生成树模型
# =========================

for i, (x, y) in enumerate(tree_positions):

    trunk_height = random.uniform(
        TRUNK_HEIGHT_MIN,
        TRUNK_HEIGHT_MAX
    )

    leaf_radius = random.uniform(
        LEAF_RADIUS_MIN,
        LEAF_RADIUS_MAX
    )

    crown_z = trunk_height + leaf_radius * 0.5

    world += f"""
  <model name="tree_{i}">
    <static>true</static>

    <link name="tree_link">

      <!-- 树干碰撞 -->
      <collision name="trunk_collision">
        <pose>{x:.2f} {y:.2f} {trunk_height/2:.2f} 0 0 0</pose>

        <geometry>
          <cylinder>
            <radius>{TRUNK_RADIUS}</radius>
            <length>{trunk_height:.2f}</length>
          </cylinder>
        </geometry>

      </collision>

      <!-- 树干视觉 -->
      <visual name="trunk_visual">

        <pose>{x:.2f} {y:.2f} {trunk_height/2:.2f} 0 0 0</pose>

        <geometry>
          <cylinder>
            <radius>{TRUNK_RADIUS}</radius>
            <length>{trunk_height:.2f}</length>
          </cylinder>
        </geometry>

        <material>
          <ambient>0.35 0.20 0.05 1</ambient>
          <diffuse>0.45 0.25 0.08 1</diffuse>
        </material>

      </visual>

      <!-- 树冠（仅视觉） -->
      <visual name="leaf_visual">

        <pose>{x:.2f} {y:.2f} {crown_z:.2f} 0 0 0</pose>

        <geometry>
          <sphere>
            <radius>{leaf_radius:.2f}</radius>
          </sphere>
        </geometry>

        <material>
          <ambient>0.05 0.45 0.05 1</ambient>
          <diffuse>0.10 0.65 0.10 1</diffuse>
        </material>

      </visual>

    </link>

  </model>

"""

# =========================
# world 文件尾
# =========================

world += """
</world>
</sdf>
"""

# =========================
# 写入文件
# =========================

with open(WORLD_NAME, "w") as f:
    f.write(world)

print("生成完成:", WORLD_NAME)
print("树木数量:", len(tree_positions))
