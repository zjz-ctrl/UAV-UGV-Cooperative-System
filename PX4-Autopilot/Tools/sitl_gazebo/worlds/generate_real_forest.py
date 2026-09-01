#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
生成“均匀森林” Gazebo world

特点：
1. 树木分布更加均匀（接近网格 + 随机扰动）
2. 树间距约 1m
3. 原点 2m 范围内无树（方便无人机起飞）
4. 树更高
5. 包含：
   - 草地
   - 树干（棕色圆柱）
   - 树冠（绿色球）
6. 可直接用于 Gazebo Classic
"""

import random
import math

# =========================
# 参数
# =========================

WORLD_NAME = "forest_uniform.world"

FOREST_SIZE = 60          # 森林范围（正方形）
GRID_SPACING = 1.1        # 树平均间距

CLEAR_RADIUS = 3.0        # 原点清空区域

# 树干
TRUNK_RADIUS = 0.16
TRUNK_HEIGHT_MIN = 4.0
TRUNK_HEIGHT_MAX = 7.0

# 树冠
LEAF_RADIUS_MIN = 0.7
LEAF_RADIUS_MAX = 1.1

# 随机扰动（避免太规则）
POSITION_NOISE = 0.18

# =========================
# Gazebo 文件头
# =========================

world_text = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="forest_world">

    <!-- 太阳 -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- 草地 -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

"""

# =========================
# 生成树木
# =========================

tree_id = 0

half = FOREST_SIZE / 2.0

x_values = []
y_values = []

cur = -half

while cur <= half:
    x_values.append(cur)
    cur += GRID_SPACING

cur = -half

while cur <= half:
    y_values.append(cur)
    cur += GRID_SPACING

for x_base in x_values:
    for y_base in y_values:

        # 添加随机扰动
        x = x_base + random.uniform(-POSITION_NOISE, POSITION_NOISE)
        y = y_base + random.uniform(-POSITION_NOISE, POSITION_NOISE)

        # 原点附近留空
        if math.sqrt(x*x + y*y) < CLEAR_RADIUS:
            continue

        # 随机树高
        trunk_height = random.uniform(
            TRUNK_HEIGHT_MIN,
            TRUNK_HEIGHT_MAX
        )

        # 树冠大小
        leaf_radius = random.uniform(
            LEAF_RADIUS_MIN,
            LEAF_RADIUS_MAX
        )

        # 树冠位置
        leaf_z = trunk_height + leaf_radius * 0.6

        # 树干
        world_text += f"""
    <model name="tree_{tree_id}_trunk">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {trunk_height/2:.2f} 0 0 0</pose>

      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{TRUNK_RADIUS}</radius>
              <length>{trunk_height:.2f}</length>
            </cylinder>
          </geometry>
        </collision>

        <visual name="visual">
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
      </link>
    </model>
"""

        # 树冠
        world_text += f"""
    <model name="tree_{tree_id}_leaf">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {leaf_z:.2f} 0 0 0</pose>

      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>{leaf_radius:.2f}</radius>
            </sphere>
          </geometry>
        </collision>

        <visual name="visual">
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

        tree_id += 1

# =========================
# 结束
# =========================

world_text += """
  </world>
</sdf>
"""

# =========================
# 写入 world 文件
# =========================

with open(WORLD_NAME, "w") as f:
    f.write(world_text)

print(f"生成完成: {WORLD_NAME}")
print(f"树木数量: {tree_id}")
