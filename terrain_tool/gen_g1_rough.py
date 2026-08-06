"""
Generate a large rough ground scene for G1 robot.
Output: ../unitree_robots/g1/scene_rough.xml

Usage:
    cd terrain_tool/
    python3 gen_g1_rough.py

Author: Zhang Zhen (zhangzhen@cmhi.chinamobile.com)
"""

import xml.etree.ElementTree as xml_et
import numpy as np
import sys
import os

# Reuse TerrainGenerator from terrain_generator.py
sys.path.insert(0, os.path.dirname(__file__))

# Override global config before importing
import terrain_generator as tg_mod
tg_mod.ROBOT = "g1"
tg_mod.INPUT_SCENE_PATH = os.path.join(os.path.dirname(__file__), "scene_g1.xml")
tg_mod.OUTPUT_SCENE_PATH = os.path.join(os.path.dirname(__file__), "../unitree_robots/g1/scene_rough.xml")

from terrain_generator import TerrainGenerator

if __name__ == "__main__":
    tg = TerrainGenerator()

    # Large rough ground centered around the robot spawn point
    # Total area ~10m x 8m, robot spawns near origin
    tg.AddRoughGround(
        init_pos=[-5.0, -4.0, 0.0],   # offset so terrain surrounds origin
        euler=[0, 0, 0.0],
        nums=[20, 16],                 # 20x16 blocks -> ~10m x 8m
        box_size=[0.5, 0.5, 0.08],     # thin blocks, height 8cm
        box_euler=[0.0, 0.0, 0.0],
        separation=[0.5, 0.5],         # spacing matches box size for full coverage
        box_size_rand=[0.05, 0.05, 0.06],   # random height variation up to ±6cm
        box_euler_rand=[0.08, 0.08, 0.0],   # slight random tilt
        separation_rand=[0.02, 0.02],
    )

    tg.Save()
    print(f"Saved to: {tg_mod.OUTPUT_SCENE_PATH}")
    print("Run simulator with: ./unitree_mujoco -s scene_rough.xml")
