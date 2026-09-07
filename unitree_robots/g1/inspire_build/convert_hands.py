"""Convert inspire hand URDFs to MJCF body snippets for merging into g1_29dof.

Steps:
1. Load each URDF with MuJoCo (mimic tags are ignored by MuJoCo's URDF loader,
   so mimic joints become regular joints; we couple them later with <equality>).
2. Save the MJCF, strip debug axis geoms (cylinders/spheres without a mesh).
3. Print the <body> subtree of the hand base link for manual/scripted merging.
"""
import os
import re
import sys
import xml.etree.ElementTree as ET

import mujoco

SRC = "/home/panu/Documents/fibo/project_humanoid/xr_teleoperate/assets/inspire_hand"
OUT = os.path.dirname(os.path.abspath(__file__))

for side, base in (("left", "L_hand_base_link"), ("right", "R_hand_base_link")):
    urdf = os.path.join(SRC, f"inspire_hand_{side}.urdf")
    spec = mujoco.MjSpec.from_file(urdf)
    spec.meshdir = SRC  # filenames already contain the meshes/ prefix
    xml_path = os.path.join(OUT, f"inspire_{side}_raw.xml")
    with open(xml_path, "w") as f:
        f.write(spec.to_xml())
    print(f"saved {xml_path}")

    tree = ET.parse(xml_path)
    root = tree.getroot()
    wb = root.find("worldbody")
    base_body = wb.find(f".//body[@name='{base}']") or wb.find("body")
    # strip debug axis geoms (non-mesh geoms in base body only)
    for geom in list(base_body.findall("geom")):
        if geom.get("type") in ("cylinder", "sphere"):
            base_body.remove(geom)
    snippet_path = os.path.join(OUT, f"inspire_{side}_body.xml")
    ET.ElementTree(base_body).write(snippet_path)
    print(f"saved {snippet_path}")

    # list joints for verification
    joints = [j.get("name") for j in base_body.iter("joint")]
    print(f"{side} joints ({len(joints)}):")
    for j in joints:
        print("  ", j)
