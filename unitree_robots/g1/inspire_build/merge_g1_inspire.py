"""Build g1_29dof_inspire_fixed.xml:

G1 29-DoF body (pelvis welded to the world, no freejoint) with Inspire hands
attached to both wrist yaw links. Adds:
- 12 position actuators for the driven finger joints, ordered to match the
  Unitree inspire DDS convention (ids 0-5 right hand, 6-11 left hand).
- equality couplings replacing the URDF mimic joints.

Mount transform comes from the official h1_2.urdf inspire mount:
  left : pos 0.054 0 0, rpy(0,0,+pi/2)   -> quat (0.7071068, 0, 0, 0.7071068)
  right: pos 0.054 0 0, rpy(pi,0,-pi/2)  -> quat (0, 0.7071068, -0.7071068, 0)
"""
import os
import re
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
G1_DIR = os.path.dirname(HERE)

SQ2 = 0.70710678118654757

HAND_JOINT_ATTRS = {"damping": "0.05", "armature": "0.002", "frictionloss": "0.01"}

# inspire DDS id -> (joint name, min, max) ; q_norm = (max - q)/(max-min)
DDS_ORDER = [
    ("R_pinky_proximal_joint", 0.0, 1.7),
    ("R_ring_proximal_joint", 0.0, 1.7),
    ("R_middle_proximal_joint", 0.0, 1.7),
    ("R_index_proximal_joint", 0.0, 1.7),
    ("R_thumb_proximal_pitch_joint", 0.0, 0.5),
    ("R_thumb_proximal_yaw_joint", -0.1, 1.3),
    ("L_pinky_proximal_joint", 0.0, 1.7),
    ("L_ring_proximal_joint", 0.0, 1.7),
    ("L_middle_proximal_joint", 0.0, 1.7),
    ("L_index_proximal_joint", 0.0, 1.7),
    ("L_thumb_proximal_pitch_joint", 0.0, 0.5),
    ("L_thumb_proximal_yaw_joint", -0.1, 1.3),
]

# mimic couplings: dependent joint = multiplier * driver joint
MIMICS = []
for s in ("L", "R"):
    for f in ("index", "middle", "ring", "pinky"):
        MIMICS.append((f"{s}_{f}_intermediate_joint", f"{s}_{f}_proximal_joint", 1.0))
    MIMICS.append((f"{s}_thumb_intermediate_joint", f"{s}_thumb_proximal_pitch_joint", 1.6))
    MIMICS.append((f"{s}_thumb_distal_joint", f"{s}_thumb_proximal_pitch_joint", 2.4))


# base-link inertials from the URDFs (dropped by the URDF importer since the
# root link becomes the worldbody). fullinertia order: ixx iyy izz ixy ixz iyz
BASE_INERTIAL = {
    "left": {
        "pos": "-0.002551 -0.066047 -0.0019357",
        "mass": "0.14143",
        "fullinertia": "0.0001234 8.3835e-05 7.7231e-05 2.1995e-06 -1.7694e-06 1.5968e-06",
    },
    "right": {
        "pos": "-0.0025264 -0.066047 0.0019598",
        "mass": "0.14143",
        "fullinertia": "0.00012281 8.3832e-05 7.6663e-05 2.1711e-06 1.7709e-06 -1.6551e-06",
    },
}


def load_hand(side: str) -> tuple[list[ET.Element], ET.Element]:
    """Return (mesh asset elements, hand base body element).

    The URDF importer turns the root link (X_hand_base_link) into the
    worldbody, so we rebuild it as a proper body and move the base geoms and
    finger subtrees into it.
    """
    raw = ET.parse(os.path.join(HERE, f"inspire_{side}_raw.xml")).getroot()
    meshes = list(raw.find("asset").findall("mesh"))
    for m in meshes:
        # file paths become relative to g1 meshdir="meshes" (STLs copied there)
        m.set("file", os.path.basename(m.get("file")))
        m.attrib.pop("content_type", None)
    base_name = "L_hand_base_link" if side == "left" else "R_hand_base_link"
    wb = raw.find("worldbody")

    base = ET.Element("body", {"name": base_name})
    ET.SubElement(base, "inertial", BASE_INERTIAL[side])
    for geom in wb.findall("geom"):
        # keep only base mesh geoms, drop URDF debug axis cylinders/spheres
        if geom.get("mesh") == base_name:
            base.append(geom)
    for body in wb.findall("body"):
        base.append(body)

    # stabilizing joint params
    for j in base.iter("joint"):
        for k, v in HAND_JOINT_ATTRS.items():
            j.set(k, v)
    return meshes, base


def main() -> None:
    tree = ET.parse(os.path.join(G1_DIR, "g1_29dof.xml"))
    root = tree.getroot()
    root.set("model", "g1_29dof_inspire_fixed")

    # --- fix pelvis: remove the freejoint ---
    pelvis = root.find(".//body[@name='pelvis']")
    fj = pelvis.find("joint[@name='floating_base_joint']")
    pelvis.remove(fj)

    # --- stereo head cameras (for Quest binocular view) ---
    # torso_link carries the head mesh (center ~(0.007, 0, 0.375), front face
    # ~x=0.075). Cameras look along body +X (forward), image-up = body +Z,
    # pitched 25 deg downward toward the workspace.
    # xyaxes: cam x = -Y_body (image right), cam y = up vector after pitch.
    torso = root.find(".//body[@name='torso_link']")
    ipd = 0.064  # interpupillary distance
    pitch_up = "0.4226 0 0.9063"  # sin(25deg), 0, cos(25deg)
    for name, y in (("head_left_eye", ipd / 2), ("head_right_eye", -ipd / 2)):
        ET.SubElement(torso, "camera", {
            "name": name,
            "pos": f"0.075 {y} 0.41",
            "xyaxes": f"0 -1 0 {pitch_up}",
            "fovy": "70",
        })

    # offscreen framebuffer large enough for 640x480 eye renders
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", {"offwidth": "1280", "offheight": "720"})

    asset = root.find("asset")

    for side, parent_name, mount_quat in (
        ("left", "left_wrist_yaw_link", f"{SQ2} 0 0 {SQ2}"),
        ("right", "right_wrist_yaw_link", f"0 {SQ2} -{SQ2} 0"),
    ):
        meshes, hand_body = load_hand(side)
        for m in meshes:
            asset.append(m)

        wrist = root.find(f".//body[@name='{parent_name}']")
        # remove rubber hand visual geom
        for geom in list(wrist.findall("geom")):
            if geom.get("mesh", "").endswith("_rubber_hand"):
                wrist.remove(geom)

        hand_body.set("pos", "0.054 0 0")
        hand_body.set("quat", mount_quat)
        wrist.append(hand_body)

    # --- actuators: 12 position actuators in inspire DDS order ---
    actuator = root.find("actuator")
    for name, lo, hi in DDS_ORDER:
        ET.SubElement(actuator, "position", {
            "name": name.replace("_joint", ""),
            "joint": name,
            "kp": "1.0",
            "kv": "0.05",
            "ctrlrange": f"{lo} {hi}",
            "forcerange": "-1 1",
        })

    # --- equality couplings for mimic joints ---
    equality = ET.SubElement(root, "equality")
    for dep, drv, mult in MIMICS:
        ET.SubElement(equality, "joint", {
            "joint1": dep,
            "joint2": drv,
            "polycoef": f"0 {mult} 0 0 0",
        })

    ET.indent(tree, space="  ")
    out = os.path.join(G1_DIR, "g1_29dof_inspire_fixed.xml")
    tree.write(out)
    print(f"wrote {out}")

    scene = f"""<mujoco model=\"g1_29dof_inspire_fixed scene\">
  <include file=\"g1_29dof_inspire_fixed.xml\"/>

  <statistic center=\"0 0 0.5\" extent=\"2.0\"/>

  <visual>
    <headlight diffuse=\"0.6 0.6 0.6\" ambient=\"0.3 0.3 0.3\" specular=\"0 0 0\"/>
    <rgba haze=\"0.15 0.25 0.35 1\"/>
    <global azimuth=\"-130\" elevation=\"-20\"/>
  </visual>

  <asset>
    <texture type=\"skybox\" builtin=\"gradient\" rgb1=\"0.3 0.5 0.7\" rgb2=\"0 0 0\" width=\"512\" height=\"3072\"/>
    <texture type=\"2d\" name=\"groundplane\" builtin=\"checker\" mark=\"edge\" rgb1=\"0.2 0.3 0.4\" rgb2=\"0.1 0.2 0.3\"
      markrgb=\"0.8 0.8 0.8\" width=\"300\" height=\"300\"/>
    <material name=\"groundplane\" texture=\"groundplane\" texuniform=\"true\" texrepeat=\"5 5\" reflectance=\"0.2\"/>
  </asset>

  <worldbody>
    <light pos=\"0 0 1.5\" dir=\"0 0 -1\" directional=\"true\"/>
    <geom name=\"floor\" size=\"0 0 0.05\" type=\"plane\" material=\"groundplane\"/>
  </worldbody>
</mujoco>
"""
    scene_out = os.path.join(G1_DIR, "scene_29dof_inspire_fixed.xml")
    with open(scene_out, "w") as f:
        f.write(scene)
    print(f"wrote {scene_out}")


if __name__ == "__main__":
    main()
