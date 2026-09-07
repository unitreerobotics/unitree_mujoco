import unittest
from pathlib import Path
from xml.etree import ElementTree

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "unitree_robots/g1/g1_29dof_inspire_fixed.xml"
SCENE = ROOT / "unitree_robots/g1/scene_29dof_inspire_fixed.xml"
ACTUATORS = [
    "R_pinky_proximal", "R_ring_proximal", "R_middle_proximal",
    "R_index_proximal", "R_thumb_proximal_pitch", "R_thumb_proximal_yaw",
    "L_pinky_proximal", "L_ring_proximal", "L_middle_proximal",
    "L_index_proximal", "L_thumb_proximal_pitch", "L_thumb_proximal_yaw",
]
RANGES = np.array(
    [[0.0, 1.7]] * 4 + [[0.0, 0.5], [-0.1, 1.3]]
    + [[0.0, 1.7]] * 4 + [[0.0, 0.5], [-0.1, 1.3]]
)


def names(model, obj, count):
    return [mujoco.mj_id2name(model, obj, i) for i in range(count)]


class G1InspireModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_path(str(SCENE))
        cls.hand_actuators = np.arange(29, 41)
        cls.hand_qadr = np.array([
            cls.model.jnt_qposadr[cls.model.actuator_trnid[i, 0]]
            for i in cls.hand_actuators
        ])

    def test_structure_and_mapping(self):
        mujoco.MjModel.from_xml_path(str(MODEL))
        model = self.model
        joint_names = names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
        actuator_names = names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)
        hand_joints = [n for n in joint_names if n.startswith(("L_", "R_"))]

        self.assertEqual((model.nq, model.nv, model.nu), (53, 53, 41))
        self.assertEqual(len(hand_joints), 24)
        self.assertEqual(model.neq, 12)
        self.assertEqual(actuator_names[29:41], ACTUATORS)
        np.testing.assert_array_equal(model.actuator_ctrlrange[29:41], RANGES)
        self.assertEqual(len(joint_names), len(set(joint_names)))
        self.assertEqual(len(actuator_names), len(set(actuator_names)))

        for hand, wrist in (("L_hand_base_link", "left_wrist_yaw_link"),
                            ("R_hand_base_link", "right_wrist_yaw_link")):
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, hand)
            self.assertEqual(
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                  model.body_parentid[bid]),
                wrist,
            )

        root = ElementTree.parse(MODEL).getroot()
        meshdir = MODEL.parent / root.find("compiler").get("meshdir")
        self.assertTrue(all((meshdir / mesh.get("file")).is_file()
                            for mesh in root.findall("./asset/mesh")))

    def test_full_range_and_equalities(self):
        model = self.model
        data = mujoco.MjData(model)

        for target in (RANGES[:, 1], RANGES[:, 0]):
            data.ctrl[self.hand_actuators] = target
            for _ in range(3000):
                mujoco.mj_step(model, data)
            np.testing.assert_allclose(data.qpos[self.hand_qadr[[5, 11]]],
                                       target[[5, 11]], atol=0.015)

        data.ctrl[self.hand_actuators] = RANGES[:, 1] - 0.5 * np.diff(RANGES, axis=1)[:, 0]
        for _ in range(2000):
            mujoco.mj_step(model, data)
        for i in range(model.neq):
            joint1, joint2 = model.eq_obj1id[i], model.eq_obj2id[i]
            q1 = data.qpos[model.jnt_qposadr[joint1]]
            q2 = data.qpos[model.jnt_qposadr[joint2]]
            self.assertAlmostEqual(q1, model.eq_data[i, 1] * q2, delta=0.002)

    def test_finger_object_collision_remains_enabled(self):
        model = self.model
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                "L_index_intermediate")
        marker = next(i for i in np.flatnonzero(model.geom_bodyid == body)
                      if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_SPHERE)

        spec = mujoco.MjSpec.from_file(str(SCENE))
        spec.worldbody.add_geom(name="collision_probe",
                                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                                size=[0.01, 0, 0], pos=data.geom_xpos[marker])
        probe_model = spec.compile()
        probe_data = mujoco.MjData(probe_model)
        mujoco.mj_forward(probe_model, probe_data)
        probe = mujoco.mj_name2id(probe_model, mujoco.mjtObj.mjOBJ_GEOM,
                                 "collision_probe")
        self.assertTrue(any(probe in (c.geom1, c.geom2)
                            for c in probe_data.contact))


if __name__ == "__main__":
    unittest.main()
