# G1 variants generated from Unitree's real descriptions

Built from Unitree's URDFs (`unitree_ros/robots`), not written by hand. Each `scene_*_fixed_base.xml`
includes its model, sets `timestep=0.002` (light finger links) and welds the pelvis to the world;
the same scene without `_fixed_base` leaves the pelvis free.

**All variants share the same body: `g1_29dof.xml`** (geometry, masses, joint classes, actuators, IMU).
Only the hands change: each hand sub-tree is cut from Unitree's real URDF and attached to the
`wrist_yaw_link` at the pose that URDF gives. The `g1_29dof.xml` rubber hand is removed and the wrist
inertia is replaced by the URDF's wrist-without-rubber-hand one (0.085 kg instead of 0.255 kg), so the
rubber hand's mass is not counted on top of the new hand.

| model | hand source (unitree_ros/robots) | body motors | hand motors |
|---|---|---|---|
| `g1_29dof_dex3.xml` | `g1_description/g1_29dof_with_hand_rev_1_0.urdf` (Dex3; thumb_1 range from Unitree's `.xml` of the same name, -0.724 rad, the URDF says -0.611) | 29 | 14 (7 per hand; order left thumb, middle, index / right thumb, index, middle, as in HandCmd_) |
| `g1_29dof_dex1.xml` | `g1_description/g1_29dof_mode_15_with_dex1_1.urdf` | 29 | 4 (2 prismatic fingers per hand) |
| `g1_29dof_inspire_ftp.xml` | `g1_description/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf` | 29 | 12 (6 per hand; the rest are mimic joints) |
| `g1_29dof_inspire_dfx.xml` | `g1_description/g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf` (xr_teleoperate also uses this geometry for the DFX; thumb bend limited to 0..0.5 rad like there) | 29 | 12 (6 per hand; the rest are mimic joints) |
| `g1_29dof_brainco.xml` | `g1_with_brainco_hand/g1_29dof_mode_15_brainco_hand.urdf` | 29 | 12 (6 per hand; the rest are mimic joints) |

Actuator order: body in `LowCmd_` order (`left_hip_pitch` ... `right_wrist_yaw`), then left hand, then
right hand. Sensors: jointpos for all actuators, then jointvel, then jointactuatorfrc, then IMU sensors
(the layout the bridge indexes).

Hand joint dynamics (damping 0.01, armature 0.001) are our choice, not Unitree's. Mimic joints (URDF
`<mimic>`) are `<equality><joint>` constraints. Tip joints with lower == upper limit were made fixed.
Contact pairs that overlap at the zero pose (thumb vs hand base) are excluded.
