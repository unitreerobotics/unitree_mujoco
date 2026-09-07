ROBOT = "g1"  # Only G1 is supported in the ROS2-native simulator.
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene_29dof.xml"  # Default to the G1 29DoF scene.

USE_JOYSTICK = 1  # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox"  # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0  # Joystick number

PRINT_SCENE_INFORMATION = True  # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = True  # Virtual spring band, used for lifting h1

SIMULATE_DT = 0.005  # Need to be larger than the runtime of viewer.sync()
VIEWER_DT = 0.02  # 50 fps for viewer
