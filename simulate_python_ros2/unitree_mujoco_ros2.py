import threading
import time
from threading import Thread

import mujoco
import mujoco.viewer
import rclpy

try:
    from . import config
    from .unitree_ros2_bridge import ElasticBand, UnitreeRos2Bridge
except ImportError:
    import config
    from unitree_ros2_bridge import ElasticBand, UnitreeRos2Bridge


def main():
    if config.ROBOT != "g1":
        raise ValueError("simulate_python_ros2 only supports config.ROBOT == 'g1'")

    locker = threading.Lock()
    mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
    mj_data = mujoco.MjData(mj_model)
    mj_model.opt.timestep = config.SIMULATE_DT

    if config.ENABLE_ELASTIC_BAND:
        elastic_band = ElasticBand()
        band_attached_link = mj_model.body("torso_link").id
        viewer = mujoco.viewer.launch_passive(
            mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
        )
    else:
        elastic_band = None
        band_attached_link = None
        viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

    time.sleep(0.2)

    def simulation_thread():
        rclpy.init()
        bridge = UnitreeRos2Bridge(
            mj_model,
            mj_data,
            config.ROBOT_SCENE,
            use_joystick=config.USE_JOYSTICK,
            joystick_type=config.JOYSTICK_TYPE,
            joystick_device=config.JOYSTICK_DEVICE,
        )

        if config.PRINT_SCENE_INFORMATION:
            print("ROS2-native G1 simulator running with scene:", config.ROBOT_SCENE)

        try:
            while viewer.is_running():
                step_start = time.perf_counter()
                rclpy.spin_once(bridge, timeout_sec=0.0)

                with locker:
                    bridge.apply_latest_command()
                    if config.ENABLE_ELASTIC_BAND and elastic_band is not None and elastic_band.enable:
                        mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                            mj_data.qpos[:3], mj_data.qvel[:3]
                        )
                    mujoco.mj_step(mj_model, mj_data)
                    bridge.publish_step_messages()

                time_until_next_step = mj_model.opt.timestep - (time.perf_counter() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
        finally:
            bridge.destroy_node()
            rclpy.shutdown()

    def physics_viewer_thread():
        while viewer.is_running():
            with locker:
                viewer.sync()
            time.sleep(config.VIEWER_DT)

    viewer_thread = Thread(target=physics_viewer_thread)
    sim_thread = Thread(target=simulation_thread)

    viewer_thread.start()
    sim_thread.start()


if __name__ == "__main__":
    main()
