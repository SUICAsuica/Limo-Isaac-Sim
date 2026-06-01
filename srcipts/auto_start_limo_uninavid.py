"""Auto-start Limo + Uni-NaVid support inside Isaac Sim.

Launch with:
  ./isaac-sim.sh --exec /home/novel/Limo-Isaac-Sim/srcipts/auto_start_limo_uninavid.py
"""

import asyncio
import traceback

import omni.timeline
import omni.usd


LIMO_STAGE_PATH = "/home/novel/Limo-Isaac-Sim/limo_description/urdf/limo_base/limo_base.usd"
SCRIPT_DIR = "/home/novel/Limo-Isaac-Sim/srcipts"
LOG_PATH = "/tmp/limo_uninavid_isaac_setup.log"


def log(message):
    print(message)
    with open(LOG_PATH, "a", encoding="utf-8") as fp:
        fp.write(message + "\n")


def run_script(path):
    log(f"Running: {path}")
    with open(path, "r", encoding="utf-8") as fp:
        code = compile(fp.read(), path, "exec")
    namespace = {"__file__": path, "__name__": "__main__"}
    exec(code, namespace)


async def setup():
    try:
        log("Opening Limo stage...")
        ok, error = await omni.usd.get_context().open_stage_async(LIMO_STAGE_PATH)
        if not ok:
            raise RuntimeError(f"Failed to open stage: {error}")

        # Give the stage and viewport a few frames to settle.
        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        run_script(f"{SCRIPT_DIR}/add_uninavid_room_env.py")
        run_script(f"{SCRIPT_DIR}/stabilize_limo_physics.py")
        log("Skipping wheel articulation /cmd_vel graph; using kinematic Limo controller.")
        run_script(f"{SCRIPT_DIR}/add_limo_goal_controller.py")
        run_script(f"{SCRIPT_DIR}/fix_limo_visual_wheels.py")
        run_script(f"{SCRIPT_DIR}/add_limo_camera_ros2.py")

        try:
            run_script(f"{SCRIPT_DIR}/publish_active_viewport_camera_ros2.py")
        except Exception:
            log("Camera ROS publish setup failed; continuing with /cmd_vel graph.")
            log(traceback.format_exc())

        omni.timeline.get_timeline_interface().play()
        log("Isaac setup complete. Timeline is playing.")
    except Exception:
        log("Isaac setup failed:")
        log(traceback.format_exc())


asyncio.ensure_future(setup())
