"""Auto-start Limo + Uni-NaVid support inside Isaac Sim.

Launch with:
  ./isaac-sim.sh --exec /home/novel/Limo-Isaac-Sim/srcipts/auto_start_limo_uninavid.py
"""

import asyncio
import traceback

import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom


LIMO_STAGE_PATH = "/home/novel/Limo-Isaac-Sim/limo_description/urdf/limo_base/limo_base.usd"
SCRIPT_DIR = "/home/novel/Limo-Isaac-Sim/srcipts"
LOG_PATH = "/tmp/limo_uninavid_isaac_setup.log"
LIMO_ROOT_PATH = "/limo_xacro"


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


def reset_limo_pose():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No stage is open after loading Limo USD.")

    prim = stage.GetPrimAtPath(LIMO_ROOT_PATH)
    if not prim.IsValid():
        candidates = [
            item.GetPath().pathString
            for item in stage.Traverse()
            if item.IsA(UsdGeom.Xform) and "limo" in item.GetPath().pathString.lower()
        ]
        raise RuntimeError(f"Limo root was not found at {LIMO_ROOT_PATH}. Candidates: {candidates[:12]}")

    xformable = UsdGeom.Xformable(prim)
    translate_op = None
    orient_op = None
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
        elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            orient_op = op
    if translate_op is None:
        translate_op = xformable.AddTranslateOp()
    if orient_op is None:
        orient_op = xformable.AddOrientOp()

    translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.0), Usd.TimeCode.Default())
    orient_op.Set(Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0)), Usd.TimeCode.Default())
    log(f"Reset Limo pose: {LIMO_ROOT_PATH} at origin.")


async def setup():
    try:
        log("Opening Limo stage...")
        ok, error = await omni.usd.get_context().open_stage_async(LIMO_STAGE_PATH)
        if not ok:
            raise RuntimeError(f"Failed to open stage: {error}")

        # Give the stage and viewport a few frames to settle.
        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        reset_limo_pose()
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
