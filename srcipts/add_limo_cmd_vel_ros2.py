"""Wire ROS 2 /cmd_vel to the Limo wheel joints in Isaac Sim.

Run inside Isaac Sim: Window > Script Editor > open this file > Run.
Then press Play and publish /cmd_vel from a terminal.
"""

import omni.graph.core as og
import omni.usd
from isaacsim.core.utils import extensions
from pxr import UsdGeom, UsdPhysics

try:
    import usdrt
except ImportError:
    usdrt = None


# Leave empty to auto-detect the Limo articulation root.
LIMO_ARTICULATION_PATH = ""

GRAPH_PATH = "/World/UniNaVidCmdVelROS2Graph"
CMD_VEL_TOPIC = "/cmd_vel"

# Limo URDF values are close to these. Adjust only if motion scale is clearly wrong.
WHEEL_RADIUS_M = 0.045
WHEEL_DISTANCE_M = 0.172

# Order must match DifferentialController velocityCommand: left, right.
FRONT_WHEEL_JOINTS = ["front_left_wheel", "front_right_wheel"]
REAR_WHEEL_JOINTS = ["rear_left_wheel", "rear_right_wheel"]
ALL_WHEEL_JOINTS = set(FRONT_WHEEL_JOINTS + REAR_WHEEL_JOINTS)


def get_stage():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No stage is open.")
    return stage


def is_articulation_root(prim):
    if not prim or not prim.IsValid():
        return False
    try:
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return True
    except Exception:
        pass
    api_names = [str(name).lower() for name in prim.GetAppliedSchemas()]
    return any("articulationroot" in name for name in api_names)


def auto_find_limo_articulation(stage):
    articulation_candidates = []
    xform_candidates = []
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        lower_path = path.lower()
        if "limo" not in lower_path:
            continue
        if is_articulation_root(prim):
            articulation_candidates.append(path)
        elif prim.IsA(UsdGeom.Xform):
            xform_candidates.append(path)

    candidates = articulation_candidates or xform_candidates
    if not candidates:
        raise RuntimeError(
            "Could not find a Limo prim. Set LIMO_ARTICULATION_PATH at the top of this script."
        )

    wheel_joint_paths = []
    for prim in stage.Traverse():
        if prim.GetName() in ALL_WHEEL_JOINTS:
            wheel_joint_paths.append(prim.GetPath().pathString)
    if len(wheel_joint_paths) < len(ALL_WHEEL_JOINTS):
        print(f"Warning: found only these Limo wheel joints: {wheel_joint_paths}")

    candidates.sort(key=len)
    return candidates[0]


def target_path(path):
    if usdrt is not None:
        return [usdrt.Sdf.Path(path)]
    return path


def create_cmd_vel_graph(robot_path):
    extensions.enable_extension("isaacsim.core.nodes")
    extensions.enable_extension("isaacsim.ros2.bridge")
    extensions.enable_extension("isaacsim.robot.wheeled_robots")

    stage = get_stage()
    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        stage.RemovePrim(GRAPH_PATH)

    keys = og.Controller.Keys
    graph, _, _, _ = og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("subscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("breakLinVel", "omni.graph.nodes.BreakVector3"),
                ("breakAngVel", "omni.graph.nodes.BreakVector3"),
                ("FrontDiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("FrontArtController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("RearDiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("RearArtController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("subscribeTwist.inputs:topicName", CMD_VEL_TOPIC),
                ("FrontDiffController.inputs:wheelRadius", WHEEL_RADIUS_M),
                ("FrontDiffController.inputs:wheelDistance", WHEEL_DISTANCE_M),
                ("FrontDiffController.inputs:maxLinearSpeed", 0.35),
                ("FrontDiffController.inputs:maxAngularSpeed", 1.5),
                ("FrontDiffController.inputs:maxWheelSpeed", 18.0),
                ("FrontArtController.inputs:robotPath", robot_path),
                ("FrontArtController.inputs:jointNames", FRONT_WHEEL_JOINTS),
                ("RearDiffController.inputs:wheelRadius", WHEEL_RADIUS_M),
                ("RearDiffController.inputs:wheelDistance", WHEEL_DISTANCE_M),
                ("RearDiffController.inputs:maxLinearSpeed", 0.35),
                ("RearDiffController.inputs:maxAngularSpeed", 1.5),
                ("RearDiffController.inputs:maxWheelSpeed", 18.0),
                ("RearArtController.inputs:robotPath", robot_path),
                ("RearArtController.inputs:jointNames", REAR_WHEEL_JOINTS),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "subscribeTwist.inputs:execIn"),
                ("subscribeTwist.outputs:linearVelocity", "breakLinVel.inputs:tuple"),
                ("subscribeTwist.outputs:angularVelocity", "breakAngVel.inputs:tuple"),
                ("subscribeTwist.outputs:execOut", "FrontDiffController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "FrontArtController.inputs:execIn"),
                ("breakLinVel.outputs:x", "FrontDiffController.inputs:linearVelocity"),
                ("breakAngVel.outputs:z", "FrontDiffController.inputs:angularVelocity"),
                (
                    "FrontDiffController.outputs:velocityCommand",
                    "FrontArtController.inputs:velocityCommand",
                ),
                ("subscribeTwist.outputs:execOut", "RearDiffController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "RearArtController.inputs:execIn"),
                ("breakLinVel.outputs:x", "RearDiffController.inputs:linearVelocity"),
                ("breakAngVel.outputs:z", "RearDiffController.inputs:angularVelocity"),
                (
                    "RearDiffController.outputs:velocityCommand",
                    "RearArtController.inputs:velocityCommand",
                ),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)


def main():
    stage = get_stage()
    robot_path = LIMO_ARTICULATION_PATH or auto_find_limo_articulation(stage)
    create_cmd_vel_graph(robot_path)

    print(f"Added ROS 2 cmd_vel graph: {GRAPH_PATH}")
    print(f"Robot/articulation target: {robot_path}")
    print(f"Subscribed topic: {CMD_VEL_TOPIC}")
    print(f"Front joints: {FRONT_WHEEL_JOINTS}")
    print(f"Rear joints: {REAR_WHEEL_JOINTS}")
    print("Press Play, then run ./srcipts/run_ros2_cmd_vel_test.sh --linear-x 0.18 --duration 3")


main()
