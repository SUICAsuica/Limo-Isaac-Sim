"""Stabilize Limo physics for Isaac Sim.

Run inside Isaac Sim: Window > Script Editor > open this file > Run.
Use this after placing/importing Limo and before pressing Play.
"""

import omni.usd
from pxr import Gf, PhysxSchema, PhysicsSchemaTools, Usd, UsdGeom, UsdPhysics


LIMO_ARTICULATION_PATH = ""

GROUND_PATH = "/World/UniNaVidRigidGround"
GROUND_SIZE = 20.0
GROUND_Z = 0.0

WHEEL_LINK_NAMES = {
    "front_left_wheel_link",
    "front_right_wheel_link",
    "rear_left_wheel_link",
    "rear_right_wheel_link",
}
WHEEL_JOINT_NAMES = {
    "front_left_wheel",
    "front_right_wheel",
    "rear_left_wheel",
    "rear_right_wheel",
}

WHEEL_DRIVE_STIFFNESS = 0.0
WHEEL_DRIVE_DAMPING = 2000.0
WHEEL_DRIVE_MAX_FORCE = 100000.0
KINEMATIC_ROOT_PATH = "/limo_xacro"


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


def auto_find_limo(stage):
    articulation_candidates = []
    xform_candidates = []
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if "limo" not in path.lower():
            continue
        if is_articulation_root(prim):
            articulation_candidates.append(path)
        elif prim.IsA(UsdGeom.Xform):
            xform_candidates.append(path)
    candidates = articulation_candidates or xform_candidates
    if not candidates:
        raise RuntimeError("Could not find Limo. Set LIMO_ARTICULATION_PATH at the top.")

    wheel_joint_paths = []
    for prim in stage.Traverse():
        if prim.GetName() in WHEEL_JOINT_NAMES:
            wheel_joint_paths.append(prim.GetPath().pathString)
    if len(wheel_joint_paths) < len(WHEEL_JOINT_NAMES):
        print(f"Warning: found only these Limo wheel joints: {wheel_joint_paths}")

    candidates.sort(key=len)
    return candidates[0]


def ensure_physics_scene(stage):
    scene_path = "/World/PhysicsScene"
    scene = UsdPhysics.Scene.Get(stage, scene_path)
    if not scene:
        scene = UsdPhysics.Scene.Define(stage, scene_path)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateEnableCCDAttr().Set(True)
    physx_scene.CreateSolverTypeAttr().Set("TGS")
    physx_scene.GetTimeStepsPerSecondAttr().Set(120)
    return scene_path


def ensure_ground(stage):
    if stage.GetPrimAtPath(GROUND_PATH).IsValid():
        stage.RemovePrim(GROUND_PATH)
    PhysicsSchemaTools.addGroundPlane(
        stage,
        GROUND_PATH,
        "Z",
        GROUND_SIZE,
        Gf.Vec3f(0.0, 0.0, GROUND_Z),
        Gf.Vec3f(0.45, 0.45, 0.45),
    )
    return GROUND_PATH


def lift_if_below_ground(stage, robot_path):
    prim = stage.GetPrimAtPath(robot_path)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bbox = cache.ComputeWorldBound(prim).ComputeAlignedBox()
    min_z = bbox.GetMin()[2]
    lift = GROUND_Z - min_z + 0.005
    if lift <= 0.0:
        return 0.0

    xform = UsdGeom.Xformable(prim)
    translate_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    if translate_op is None:
        translate_op = xform.AddTranslateOp()

    current = translate_op.Get() or Gf.Vec3d(0.0, 0.0, 0.0)
    translate_op.Set(Gf.Vec3d(current[0], current[1], current[2] + lift))
    return lift


def tune_limo_physics(stage, robot_path):
    robot_prim = stage.GetPrimAtPath(robot_path)
    physx_art = PhysxSchema.PhysxArticulationAPI.Apply(robot_prim)
    physx_art.CreateSolverPositionIterationCountAttr().Set(16)
    physx_art.CreateSolverVelocityIterationCountAttr().Set(4)

    tuned = []
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        is_robot_child = path.startswith(robot_path)
        is_limo_wheel = prim.GetName() in WHEEL_LINK_NAMES and "limo" in path.lower()
        if not (is_robot_child or is_limo_wheel):
            continue
        if prim.GetName() in WHEEL_LINK_NAMES:
            body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            body_api.CreateEnableCCDAttr().Set(True)
            tuned.append(path)
    return tuned


def tune_wheel_drives(stage):
    tuned = []
    for prim in stage.Traverse():
        if prim.GetName() not in WHEEL_JOINT_NAMES:
            continue
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateStiffnessAttr().Set(WHEEL_DRIVE_STIFFNESS)
        drive.CreateDampingAttr().Set(WHEEL_DRIVE_DAMPING)
        drive.CreateMaxForceAttr().Set(WHEEL_DRIVE_MAX_FORCE)
        tuned.append(prim.GetPath().pathString)
    return tuned


def disable_limo_rigid_bodies(stage):
    disabled = []
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if not path.startswith(KINEMATIC_ROOT_PATH):
            continue
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        body_api = UsdPhysics.RigidBodyAPI(prim)
        body_api.CreateRigidBodyEnabledAttr().Set(False)
        body_api.CreateKinematicEnabledAttr().Set(True)
        disabled.append(path)
    return disabled


def disable_limo_physics_joints(stage):
    disabled = []
    for prim in list(stage.Traverse()):
        path = prim.GetPath().pathString
        if not path.startswith(KINEMATIC_ROOT_PATH):
            continue
        if "/joints/" not in path and prim.GetName() != "root_joint":
            continue
        if prim.IsA(UsdPhysics.Joint):
            prim.SetActive(False)
            disabled.append(path)
    return disabled


def main():
    stage = get_stage()
    robot_path = LIMO_ARTICULATION_PATH or auto_find_limo(stage)

    scene_path = ensure_physics_scene(stage)
    ground_path = ensure_ground(stage)
    lift = lift_if_below_ground(stage, robot_path)
    tuned = tune_limo_physics(stage, robot_path)
    drive_tuned = tune_wheel_drives(stage)
    disabled_bodies = disable_limo_rigid_bodies(stage)
    disabled_joints = disable_limo_physics_joints(stage)

    print(f"Physics scene: {scene_path}")
    print(f"Rigid ground: {ground_path} at z={GROUND_Z}")
    print(f"Limo target: {robot_path}")
    print(f"Lifted Limo by: {lift:.4f} m")
    print(f"Tuned wheel links: {tuned}")
    print(f"Tuned wheel drives: {drive_tuned}")
    print(f"Disabled Limo rigid bodies for kinematic control: {disabled_bodies}")
    print(f"Disabled Limo physics joints for kinematic control: {disabled_joints}")
    print("Now press Play and test /cmd_vel again.")


main()
