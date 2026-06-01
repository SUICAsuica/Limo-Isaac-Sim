"""Replace unstable imported Limo wheel visuals with fixed visual wheels.

The imported USD wheel links can drift when physics is disabled after the
articulation has been simulated.  This hides those links and adds simple
visual-only wheels under base_link so the robot stays visually intact while
the kinematic /cmd_vel controller moves the root.
"""

import omni.usd
from pxr import Gf, Sdf, UsdGeom


BASE_LINK_PATH = "/limo_xacro/base_footprint/base_link"
VISUAL_ROOT = f"{BASE_LINK_PATH}/uninavid_fixed_wheels"

ORIGINAL_WHEEL_LINKS = [
    "/limo_xacro/front_left_wheel_link",
    "/limo_xacro/front_right_wheel_link",
    "/limo_xacro/rear_left_wheel_link",
    "/limo_xacro/rear_right_wheel_link",
]

WHEELS = [
    ("front_left", 0.10, 0.065, -0.10),
    ("front_right", 0.10, -0.065, -0.10),
    ("rear_left", -0.10, 0.065, -0.10),
    ("rear_right", -0.10, -0.065, -0.10),
]


def get_stage():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No stage is open.")
    return stage


def make_mat(stage, path, color):
    mat = UsdGeom.Scope.Define(stage, path)
    prim = mat.GetPrim()
    prim.CreateAttribute("displayColor", Sdf.ValueTypeNames.Color3fArray).Set([color])
    return prim


def main():
    stage = get_stage()

    for path in ORIGINAL_WHEEL_LINKS:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            UsdGeom.Imageable(prim).MakeInvisible()

    if stage.GetPrimAtPath(VISUAL_ROOT).IsValid():
        stage.RemovePrim(VISUAL_ROOT)
    UsdGeom.Xform.Define(stage, VISUAL_ROOT)

    for name, x, y, z in WHEELS:
        path = f"{VISUAL_ROOT}/{name}_wheel"
        wheel = UsdGeom.Cylinder.Define(stage, path)
        wheel.CreateRadiusAttr(0.045)
        wheel.CreateHeightAttr(0.045)
        wheel.CreateAxisAttr("Y")
        wheel.CreateDisplayColorAttr([Gf.Vec3f(0.02, 0.02, 0.02)])
        xform = UsdGeom.Xformable(wheel.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))

        hub_path = f"{VISUAL_ROOT}/{name}_hub"
        hub = UsdGeom.Cylinder.Define(stage, hub_path)
        hub.CreateRadiusAttr(0.018)
        hub.CreateHeightAttr(0.048)
        hub.CreateAxisAttr("Y")
        hub.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.55, 0.55)])
        hub_xform = UsdGeom.Xformable(hub.GetPrim())
        hub_xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))

    print(f"Hidden original wheel links: {ORIGINAL_WHEEL_LINKS}")
    print(f"Added fixed visual wheels under: {VISUAL_ROOT}")


main()
