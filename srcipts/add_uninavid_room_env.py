from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade
import omni.usd

try:
    from isaacsim.core.utils import stage as stage_utils
    from isaacsim.storage.native import get_assets_root_path
except Exception:
    stage_utils = None
    get_assets_root_path = None


ROOT = "/World/UniNaVidRoom"
REAL_ENV_ROOT = "/World/RealisticEnvironment"
REAL_ENV_CANDIDATES = [
    "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
    "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    "/Isaac/Environments/Simple_Room/simple_room.usd",
]
IMPORTED_PROPS_ROOT = "/World/UniNaVidImportedProps"
IMPORTED_PROPS = [
    ("pallet_a", "/Isaac/Environments/Simple_Warehouse/Props/SM_PaletteA_01.usd", (-2.6, 2.3, 0.0), (0, 0, 18), (1.0, 1.0, 1.0)),
    ("cardbox_a", "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxA_01_414.usd", (-2.0, 2.25, 0.25), (0, 0, -12), (1.0, 1.0, 1.0)),
    ("cardbox_d", "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_04_1847.usd", (-2.9, 1.55, 0.25), (0, 0, 35), (1.0, 1.0, 1.0)),
    ("traffic_cone", "/Isaac/Environments/Simple_Warehouse/Props/S_TrafficCone.usd", (0.7, 2.7, 0.0), (0, 0, 0), (1.0, 1.0, 1.0)),
    ("wet_floor_sign", "/Isaac/Environments/Simple_Warehouse/Props/S_WetFloorSign.usd", (1.1, -1.2, 0.0), (0, 0, -25), (1.0, 1.0, 1.0)),
    ("book_03", "/Isaac/Environments/Office/Props/SM_Book_03.usd", (2.05, 2.15, 0.72), (0, 0, 12), (1.0, 1.0, 1.0)),
    ("book_open", "/Isaac/Environments/Office/Props/SM_BookOpen_01.usd", (2.35, 2.22, 0.72), (0, 0, -15), (1.0, 1.0, 1.0)),
    ("briefcase", "/Isaac/Environments/Office/Props/SM_Briefcase.usd", (2.95, 1.0, 0.0), (0, 0, -35), (1.0, 1.0, 1.0)),
    ("extinguisher", "/Isaac/Environments/Office/Props/SM_Extinguisher.usd", (-1.1, -2.3, 0.0), (0, 0, 0), (1.0, 1.0, 1.0)),
    ("mustard_bottle", "/Isaac/Props/YCB/Axis_Aligned_Physics/006_mustard_bottle.usd", (1.75, 2.35, 0.72), (0, 0, 0), (1.0, 1.0, 1.0)),
    ("cracker_box", "/Isaac/Props/YCB/Axis_Aligned_Physics/003_cracker_box.usd", (2.55, 2.0, 0.72), (0, 0, 20), (1.0, 1.0, 1.0)),
]


def stage():
    return omni.usd.get_context().get_stage()


def ensure_xform(path):
    return UsdGeom.Xform.Define(stage(), path)


def set_xform(path, translate=(0.0, 0.0, 0.0), rotate_xyz=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)):
    prim = stage().GetPrimAtPath(path)
    if not prim.IsValid():
        return
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotate_xyz))
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))


def add_realistic_isaac_environment():
    if get_assets_root_path is None or stage_utils is None:
        print("Isaac asset helpers are not available; using generated fallback room.")
        return False

    assets_root_path = get_assets_root_path()
    if not assets_root_path:
        print("Isaac asset root was not found; using generated fallback room.")
        return False

    st = stage()
    if st.GetPrimAtPath(REAL_ENV_ROOT).IsValid():
        st.RemovePrim(REAL_ENV_ROOT)

    for rel_path in REAL_ENV_CANDIDATES:
        usd_path = assets_root_path + rel_path
        try:
            stage_utils.add_reference_to_stage(usd_path=usd_path, prim_path=REAL_ENV_ROOT)
            set_xform(REAL_ENV_ROOT, translate=(0.0, 0.0, 0.0), rotate_xyz=(0.0, 0.0, 0.0))
            print(f"Added realistic Isaac Sim environment: {usd_path}")
            return True
        except Exception as exc:
            print(f"Could not load Isaac environment {usd_path}: {exc}")

    print("No Isaac environment candidate loaded; using generated fallback room.")
    return False


def add_imported_props():
    if get_assets_root_path is None or stage_utils is None:
        print("Isaac asset helpers are not available; skipping imported props.")
        return 0

    assets_root_path = get_assets_root_path()
    if not assets_root_path:
        print("Isaac asset root was not found; skipping imported props.")
        return 0

    st = stage()
    if st.GetPrimAtPath(IMPORTED_PROPS_ROOT).IsValid():
        st.RemovePrim(IMPORTED_PROPS_ROOT)
    ensure_xform(IMPORTED_PROPS_ROOT)

    loaded = 0
    for name, rel_path, translate, rotate_xyz, scale in IMPORTED_PROPS:
        prim_path = f"{IMPORTED_PROPS_ROOT}/{name}"
        try:
            stage_utils.add_reference_to_stage(usd_path=assets_root_path + rel_path, prim_path=prim_path)
            set_xform(prim_path, translate=translate, rotate_xyz=rotate_xyz, scale=scale)
            loaded += 1
        except Exception as exc:
            print(f"Could not import prop {rel_path}: {exc}")

    print(f"Imported Isaac Sim furniture/props: {loaded}/{len(IMPORTED_PROPS)} under {IMPORTED_PROPS_ROOT}")
    return loaded


def make_material(path, color, roughness=0.65):
    st = stage()
    mat = UsdShade.Material.Define(st, path)
    shader = UsdShade.Shader.Define(st, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def bind_material(prim, mat):
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def cube(path, loc, scale, mat, collision=True):
    prim = UsdGeom.Cube.Define(stage(), path)
    prim.CreateSizeAttr(1.0)
    xform = UsdGeom.Xformable(prim.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*loc))
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))
    bind_material(prim.GetPrim(), mat)
    if collision:
        UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
    return prim


def cylinder(path, loc, radius, height, mat, collision=True):
    prim = UsdGeom.Cylinder.Define(stage(), path)
    prim.CreateRadiusAttr(radius)
    prim.CreateHeightAttr(height)
    prim.CreateAxisAttr("Z")
    xform = UsdGeom.Xformable(prim.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*loc))
    bind_material(prim.GetPrim(), mat)
    if collision:
        UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
    return prim


def add_chair(path, x, y, yaw_deg, mats):
    root = ensure_xform(path)
    xf = UsdGeom.Xformable(root.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    xf.AddRotateZOp().Set(yaw_deg)

    cube(f"{path}/seat", (0, 0, 0.42), (0.45, 0.45, 0.08), mats["chair"])
    cube(f"{path}/back", (0, 0.22, 0.78), (0.45, 0.07, 0.42), mats["chair"])
    for i, lx in enumerate((-0.18, 0.18)):
        for j, ly in enumerate((-0.18, 0.18)):
            cube(f"{path}/leg_{i}_{j}", (lx, ly, 0.21), (0.045, 0.045, 0.42), mats["dark"])


def add_room():
    st = stage()
    if st is None:
        raise RuntimeError("No stage is open. Open or create an Isaac Sim stage first.")

    ensure_xform(ROOT)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)

    if add_realistic_isaac_environment():
        # Keep a few semantic visual targets in the realistic scene so natural
        # language commands such as "brown table" and "green chair" have clear
        # objects even when the referenced warehouse asset lacks furniture.
        ensure_xform(ROOT)
        mats_root = f"{ROOT}/Materials"
        mats = {
            "chair": make_material(f"{mats_root}/chair_green", (0.10, 0.45, 0.25)),
            "table": make_material(f"{mats_root}/table_wood", (0.55, 0.33, 0.16)),
            "target": make_material(f"{mats_root}/target_red", (0.85, 0.08, 0.05)),
            "dark": make_material(f"{mats_root}/dark_metal", (0.06, 0.06, 0.06)),
        }
        add_chair(f"{ROOT}/target_chair", 2.0, 1.2, -20, mats)
        cube(f"{ROOT}/brown_table/top", (2.2, 2.2, 0.58), (0.9, 0.55, 0.08), mats["table"])
        for i, lx in enumerate((1.8, 2.6)):
            for j, ly in enumerate((1.9, 2.5)):
                cube(f"{ROOT}/brown_table/leg_{i}_{j}", (lx, ly, 0.29), (0.055, 0.055, 0.58), mats["dark"])
        cylinder(f"{ROOT}/red_goal_marker", (2.0, 1.75, 0.35), 0.18, 0.7, mats["target"])
        add_imported_props()
        print("Added Uni-NaVid semantic props at /World/UniNaVidRoom")
        print("Suggested instruction: find the brown table, move toward it, go under it, and stop.")
        return

    mats_root = f"{ROOT}/Materials"
    mats = {
        "floor": make_material(f"{mats_root}/floor", (0.45, 0.47, 0.42)),
        "wall": make_material(f"{mats_root}/wall", (0.72, 0.74, 0.70)),
        "chair": make_material(f"{mats_root}/chair_green", (0.10, 0.45, 0.25)),
        "table": make_material(f"{mats_root}/table_wood", (0.55, 0.33, 0.16)),
        "target": make_material(f"{mats_root}/target_red", (0.85, 0.08, 0.05)),
        "blue": make_material(f"{mats_root}/blue_marker", (0.05, 0.20, 0.85)),
        "dark": make_material(f"{mats_root}/dark_metal", (0.06, 0.06, 0.06)),
        "plant": make_material(f"{mats_root}/plant", (0.08, 0.35, 0.08)),
    }

    # Room shell. Limo can start near the origin and see a clear corridor.
    cube(f"{ROOT}/floor", (0, 0, -0.025), (8.0, 7.0, 0.05), mats["floor"])
    cube(f"{ROOT}/wall_back", (0, 3.5, 1.0), (8.0, 0.12, 2.0), mats["wall"])
    cube(f"{ROOT}/wall_front_low", (0, -3.5, 0.55), (8.0, 0.12, 1.1), mats["wall"])
    cube(f"{ROOT}/wall_left", (-4.0, 0, 1.0), (0.12, 7.0, 2.0), mats["wall"])
    cube(f"{ROOT}/wall_right", (4.0, 0, 1.0), (0.12, 7.0, 2.0), mats["wall"])

    # Navigation landmarks and obstacles.
    add_chair(f"{ROOT}/target_chair", 1.6, 2.1, -25, mats)
    add_chair(f"{ROOT}/side_chair_left", -2.3, 1.4, 30, mats)
    add_chair(f"{ROOT}/side_chair_right", 2.7, -1.2, -60, mats)

    cube(f"{ROOT}/table/top", (-1.2, 2.1, 0.58), (0.8, 0.5, 0.08), mats["table"])
    for i, lx in enumerate((-1.55, -0.85)):
        for j, ly in enumerate((1.85, 2.35)):
            cube(f"{ROOT}/table/leg_{i}_{j}", (lx, ly, 0.29), (0.055, 0.055, 0.58), mats["dark"])

    cube(f"{ROOT}/low_box_obstacle", (-0.15, 1.0, 0.18), (0.8, 0.35, 0.36), mats["blue"])
    cylinder(f"{ROOT}/red_goal_marker", (1.6, 2.75, 0.35), 0.18, 0.7, mats["target"])
    cylinder(f"{ROOT}/plant_pot", (-2.8, -2.4, 0.22), 0.22, 0.44, mats["table"])
    cylinder(f"{ROOT}/plant_top", (-2.8, -2.4, 0.68), 0.34, 0.42, mats["plant"], collision=False)
    add_imported_props()

    # Lighting and overview camera.
    dome = UsdLux.DomeLight.Define(st, f"{ROOT}/dome_light")
    dome.CreateIntensityAttr(350.0)
    light = UsdLux.RectLight.Define(st, f"{ROOT}/ceiling_light")
    light.CreateIntensityAttr(550.0)
    light.CreateWidthAttr(5.0)
    light.CreateHeightAttr(4.0)
    UsdGeom.Xformable(light.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, 3.2))

    cam = UsdGeom.Camera.Define(st, f"{ROOT}/overview_camera")
    cam_xf = UsdGeom.Xformable(cam.GetPrim())
    cam_xf.ClearXformOpOrder()
    cam_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, -6.4, 4.4))
    cam_xf.AddRotateXYZOp().Set(Gf.Vec3f(58.0, 0.0, 0.0))
    cam.CreateFocalLengthAttr(20.0)

    print("Added Uni-NaVid room environment at /World/UniNaVidRoom")
    print("Suggested instruction: move to the green chair near the red marker, then stop.")


add_room()
