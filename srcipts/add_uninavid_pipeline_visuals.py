"""Add visible Uni-NaVid pipeline graph overlays inside Isaac Sim."""

import omni.graph.core as og
import omni.usd
from isaacsim.core.utils import extensions
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, Sdf, UsdGeom, UsdShade


ROOT = "/World/UniNaVidPipelineGraph"
SCREEN_GRAPH_PATH = "/World/UniNaVidPipelineScreenTextGraph"
CONNECTION_GRAPH_PATH = "/World/UniNaVidRosConnectionActionGraph"


def stage():
    return omni.usd.get_context().get_stage()


def ensure_xform(path):
    return UsdGeom.Xform.Define(stage(), path)


def make_material(path, color):
    st = stage()
    mat = UsdShade.Material.Define(st, path)
    shader = UsdShade.Shader.Define(st, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.45)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def bind_material(prim, mat):
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def cube(path, loc, scale, mat):
    prim = UsdGeom.Cube.Define(stage(), path)
    prim.CreateSizeAttr(1.0)
    xform = UsdGeom.Xformable(prim.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*loc))
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))
    bind_material(prim.GetPrim(), mat)
    return prim


def cylinder_between(path, start, end, radius, mat):
    sx, sy, sz = start
    ex, ey, ez = end
    mx = (sx + ex) * 0.5
    my = (sy + ey) * 0.5
    mz = (sz + ez) * 0.5
    dx = ex - sx
    dy = ey - sy
    dz = ez - sz
    length = (dx * dx + dy * dy + dz * dz) ** 0.5
    yaw = __import__("math").degrees(__import__("math").atan2(dy, dx))

    prim = UsdGeom.Cylinder.Define(stage(), path)
    prim.CreateRadiusAttr(radius)
    prim.CreateHeightAttr(length)
    prim.CreateAxisAttr("X")
    xform = UsdGeom.Xformable(prim.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(mx, my, mz))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, yaw))
    bind_material(prim.GetPrim(), mat)
    return prim


def add_3d_graph():
    st = stage()
    if st is None:
        raise RuntimeError("No stage is open.")
    if st.GetPrimAtPath(ROOT).IsValid():
        st.RemovePrim(ROOT)
    ensure_xform(ROOT)

    mats_root = f"{ROOT}/Materials"
    mats = {
        "panel": make_material(f"{mats_root}/panel", (0.08, 0.10, 0.13)),
        "input": make_material(f"{mats_root}/input", (0.92, 0.94, 0.98)),
        "model": make_material(f"{mats_root}/model", (0.20, 0.55, 0.90)),
        "ros": make_material(f"{mats_root}/ros", (0.20, 0.72, 0.36)),
        "isaac": make_material(f"{mats_root}/isaac", (0.95, 0.58, 0.20)),
        "camera": make_material(f"{mats_root}/camera", (0.55, 0.35, 0.88)),
        "arrow": make_material(f"{mats_root}/arrow", (0.95, 0.95, 0.95)),
    }

    # A physical graph board placed to the left-front side of the robot.
    cube(f"{ROOT}/black_back_board", (-2.9, -2.85, 1.55), (2.8, 0.05, 1.25), mats["panel"])
    nodes = [
        ("user_instruction", (-4.05, -2.78, 2.05), mats["input"]),
        ("ros2_bridge", (-3.25, -2.78, 2.05), mats["ros"]),
        ("uninavid_server", (-2.45, -2.78, 2.05), mats["model"]),
        ("cmd_vel_topic", (-1.65, -2.78, 2.05), mats["ros"]),
        ("isaac_limo", (-1.65, -2.78, 1.35), mats["isaac"]),
        ("front_camera_topic", (-2.85, -2.78, 1.35), mats["camera"]),
    ]
    for name, loc, mat in nodes:
        cube(f"{ROOT}/node_{name}", loc, (0.28, 0.035, 0.18), mat)

    cylinder_between(f"{ROOT}/arrow_instruction_to_bridge", (-3.77, -2.78, 2.05), (-3.53, -2.78, 2.05), 0.018, mats["arrow"])
    cylinder_between(f"{ROOT}/arrow_bridge_to_server", (-2.97, -2.78, 2.05), (-2.73, -2.78, 2.05), 0.018, mats["arrow"])
    cylinder_between(f"{ROOT}/arrow_server_to_cmd", (-2.17, -2.78, 2.05), (-1.93, -2.78, 2.05), 0.018, mats["arrow"])
    cylinder_between(f"{ROOT}/arrow_cmd_to_limo", (-1.65, -2.78, 1.87), (-1.65, -2.78, 1.53), 0.018, mats["arrow"])
    cylinder_between(f"{ROOT}/arrow_limo_to_camera", (-1.93, -2.78, 1.35), (-2.57, -2.78, 1.35), 0.018, mats["arrow"])
    cylinder_between(f"{ROOT}/arrow_camera_to_bridge", (-2.85, -2.78, 1.53), (-3.25, -2.78, 1.87), 0.018, mats["arrow"])

    cam_path = f"{ROOT}/overview_camera"
    cam = UsdGeom.Camera.Define(st, cam_path)
    cam_xf = UsdGeom.Xformable(cam.GetPrim())
    cam_xf.ClearXformOpOrder()
    cam_xf.AddTranslateOp().Set(Gf.Vec3d(0.8, -6.8, 4.2))
    cam_xf.AddRotateXYZOp().Set(Gf.Vec3f(60.0, 0.0, 8.0))
    cam.CreateFocalLengthAttr(18.0)
    return cam_path


def add_screen_text_graph():
    extensions.enable_extension("omni.graph.visualization.nodes")
    st = stage()
    if st.GetPrimAtPath(SCREEN_GRAPH_PATH).IsValid():
        st.RemovePrim(SCREEN_GRAPH_PATH)

    text = (
        "Uni-NaVid pipeline\\n"
        "1. Limo camera publishes /camera/color/image_raw\\n"
        "2. ROS2 bridge sends image + instruction to Uni-NaVid server\\n"
        "3. Uni-NaVid returns actions: forward / left / right / stop\\n"
        "4. Bridge publishes /cmd_vel\\n"
        "5. Isaac Sim kinematic controller moves /limo_xacro"
    )
    keys = og.Controller.Keys
    graph, _, _, _ = og.Controller.edit(
        {"graph_path": SCREEN_GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("DrawText", "omni.graph.visualization.nodes.DrawScreenSpaceText"),
            ],
            keys.SET_VALUES: [
                ("DrawText.inputs:position", [2.5, 5.0]),
                ("DrawText.inputs:text", text),
                ("DrawText.inputs:size", 20.0),
                ("DrawText.inputs:boxWidth", 620),
                ("DrawText.inputs:textColor", [0.05, 0.95, 1.25, 1.0]),
            ],
            keys.CONNECT: [("OnTick.outputs:tick", "DrawText.inputs:execIn")],
        },
    )
    og.Controller.evaluate_sync(graph)


def add_connection_action_graph():
    """Create an OmniGraph-readable connection map for Graph Editor.

    The Uni-NaVid bridge and HTTP model server are external Python processes,
    so they cannot appear as real Isaac Sim compute nodes without rewriting the
    bridge as an OmniGraph node. This graph intentionally exposes the ROS/data
    flow as named Action Graph nodes that can be inspected in Isaac Sim.
    """
    extensions.enable_extension("omni.graph.ui_nodes")
    extensions.enable_extension("isaacsim.ros2.bridge")

    st = stage()
    if st.GetPrimAtPath(CONNECTION_GRAPH_PATH).IsValid():
        st.RemovePrim(CONNECTION_GRAPH_PATH)

    keys = og.Controller.Keys
    graph, _, _, _ = og.Controller.edit(
        {"graph_path": CONNECTION_GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("A01_OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("A02_ROS2_Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("A03_CameraTopic__camera_color_image_raw", "omni.graph.nodes.ConstantString"),
                ("A04_ExternalPythonBridge__uninavid_limo_ros2_bridge", "omni.graph.ui_nodes.PrintText"),
                ("A05_HTTP_Predict__127_0_0_1_8088", "omni.graph.nodes.ConstantString"),
                ("A06_UniNaVidServer__tools_uninavid_server_py", "omni.graph.ui_nodes.PrintText"),
                ("A07_Actions__forward_left_right_stop", "omni.graph.nodes.ConstantString"),
                ("A08_ROS2Publish__cmd_vel", "omni.graph.ui_nodes.PrintText"),
                ("A09_ROS2SubscribeTwist__cmd_vel", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("A10_BreakLinearVelocity", "omni.graph.nodes.BreakVector3"),
                ("A11_BreakAngularVelocity", "omni.graph.nodes.BreakVector3"),
                ("A12_PythonKinematicController__limo_goal_controller", "omni.graph.ui_nodes.PrintText"),
                ("A13_LimoRoot___limo_xacro", "omni.graph.nodes.ConstantString"),
                ("A14_FrontCameraPublisher__ReplicatorROS2Writer", "omni.graph.ui_nodes.PrintText"),
            ],
            keys.SET_VALUES: [
                ("A03_CameraTopic__camera_color_image_raw.inputs:value", "/camera/color/image_raw"),
                ("A04_ExternalPythonBridge__uninavid_limo_ros2_bridge.inputs:logLevel", "Warning"),
                ("A05_HTTP_Predict__127_0_0_1_8088.inputs:value", "POST http://127.0.0.1:8088/predict"),
                ("A06_UniNaVidServer__tools_uninavid_server_py.inputs:logLevel", "Warning"),
                ("A07_Actions__forward_left_right_stop.inputs:value", "actions: forward / left / right / stop"),
                ("A08_ROS2Publish__cmd_vel.inputs:logLevel", "Warning"),
                ("A09_ROS2SubscribeTwist__cmd_vel.inputs:topicName", "/cmd_vel"),
                ("A12_PythonKinematicController__limo_goal_controller.inputs:logLevel", "Warning"),
                ("A13_LimoRoot___limo_xacro.inputs:value", "/limo_xacro"),
                ("A14_FrontCameraPublisher__ReplicatorROS2Writer.inputs:logLevel", "Warning"),
            ],
            keys.CONNECT: [
                ("A01_OnPlaybackTick.outputs:tick", "A09_ROS2SubscribeTwist__cmd_vel.inputs:execIn"),
                ("A09_ROS2SubscribeTwist__cmd_vel.outputs:linearVelocity", "A10_BreakLinearVelocity.inputs:tuple"),
                ("A09_ROS2SubscribeTwist__cmd_vel.outputs:angularVelocity", "A11_BreakAngularVelocity.inputs:tuple"),
                ("A09_ROS2SubscribeTwist__cmd_vel.outputs:execOut", "A12_PythonKinematicController__limo_goal_controller.inputs:execIn"),
                ("A03_CameraTopic__camera_color_image_raw.inputs:value", "A04_ExternalPythonBridge__uninavid_limo_ros2_bridge.inputs:text"),
                ("A05_HTTP_Predict__127_0_0_1_8088.inputs:value", "A06_UniNaVidServer__tools_uninavid_server_py.inputs:text"),
                ("A07_Actions__forward_left_right_stop.inputs:value", "A08_ROS2Publish__cmd_vel.inputs:text"),
                ("A13_LimoRoot___limo_xacro.inputs:value", "A14_FrontCameraPublisher__ReplicatorROS2Writer.inputs:text"),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)
    print(f"Added OmniGraph ROS connection map: {CONNECTION_GRAPH_PATH}")


def main():
    camera_path = add_3d_graph()
    add_screen_text_graph()
    add_connection_action_graph()
    viewport = get_active_viewport()
    if viewport is not None:
        viewport.camera_path = camera_path
        print(f"Active viewport camera set to pipeline overview: {camera_path}")
    print(f"Added visible Uni-NaVid pipeline graph: {ROOT}")
    print(f"Added screen-space pipeline explanation: {SCREEN_GRAPH_PATH}")


main()
