"""Add a front camera to Limo and publish it as /camera/color/image_raw.

Run this inside Isaac Sim: Window > Script Editor > open this file > Run.
If the camera points backward or sideways, edit CAMERA_ROTATE_XYZ below and run again.
"""

import omni.graph.core as og
import omni.kit.commands
import omni.usd
from isaacsim.core.utils import extensions
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, UsdGeom

try:
    import usdrt
except ImportError:
    usdrt = None


# Leave empty to auto-detect Limo's moving base link.
LIMO_PRIM_PATH = ""

CAMERA_NAME = "uninavid_front_camera"
# Limo base_link frame is usually x-forward, y-left, z-up.
# Keep the camera outside the body to avoid seeing only the robot shell.
CAMERA_LOCAL_TRANSLATE = (0.55, 0.0, 0.32)
CAMERA_ROTATE_XYZ = (90.0, 0.0, -90.0)
CAMERA_RESOLUTION = (640, 480)
VIEWPORT_ID = 1

GRAPH_PATH = "/World/UniNaVidCameraROS2Graph"
RGB_TOPIC = "/camera/color/image_raw"
INFO_TOPIC = "/camera/color/camera_info"
FRAME_ID = "limo_uninavid_camera"

# Keep this False in Script Editor. Creating render products from here can freeze
# some Isaac Sim sessions. Use publish_active_viewport_camera_ros2.py instead.
ENABLE_ROS_PUBLISH_GRAPH = False


def get_stage():
    st = omni.usd.get_context().get_stage()
    if st is None:
        raise RuntimeError("No stage is open.")
    return st


def auto_find_limo_prim(st):
    base_link_candidates = []
    fallback_candidates = []
    for prim in st.Traverse():
        path = prim.GetPath().pathString
        name = prim.GetName().lower()
        lower_path = path.lower()
        if not prim.IsA(UsdGeom.Xform):
            continue
        if "limo" not in lower_path and "limo" not in name:
            continue
        if name in {"base_link", "base"} or lower_path.endswith("/base_link"):
            base_link_candidates.append(path)
        else:
            fallback_candidates.append(path)
    candidates = base_link_candidates or fallback_candidates
    if not candidates:
        raise RuntimeError("Could not find a Limo prim. Set LIMO_PRIM_PATH at the top of this script.")
    candidates.sort(key=lambda item: (len(item.split("/")), len(item)))
    return candidates[0]


def create_camera(st, parent_path):
    for prim in list(st.Traverse()):
        if prim.GetName() == CAMERA_NAME:
            st.RemovePrim(prim.GetPath())

    camera_path = f"{parent_path}/{CAMERA_NAME}"

    cam = UsdGeom.Camera.Define(st, camera_path)
    xform = UsdGeom.XformCommonAPI(cam)
    xform.SetTranslate(Gf.Vec3d(*CAMERA_LOCAL_TRANSLATE))
    xform.SetRotate(CAMERA_ROTATE_XYZ, UsdGeom.XformCommonAPI.RotationOrderXYZ)

    cam.GetHorizontalApertureAttr().Set(21.0)
    cam.GetVerticalApertureAttr().Set(16.0)
    cam.GetProjectionAttr().Set("perspective")
    cam.GetFocalLengthAttr().Set(18.0)
    cam.GetFocusDistanceAttr().Set(5.0)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.02, 1000.0))
    return camera_path


def create_ros2_camera_graph(camera_path):
    extensions.enable_extension("isaacsim.core.nodes")
    extensions.enable_extension("isaacsim.ros2.bridge")

    st = get_stage()
    if st.GetPrimAtPath(GRAPH_PATH).IsValid():
        st.RemovePrim(GRAPH_PATH)

    keys = og.Controller.Keys
    graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": GRAPH_PATH,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("CreateViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                ("GetRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                ("SetCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
                ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("CreateViewport.inputs:viewportId", VIEWPORT_ID),
                (
                    "SetCamera.inputs:cameraPrim",
                    [usdrt.Sdf.Path(camera_path)] if usdrt is not None else camera_path,
                ),
                ("RGBPublish.inputs:topicName", RGB_TOPIC),
                ("RGBPublish.inputs:type", "rgb"),
                ("RGBPublish.inputs:frameId", FRAME_ID),
                ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
                ("CameraInfoPublish.inputs:topicName", INFO_TOPIC),
                ("CameraInfoPublish.inputs:frameId", FRAME_ID),
                ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "CreateViewport.inputs:execIn"),
                ("CreateViewport.outputs:execOut", "GetRenderProduct.inputs:execIn"),
                ("CreateViewport.outputs:viewport", "GetRenderProduct.inputs:viewport"),
                ("GetRenderProduct.outputs:execOut", "SetCamera.inputs:execIn"),
                ("GetRenderProduct.outputs:renderProductPath", "SetCamera.inputs:renderProductPath"),
                ("SetCamera.outputs:execOut", "RGBPublish.inputs:execIn"),
                ("SetCamera.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                ("GetRenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
                ("GetRenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
                ("Context.outputs:context", "RGBPublish.inputs:context"),
                ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)


def set_active_viewport_camera(camera_path):
    viewport = get_active_viewport()
    if viewport is None:
        print("No active viewport found. Select the camera manually from the viewport camera menu.")
        return
    viewport.camera_path = camera_path
    print(f"Active viewport camera set to: {camera_path}")


def main():
    st = get_stage()
    parent_path = LIMO_PRIM_PATH or auto_find_limo_prim(st)
    camera_path = create_camera(st, parent_path)
    set_active_viewport_camera(camera_path)
    if ENABLE_ROS_PUBLISH_GRAPH:
        create_ros2_camera_graph(camera_path)

    print(f"Added camera: {camera_path}")
    if ENABLE_ROS_PUBLISH_GRAPH:
        print(f"Publishing RGB topic: {RGB_TOPIC}")
        print(f"Publishing camera info topic: {INFO_TOPIC}")
        print("Press Play in Isaac Sim to start publishing.")
    else:
        print("ROS publish graph was not created. Run publish_active_viewport_camera_ros2.py after confirming the view.")


main()
