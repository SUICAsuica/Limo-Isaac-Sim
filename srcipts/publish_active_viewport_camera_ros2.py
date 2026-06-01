"""Publish the active Isaac Sim viewport as ROS 2 camera topics.

Run after add_limo_camera_ros2.py and after confirming the viewport shows the
Limo front camera. This avoids creating a new viewport/render product.
"""

import omni.graph.core as og
import omni.usd
from isaacsim.core.utils import extensions
from omni.kit.viewport.utility import get_active_viewport


GRAPH_PATH = "/World/UniNaVidActiveViewportCameraROS2Graph"
RGB_TOPIC = "/camera/color/image_raw"
INFO_TOPIC = "/camera/color/camera_info"
FRAME_ID = "limo_uninavid_camera"


def get_stage():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No stage is open.")
    return stage


def main():
    extensions.enable_extension("isaacsim.ros2.bridge")

    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No active viewport. Click the Isaac Sim viewport once and run again.")

    render_product_path = viewport.get_render_product_path()
    if not render_product_path:
        raise RuntimeError("Active viewport has no render product path. Press Play once, stop, then run again.")

    stage = get_stage()
    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        stage.RemovePrim(GRAPH_PATH)

    keys = og.Controller.Keys
    graph, _, _, _ = og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("RGBPublish.inputs:topicName", RGB_TOPIC),
                ("RGBPublish.inputs:type", "rgb"),
                ("RGBPublish.inputs:frameId", FRAME_ID),
                ("RGBPublish.inputs:renderProductPath", render_product_path),
                ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
                ("CameraInfoPublish.inputs:topicName", INFO_TOPIC),
                ("CameraInfoPublish.inputs:frameId", FRAME_ID),
                ("CameraInfoPublish.inputs:renderProductPath", render_product_path),
                ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RGBPublish.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CameraInfoPublish.inputs:execIn"),
                ("Context.outputs:context", "RGBPublish.inputs:context"),
                ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)

    print(f"Publishing active viewport render product: {render_product_path}")
    print(f"RGB topic: {RGB_TOPIC}")
    print(f"Camera info topic: {INFO_TOPIC}")
    print("Press Play to publish camera images.")


main()
