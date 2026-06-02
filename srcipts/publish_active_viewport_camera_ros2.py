"""Publish the Limo front camera as ROS 2 camera topics.

The primary path uses Isaac Sim's Replicator ROS 2 writers, matching the
Isaac Sim camera publishing tests. A CameraHelper OmniGraph fallback is kept
for older sessions where the writer registry is unavailable.
"""

import omni.graph.core as og
import omni.usd
from isaacsim.core.utils import extensions

try:
    import usdrt
except ImportError:
    usdrt = None


GRAPH_PATH = "/World/UniNaVidActiveViewportCameraROS2Graph"
CAMERA_NAME = "uninavid_front_camera"
RGB_TOPIC = "camera/color/image_raw"
INFO_TOPIC = "camera/color/camera_info"
FRAME_ID = "limo_uninavid_camera"
VIEWPORT_ID = 1
CAMERA_RESOLUTION = (640, 480)
PUBLISH_FREQUENCY_HZ = 20


def get_stage():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No stage is open.")
    return stage


def find_camera_path(stage):
    for prim in stage.Traverse():
        if prim.GetName() == CAMERA_NAME:
            return prim.GetPath().pathString
    raise RuntimeError(f"Camera prim not found: {CAMERA_NAME}")


def target_path(path):
    if usdrt is not None:
        return [usdrt.Sdf.Path(path)]
    return path


def set_gate_step(render_product_path, sensor_type, step_size):
    try:
        import omni.syntheticdata
        import omni.syntheticdata._syntheticdata as sd

        if sensor_type == "rgb":
            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
            gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                rv + "IsaacSimulationGate", render_product_path
            )
        else:
            gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                "PostProcessDispatch" + "IsaacSimulationGate", render_product_path
            )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
    except Exception as exc:
        print(f"Could not set camera publish gate for {sensor_type}: {exc}")


def publish_with_replicator(camera_path):
    extensions.enable_extension("isaacsim.ros2.bridge")
    extensions.enable_extension("omni.replicator.core")

    import omni.replicator.core as rep
    import omni.syntheticdata._syntheticdata as sd
    from isaacsim.ros2.bridge import read_camera_info

    rv_rgb = __import__("omni.syntheticdata").syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.Rgb.name
    )
    render_product = rep.create.render_product(camera_path, CAMERA_RESOLUTION)
    render_product_path = getattr(render_product, "path", render_product)
    if not isinstance(render_product_path, str):
        render_product_path = str(render_product_path)

    rgb_writer = rep.writers.get(rv_rgb + "ROS2PublishImage")
    rgb_writer.initialize(
        frameId=FRAME_ID,
        nodeNamespace="",
        queueSize=1,
        topicName=RGB_TOPIC,
    )
    rgb_writer.attach([render_product_path])

    info_writer = rep.writers.get("ROS2PublishCameraInfo")
    camera_info, _ = read_camera_info(render_product_path=render_product_path)
    info_writer.initialize(
        frameId=FRAME_ID,
        nodeNamespace="",
        queueSize=1,
        topicName=INFO_TOPIC,
        width=camera_info.width,
        height=camera_info.height,
        projectionType=camera_info.distortion_model,
        k=camera_info.k.reshape([1, 9]),
        r=camera_info.r.reshape([1, 9]),
        p=camera_info.p.reshape([1, 12]),
        physicalDistortionModel=camera_info.distortion_model,
        physicalDistortionCoefficients=camera_info.d,
    )
    info_writer.attach([render_product_path])

    step_size = max(1, int(60 / PUBLISH_FREQUENCY_HZ))
    set_gate_step(render_product_path, "rgb", step_size)
    set_gate_step(render_product_path, "info", step_size)

    print(f"Publishing Limo front camera with Replicator: {camera_path}")
    print(f"Render product: {render_product_path}")
    print(f"RGB topic: /{RGB_TOPIC}")
    print(f"Camera info topic: /{INFO_TOPIC}")


def publish_with_camera_helper(camera_path):
    extensions.enable_extension("isaacsim.core.nodes")
    extensions.enable_extension("isaacsim.ros2.bridge")

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
                ("CreateViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                ("GetRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                ("SetCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
                ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("CreateViewport.inputs:viewportId", VIEWPORT_ID),
                ("SetCamera.inputs:cameraPrim", target_path(camera_path)),
                ("RGBPublish.inputs:topicName", RGB_TOPIC),
                ("RGBPublish.inputs:type", "rgb"),
                ("RGBPublish.inputs:frameId", FRAME_ID),
                ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
                ("CameraInfoPublish.inputs:topicName", INFO_TOPIC),
                ("CameraInfoPublish.inputs:frameId", FRAME_ID),
                ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateViewport.inputs:execIn"),
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

    print(f"Publishing Limo front camera: {camera_path}")
    print(f"RGB topic: /{RGB_TOPIC}")
    print(f"Camera info topic: /{INFO_TOPIC}")
    print("Press Play to publish camera images.")


def main():
    stage = get_stage()
    camera_path = find_camera_path(stage)
    try:
        publish_with_replicator(camera_path)
    except Exception as exc:
        print(f"Replicator ROS 2 camera publish failed, falling back to CameraHelper graph: {exc}")
        publish_with_camera_helper(camera_path)


main()
