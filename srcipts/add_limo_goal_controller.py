"""Drive Limo in Isaac Sim from ROS 2 /goal and /cmd_vel.

This controller is intentionally kinematic.  The imported Limo USD on this
machine exposes wheel joints, but the articulation drive does not reliably
move the base.  Moving the robot root directly keeps the camera attached and
gives a dependable target for Uni-NaVid and ROS tests.
"""

import math
import time

import omni.kit.app
import omni.usd
from pxr import Gf, Usd, UsdGeom

import rclpy
from geometry_msgs.msg import PoseStamped, Twist


ROBOT_PATH = "/limo_xacro"
GOAL_TOPIC = "/goal"
CMD_VEL_TOPIC = "/cmd_vel"
MAX_LINEAR_SPEED = 0.35
MAX_ANGULAR_SPEED = 1.2
GOAL_TOLERANCE_M = 0.08
CMD_TIMEOUT_SEC = 0.35
LOG_PATH = "/tmp/limo_goal_controller.log"


def log(message):
    line = f"{time.strftime('%F %T')} {message}"
    print(line)
    with open(LOG_PATH, "a", encoding="utf-8") as fp:
        fp.write(line + "\n")


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def angle_wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quat(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def get_or_add_ops(xformable):
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
    return translate_op, orient_op


class LimoGoalController:
    def __init__(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("No stage is open.")

        prim = stage.GetPrimAtPath(ROBOT_PATH)
        if not prim.IsValid():
            raise RuntimeError(f"Robot prim not found: {ROBOT_PATH}")

        self.xformable = UsdGeom.Xformable(prim)
        self.translate_op, self.orient_op = get_or_add_ops(self.xformable)
        world = self.xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pos = world.ExtractTranslation()
        self.x = float(pos[0])
        self.y = float(pos[1])
        self.z = float(pos[2])
        self.yaw = 0.0

        self.goal = None
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.last_cmd_time = 0.0
        self.last_update_time = time.monotonic()

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node("limo_goal_controller")
        self.node.create_subscription(PoseStamped, GOAL_TOPIC, self.on_goal, 10)
        self.node.create_subscription(Twist, CMD_VEL_TOPIC, self.on_cmd_vel, 10)

        app = omni.kit.app.get_app()
        self.update_sub = app.get_update_event_stream().create_subscription_to_pop(self.on_update)
        log(f"enabled: robot={ROBOT_PATH}, goal_topic={GOAL_TOPIC}, cmd_topic={CMD_VEL_TOPIC}")

    def on_goal(self, msg):
        q = msg.pose.orientation
        use_yaw = abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) > 1e-6
        goal_yaw = yaw_from_quat(q) if use_yaw else None
        frame_id = msg.header.frame_id.strip().lower()
        goal_x = float(msg.pose.position.x)
        goal_y = float(msg.pose.position.y)
        if frame_id in ("base_link", "limo", "limo_xacro", "relative"):
            rel_x = goal_x
            rel_y = goal_y
            goal_x = self.x + math.cos(self.yaw) * rel_x - math.sin(self.yaw) * rel_y
            goal_y = self.y + math.sin(self.yaw) * rel_x + math.cos(self.yaw) * rel_y
            if goal_yaw is not None:
                goal_yaw = angle_wrap(self.yaw + goal_yaw)
        self.goal = (goal_x, goal_y, goal_yaw)
        log(f"new goal: x={self.goal[0]:.3f}, y={self.goal[1]:.3f}, yaw={self.goal[2]}")

    def on_cmd_vel(self, msg):
        self.cmd_linear = float(msg.linear.x)
        self.cmd_angular = float(msg.angular.z)
        self.last_cmd_time = time.monotonic()
        self.goal = None

    def velocity_from_goal(self):
        if self.goal is None:
            return 0.0, 0.0

        gx, gy, goal_yaw = self.goal
        dx = gx - self.x
        dy = gy - self.y
        dist = math.hypot(dx, dy)

        if dist <= GOAL_TOLERANCE_M:
            if goal_yaw is None or abs(angle_wrap(goal_yaw - self.yaw)) < 0.08:
                log(f"goal reached: x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}")
                self.goal = None
                return 0.0, 0.0
            heading_error = angle_wrap(goal_yaw - self.yaw)
            return 0.0, clamp(2.0 * heading_error, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)

        target_heading = math.atan2(dy, dx)
        heading_error = angle_wrap(target_heading - self.yaw)
        angular = clamp(2.2 * heading_error, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
        linear = clamp(0.8 * dist, 0.0, MAX_LINEAR_SPEED)
        if abs(heading_error) > 0.9:
            linear = 0.0
        elif abs(heading_error) > 0.45:
            linear *= 0.35
        return linear, angular

    def on_update(self, _event):
        now = time.monotonic()
        dt = clamp(now - self.last_update_time, 0.0, 0.05)
        self.last_update_time = now

        rclpy.spin_once(self.node, timeout_sec=0.0)

        if self.goal is not None:
            linear, angular = self.velocity_from_goal()
        elif now - self.last_cmd_time <= CMD_TIMEOUT_SEC:
            linear = clamp(self.cmd_linear, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
            angular = clamp(self.cmd_angular, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
        else:
            linear = 0.0
            angular = 0.0

        if abs(linear) < 1e-6 and abs(angular) < 1e-6:
            return

        self.yaw = angle_wrap(self.yaw + angular * dt)
        self.x += math.cos(self.yaw) * linear * dt
        self.y += math.sin(self.yaw) * linear * dt

        half = self.yaw * 0.5
        self.translate_op.Set(Gf.Vec3d(self.x, self.y, self.z))
        self.orient_op.Set(Gf.Quatd(math.cos(half), Gf.Vec3d(0.0, 0.0, math.sin(half))))


def main():
    controller = LimoGoalController()
    globals()["_limo_goal_controller"] = controller


main()
