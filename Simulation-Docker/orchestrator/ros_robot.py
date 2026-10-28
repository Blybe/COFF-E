"""ROS 2 side of the orchestrator: the mobile base (simulated or real).

One rclpy node per target, each in its own ROS 2 domain, so the simulation and
the real Origin One can be commanded at the same time without their topics
mixing up:

  <cmd_vel topic>  publishes: drives the robot
  <odom topic>     subscribes: where the robot actually is (the check for
                   every drive step)
  <clock topic>    subscribes: Gazebo time, so "drive for 60 s" means 60 s of
                   simulation time (the sim runs at ~0.5x real time)
  navigation       the simulation is driven with a Nav2 NavigateToPose action
                   goal; the real robot gets a "move through poses" behavior
                   for its autopilot and is then tracked through its global
                   pose (see targets.py)

Taking control of the real Origin One is part of the drive: it only moves when
its control mode is USER and the Twist messages arrive on /robot/cmd_vel_user
(origin_msgs/srv/SetControlMode). The simulation implements the same interface,
so SIM_DRIVE_MODE=user exercises exactly the same code path as the real robot.

The node is spun in a background thread; the engine and the dashboard call the
small methods below from their own threads.
"""

import collections
import json
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

import commands

ODOM_TIMEOUT = 5.0        # s: odometry older than this counts as "no robot"
CLOCK_TIMEOUT = 3.0       # s: /clock older than this counts as "no sim time"
SERVICE_TIMEOUT = 5.0     # s to wait for a control-mode / behavior service
MOVE_THROUGH_POSES = 10   # knowledge_base_msgs/msg/Behavior

# The Avular control-mode services, at their standard names. Used when a raw
# override points a drive step at the USER topic on a target that normally
# drives on the plain cmd_vel topic.
USER_MODE_SERVICE = "/robot/cmd_vel_controller/set_control_mode"
USER_MODE_RESET_SERVICE = "/robot/cmd_vel_controller/reset_control_mode"


def _user_topic(topic):
    """True for the Twist topics that only work in control mode USER."""
    return bool(topic) and str(topic).endswith("_user")


class NavGoal:
    """Wrapper around a NavigateToPose goal, for polling."""

    kind = "action"

    def __init__(self, future):
        self.future = future           # goal handle future from send_goal_async
        self.handle = None             # set once the goal response arrives
        self.result_future = None
        self.distance = None           # latest distance_remaining from feedback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True
        if self.handle is not None:
            try:
                self.handle.cancel_goal_async()
            except Exception:
                pass


class BehaviorGoal:
    """A "move through poses" behavior of the real robot's autopilot.

    The service returns as soon as the behavior has been accepted; the robot
    then plans and drives the route itself, so arrival is tracked through the
    robot's global pose. Cancelling one is not possible through the same
    service (it would need the job/behavior services of the knowledge base),
    so a cancelled behavior keeps running on the robot - the mission log says
    so.
    """

    kind = "behavior"

    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius
        self.distance = None
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class Robot:
    """The ROS 2 link of one target (see targets.py for the configuration)."""

    def __init__(self, cfg, log=print):
        self.cfg = cfg
        self.name = cfg["name"]
        self._log = log
        self._odom = {"x": 0.0, "y": 0.0, "yaw": 0.0, "speed": 0.0}
        self._odom_stamp = -1e9
        self._odom_count = 0
        self._pose = None              # global (map) pose, real robot only
        self._pose_stamp = -1e9
        self._sim_seconds = 0.0
        self._sim_stamp = -1e9
        self._factor = 0.0             # EMA of sim-time / real-time
        self._active_goal = None
        self._has_control = False
        self._nav_error = ""
        self._traj = collections.deque(maxlen=4000)
        self._traj.append((0.0, 0.0))
        self._pubs = {}
        self._clients = {}
        self._nav_clients = {}
        self._lock = threading.Lock()

        self._context = rclpy.Context()
        self._context.init(domain_id=int(cfg.get("domain_id") or 0))
        self.node = Node(f"coffe_orchestrator_{self.name}", context=self._context)
        # The bridge publishes sensor data best-effort; match it so we still
        # receive odometry if the network drops a packet.
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(Odometry, cfg["odom_topic"], self._odom_cb, sensor_qos)
        if cfg.get("clock_topic"):
            self.node.create_subscription(Clock, cfg["clock_topic"], self._clock_cb, 10)
        if cfg.get("pose_topic"):
            self.node.create_subscription(Odometry, cfg["pose_topic"], self._pose_cb, sensor_qos)

        self._executor = MultiThreadedExecutor(num_threads=2, context=self._context)
        self._executor.add_node(self.node)
        self._spin = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin.start()
        self._log(f"[robot:{self.name}] node up on ROS domain "
                  f"{self._context.get_domain_id()}, cmd_vel on {cfg['cmd_vel_topic']}")

    # ------------------------------------------------------------ callbacks

    @staticmethod
    def _pose_of(msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        return p.x, p.y, yaw

    def _odom_cb(self, msg):
        x, y, yaw = self._pose_of(msg)
        with self._lock:
            self._odom = {"x": x, "y": y, "yaw": yaw, "speed": msg.twist.twist.linear.x}
            self._odom_stamp = time.time()
            self._odom_count += 1
            last = self._traj[-1]
            if math.hypot(x - last[0], y - last[1]) > 0.05:
                self._traj.append((x, y))

    def _pose_cb(self, msg):
        x, y, yaw = self._pose_of(msg)
        with self._lock:
            self._pose = {"x": x, "y": y, "yaw": yaw}
            self._pose_stamp = time.time()

    def _clock_cb(self, msg):
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        now = time.time()
        with self._lock:
            if self._sim_stamp > 0:
                wall_dt = now - self._sim_stamp
                sim_dt = sim - self._sim_seconds
                if wall_dt > 0.01 and sim_dt >= 0:
                    f = sim_dt / wall_dt
                    self._factor = self._factor * 0.9 + f * 0.1 if self._factor else f
            self._sim_seconds = sim
            self._sim_stamp = now

    # ------------------------------------------------------------ clocks

    @staticmethod
    def wall_now():
        return time.time()

    def sim_seconds(self):
        with self._lock:
            return self._sim_seconds

    def sim_fresh(self):
        with self._lock:
            return time.time() - self._sim_stamp < CLOCK_TIMEOUT

    def time_factor(self):
        with self._lock:
            return self._factor if time.time() - self._sim_stamp < CLOCK_TIMEOUT else 0.0

    # ------------------------------------------------------------ driving

    def _publisher(self, topic):
        pub = self._pubs.get(topic)
        if pub is None:
            pub = self.node.create_publisher(Twist, topic, 10)
            self._pubs[topic] = pub
        return pub

    def publish_twist(self, msg, topic=None):
        """Publish a Twist message given as {"linear": {...}, "angular": {...}},
        so a raw override can set any field (not just x and z)."""
        twist = Twist()
        for field in ("x", "y", "z"):
            setattr(twist.linear, field, float((msg.get("linear") or {}).get(field, 0.0)))
            setattr(twist.angular, field, float((msg.get("angular") or {}).get(field, 0.0)))
        self._publisher(topic or self.cfg["cmd_vel_topic"]).publish(twist)

    def publish_cmd(self, linear, angular, topic=None):
        self.publish_twist({"linear": {"x": linear}, "angular": {"z": angular}}, topic=topic)

    def stop(self, topic=None):
        """Safety: make sure the robot stops driving."""
        for _ in range(10):
            self.publish_cmd(0.0, 0.0, topic=topic)
            time.sleep(0.05)

    # --------------------------------------------------- control mode

    def _service(self, name, srv_type):
        key = (name, srv_type.__name__)
        client = self._clients.get(key)
        if client is None:
            client = self.node.create_client(srv_type, name)
            self._clients[key] = client
        return client

    def _call(self, name, srv_type, request, timeout=SERVICE_TIMEOUT):
        """Call a service and wait for the answer: (ok, message)."""
        client = self._service(name, srv_type)
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f"{name} is not available (is the robot reachable?)"
        future = client.call_async(request)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, f"{name} did not answer within {timeout:.0f} s"
        try:
            response = future.result()
        except Exception as exc:                      # service call failed
            return False, f"{name} failed: {exc}"
        success = bool(getattr(response, "success", True))
        message = str(getattr(response, "message", "") or "")
        return success, message or ("ok" if success else "refused")

    def take_control(self, topic=None):
        """Take control of the mobile base: (ok, message).

        Only needed on the real robot (and in SIM_DRIVE_MODE=user): the Origin
        One ignores /robot/cmd_vel_user until its control mode is USER. A drive
        step whose raw override points at a USER topic needs it too."""
        service = self.cfg.get("mode_service")
        if not service and _user_topic(topic or self.cfg["cmd_vel_topic"]):
            service = USER_MODE_SERVICE
        if not service:
            return True, f"direct cmd_vel on {self.cfg['cmd_vel_topic']}"
        if self._has_control:
            return True, "already in control"
        try:
            from origin_msgs.srv import SetControlMode
        except ImportError as exc:
            return False, (f"the Avular messages are missing from this image ({exc}) - "
                           "rebuild coffe-orchestrator")
        request = SetControlMode.Request()
        request.mode.mode = int(self.cfg["control_mode"])
        ok, message = self._call(service, SetControlMode, request)
        if ok:
            self._has_control = True
            self._log(f"[robot:{self.name}] took control ({message})")
        return ok, message

    def release_control(self, topic=None):
        """Give control back (mode reset), best effort."""
        service = self.cfg.get("mode_reset_service")
        if not service and _user_topic(topic or self.cfg["cmd_vel_topic"]):
            service = USER_MODE_RESET_SERVICE
        if not service or not self._has_control:
            return True, "no control to release"
        self._has_control = False
        try:
            from origin_msgs.srv import ReturnControlMode
        except ImportError as exc:
            return False, f"the Avular messages are missing from this image ({exc})"
        request = ReturnControlMode.Request()
        request.mode_from.mode = int(self.cfg["control_mode"])
        ok, message = self._call(service, ReturnControlMode, request)
        self._log(f"[robot:{self.name}] released control ({message})")
        return ok, message

    def odom(self):
        with self._lock:
            fresh = time.time() - self._odom_stamp < ODOM_TIMEOUT and self._odom_count > 0
            return dict(self._odom, fresh=fresh)

    def pose(self):
        """Where the robot is in the map frame: the autopilot's global pose on
        the real robot, the odometry on the simulation."""
        with self._lock:
            if self._pose is not None and time.time() - self._pose_stamp < ODOM_TIMEOUT:
                return dict(self._pose, fresh=True)
        return self.odom()

    # ------------------------------------------------------------ navigation

    def nav_ready(self, action=None):
        if self.cfg["nav_kind"] == "action":
            try:
                return self._nav_client(action).server_is_ready()
            except Exception:
                return False
        try:
            from knowledge_base_msgs.srv import SetAndExecuteBehaviors
        except ImportError:
            return False
        return self._service(action or self.cfg["nav_name"],
                             SetAndExecuteBehaviors).service_is_ready()

    def _nav_client(self, name=None):
        name = name or self.cfg["nav_name"]
        client = self._nav_clients.get(name)
        if client is None:
            client = ActionClient(self.node, NavigateToPose, name)
            self._nav_clients[name] = client
        return client

    def nav_send(self, x, y, yaw, frame=None, action=None):
        """Send a navigation goal; None (and nav_error() set) when refused."""
        self._nav_error = ""
        if self.cfg["nav_kind"] == "action":
            return self._send_action_goal(x, y, yaw, action)
        return self._send_behavior(x, y, yaw, frame)

    def _send_action_goal(self, x, y, yaw, action=None):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.cfg["nav_frame"]
        # Keep the stamp at zero: tf2 then uses the latest available transform
        # (AMCL only publishes map->odom while the robot moves, so a pose
        # stamped "now" can be rejected as too old from standstill).
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qz, qw = commands.yaw_quaternion(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        try:
            ng = NavGoal(self._nav_client(action).send_goal_async(
                goal, feedback_callback=self._nav_feedback))
            with self._lock:
                self._active_goal = ng  # engine sends one goal at a time
            return ng
        except Exception as exc:
            self._nav_error = f"Nav2 goal failed: {exc}"
            self._log(f"[robot:{self.name}] {self._nav_error}")
            return None

    def _send_behavior(self, x, y, yaw, frame=None):
        """Ask the real robot's autopilot to plan and drive to a pose
        (MOVE_THROUGH_POSES with a single goal pose)."""
        try:
            from knowledge_base_msgs.msg import Behavior
            from knowledge_base_msgs.srv import SetAndExecuteBehaviors
        except ImportError as exc:
            self._nav_error = (f"the Avular autopilot messages are missing from this "
                               f"image ({exc}) - rebuild coffe-orchestrator")
            return None
        request = SetAndExecuteBehaviors.Request()
        behavior = Behavior()
        behavior.type = MOVE_THROUGH_POSES
        pose = PoseStamped()
        pose.header.frame_id = frame or self.cfg["nav_frame"]
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qz, qw = commands.yaw_quaternion(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        behavior.goals = [pose]
        request.behaviors = [behavior]
        ok, message = self._call(self.cfg["nav_name"], SetAndExecuteBehaviors, request)
        if not ok:
            self._nav_error = f"{self.cfg['nav_name']}: {message}"
            return None
        self._log(f"[robot:{self.name}] autopilot behavior accepted: {message}")
        goal = BehaviorGoal(x, y, self.cfg["arrival_radius"])
        with self._lock:
            self._active_goal = goal
        return goal

    def _nav_feedback(self, wrapper):
        # The feedback wrapper proxies the message fields; _feedback_msg is
        # the attribute name on newer rclpy versions, _feedback on Humble.
        fb = getattr(wrapper, "_feedback_msg", None)
        if fb is None:
            fb = getattr(wrapper, "_feedback", None)
        if fb is None:
            fb = wrapper
        dist = getattr(fb, "distance_remaining", None)
        with self._lock:
            if dist is not None and self._active_goal is not None:
                self._active_goal.distance = float(dist)

    def nav_status(self, goal):
        """(state, distance_remaining); state is one of active / succeeded /
        failed / cancelled / rejected."""
        if goal is None:
            return "failed", None
        if goal.kind == "behavior":
            return self._behavior_status(goal)
        if not goal.future.done():
            return "active", goal.distance
        if goal.handle is None:
            try:
                handle = goal.future.result()
            except Exception:
                return "failed", goal.distance
            goal.handle = handle
            accepted = getattr(handle, "accepted", True)
            if callable(accepted):  # newer rclpy versions use a method
                accepted = accepted()
            if not accepted:
                return "rejected", None
            goal.result_future = handle.get_result_async()
        if goal.cancelled:
            return "cancelled", goal.distance
        if goal.result_future is None or not goal.result_future.done():
            return "active", goal.distance
        resp = goal.result_future.result()
        status = getattr(resp, "status", getattr(goal.handle, "status", None))
        if status == GoalStatus.STATUS_SUCCEEDED:
            return "succeeded", goal.distance
        if status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_CANCELING):
            return "cancelled", goal.distance
        return "failed", goal.distance

    def _behavior_status(self, goal):
        if goal.cancelled:
            return "cancelled", goal.distance
        pose = self.pose()
        if not pose["fresh"]:
            return "active", goal.distance
        distance = math.hypot(pose["x"] - goal.x, pose["y"] - goal.y)
        goal.distance = distance
        if distance <= goal.radius:
            return "succeeded", distance
        return "active", distance

    def nav_cancel(self, goal):
        if goal is None:
            return
        goal.cancel()
        if goal.kind == "behavior":
            self._log(f"[robot:{self.name}] the autopilot keeps executing the behavior it "
                      "was given - stop the robot from Cerebra Studio or with the remote")

    def nav_error(self):
        return self._nav_error

    # ------------------------------------------------------------ waypoints

    def _scene(self):
        try:
            with open(self.cfg.get("scene_file", ""), encoding="utf-8") as f:
                return json.load(f)
        except (OSError, ValueError, TypeError):
            return None

    def waypoint(self, name):
        scene = self._scene()
        if not scene:
            return None
        for wp in scene.get("waypoints", []):
            if wp.get("name", "").lower() == str(name).lower():
                return wp["x"], wp["y"], wp.get("yaw", 0.0)
        return None

    def waypoints(self):
        scene = self._scene()
        return [wp["name"] for wp in (scene or {}).get("waypoints", [])]

    def scene_info(self):
        """Map/waypoint info for the dashboard. The waypoints are in map
        coordinates; the robot odometry starts at the spawn waypoint, so the
        dashboard draws them relative to the spawn point."""
        scene = self._scene()
        if not scene:
            return {"map": None, "spawn": None, "waypoints": []}
        return {
            "map": scene.get("map"),
            "spawn": scene.get("spawn"),
            "waypoints": [
                {"name": w.get("name"), "x": w.get("x", 0.0), "y": w.get("y", 0.0)}
                for w in scene.get("waypoints", [])
            ],
        }

    # ------------------------------------------------------------ dashboard

    def trajectory(self):
        with self._lock:
            return list(self._traj)

    def snapshot(self):
        o = self.odom()
        snap = {
            "target": self.name,
            "domain_id": self._context.get_domain_id(),
            "cmd_vel_topic": self.cfg["cmd_vel_topic"],
            "mode_service": self.cfg.get("mode_service"),
            "in_control": self._has_control,
            "nav_kind": self.cfg["nav_kind"],
            "connected": o["fresh"],
            "x": o["x"], "y": o["y"], "yaw": o["yaw"], "speed": o["speed"],
            "time_factor": self.time_factor(),
            "sim_fresh": self.sim_fresh(),
            "nav_ready": self.nav_ready(),
            "waypoints": self.waypoints(),
            "trajectory": self.trajectory(),
        }
        snap.update(self.scene_info())
        return snap

    def close(self):
        try:
            self._executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            self._context.shutdown()
        except Exception:
            pass
