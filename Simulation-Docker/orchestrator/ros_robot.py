"""ROS 2 side of the orchestrator: the Avular Origin One mobile base.

One rclpy node that talks to the simulation containers over the normal ROS 2
(DDS) network, exactly like the origin-nav container does:

  /robot/cmd_vel   publishes: drives the simulated Origin One
  /robot/odom      subscribes: where the robot actually is (the check for
                   every drive step)
  /clock           subscribes: Gazebo time, so "drive for 60 s" means 60 s of
                   simulation time (the sim runs at ~0.5x real time)
  navigate_to_pose action client: sends the Origin One to a Cerebra waypoint
                   or a map coordinate with Nav2 (planner runs in origin-nav)

The node is spun in a background thread; the engine and the dashboard call
the small methods below from their own threads.
"""

import collections
import json
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

ODOM_TIMEOUT = 5.0        # s: /robot/odom older than this counts as "no robot"
CLOCK_TIMEOUT = 3.0       # s: /clock older than this counts as "no sim time"
CMD_RATE = 10.0            # Hz at which drive commands are (re)sent


class NavGoal:
    """Small wrapper around a NavigateToPose goal, for polling."""

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


class Robot:
    def __init__(self, scene_file="/tmp/cerebra/scene.json", log=print):
        self._log = log
        self._scene_file = scene_file
        self._odom = {"x": 0.0, "y": 0.0, "yaw": 0.0, "speed": 0.0}
        self._odom_stamp = -1e9
        self._odom_count = 0
        self._sim_seconds = 0.0
        self._sim_stamp = -1e9
        self._factor = 0.0             # EMA of sim-time / real-time
        self._active_goal = None       # NavGoal whose feedback we store
        self._traj = collections.deque(maxlen=4000)
        self._traj.append((0.0, 0.0))
        self._lock = threading.Lock()

        if not rclpy.ok():
            rclpy.init()
        self.node = Node("coffe_orchestrator")
        # The bridge publishes sensor data best-effort; match it so we still
        # receive odometry if the network drops a packet.
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(Odometry, "/robot/odom", self._odom_cb, sensor_qos)
        self.node.create_subscription(Clock, "/clock", self._clock_cb, 10)
        self.cmd_pub = self.node.create_publisher(Twist, "/robot/cmd_vel", 10)
        self.nav = ActionClient(self.node, NavigateToPose, "navigate_to_pose")

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._spin = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin.start()
        self._log("[robot] orchestrator node up, waiting for /robot/odom and /clock")

    # ------------------------------------------------------------ callbacks

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        speed = msg.twist.twist.linear.x
        with self._lock:
            self._odom = {"x": p.x, "y": p.y, "yaw": yaw, "speed": speed}
            self._odom_stamp = time.time()
            self._odom_count += 1
            last = self._traj[-1]
            if math.hypot(p.x - last[0], p.y - last[1]) > 0.05:
                self._traj.append((p.x, p.y))

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

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        """Safety: make sure the robot stops driving."""
        for _ in range(10):
            self.publish_cmd(0.0, 0.0)
            time.sleep(0.05)

    def odom(self):
        with self._lock:
            fresh = time.time() - self._odom_stamp < ODOM_TIMEOUT and self._odom_count > 0
            return dict(self._odom, fresh=fresh)

    # ------------------------------------------------------------ Nav2

    def nav_ready(self):
        try:
            return self.nav.server_is_ready()
        except Exception:
            return False

    def nav_send(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        # Keep the stamp at zero: tf2 then uses the latest available transform
        # (AMCL only publishes map->odom while the robot moves, so a pose
        # stamped "now" can be rejected as too old from standstill).
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        try:
            ng = NavGoal(self.nav.send_goal_async(
                goal, feedback_callback=self._nav_feedback))
            with self._lock:
                self._active_goal = ng  # engine sends one goal at a time
            return ng
        except Exception as exc:
            self._log(f"[robot] Nav2 goal failed: {exc}")
            return None

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
        """Returns (state, distance_remaining). state is one of
        active / succeeded / failed / cancelled / rejected."""
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

    def nav_cancel(self, goal):
        if goal is not None:
            goal.cancel()

    # ------------------------------------------------------------ waypoints

    def _scene(self):
        try:
            with open(self._scene_file, encoding="utf-8") as f:
                return json.load(f)
        except (OSError, ValueError):
            return None

    def waypoint(self, name):
        scene = self._scene()
        if not scene:
            return None
        for wp in scene.get("waypoints", []):
            if wp.get("name", "").lower() == name.lower():
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
