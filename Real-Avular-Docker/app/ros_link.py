"""Real Origin One ROS2 adapter. Startup only creates ROS entities, no motion."""

import math
import os
import secrets
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque

import rclpy
from autonomy_msgs.msg import Path, StatusString, SystemStatus
from autonomy_msgs.srv import Trigger as AutonomyTrigger
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from knowledge_base_msgs.msg import Behavior
from knowledge_base_msgs.srv import (
    GetActiveMapID, GetMap, ListMaps, PublishLocalizationMap, SetAndExecuteBehaviors)
from nav_msgs.msg import Odometry
from origin_msgs.srv import ReturnControlMode, SetControlMode
from origin_msgs.msg import BatteryInfo, ControlMode, NetworkTelemetry
from rcl_interfaces.msg import Log
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def fixed_joints(urdf):
    """(parent, child, xyz, quaternion xyzw) for every fixed URDF joint."""
    result = []
    for joint in ET.fromstring(urdf).iter("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        if joint.get("type") != "fixed" or parent is None or child is None:
            continue
        origin = joint.find("origin")
        xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
        r, p, y = [float(v) / 2 for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        quat = [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]
        result.append((parent.get("link"), child.get("link"), xyz, quat))
    return result


class Robot:
    def __init__(self):
        rclpy.init()
        # zenoh-bridge-ros2dds tracks nodes by name: when an earlier process
        # with the same name expires, it also drops the live node's routes.
        self.node = Node("coffe_real_avular_" + secrets.token_hex(3))
        self.frame = os.environ.get("NAV_FRAME", "map")
        self.pose_topic = os.environ.get("POSE_TOPIC", "/autopilot/estimated_pose")
        self.seed_topic = os.environ.get("INITIAL_POSE_TOPIC", "/initialpose")
        self.lock = threading.RLock()
        self.values = {"odom_time": -1e9, "brake_time": -1e9, "pose_time": -1e9,
                       "brake": True, "speed": 0.0, "pose": None,
                       "battery": None, "battery_time": -1e9,
                       "network": None, "network_time": -1e9,
                       "control_mode": None, "control_mode_time": -1e9,
                       "autopilot": None, "autopilot_time": -1e9}
        self.odom_stamp = None
        self.robot_logs = deque(maxlen=200)
        self.command = (0.0, 0.0)
        self.command_deadline = -1e9
        self.user_control = False
        self.clients = {}
        self.overlays = {"scan": [], "path": [], "scan_time": -1e9, "path_time": -1e9,
                         "scan_status": "Waiting for laser scan", "path_status": "Waiting for onboard plan",
                         "frame": self.frame}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.velocity_pub = self.node.create_publisher(Twist, "/robot/cmd_vel_user", 10)
        self.seed_pub = self.node.create_publisher(PoseWithCovarianceStamped, self.seed_topic, 10)
        # /initialpose feeds the pose filter, this one AMCL. AMCL drops the
        # message silently unless it is transient-local.
        self.amcl_seed_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, os.environ.get("AMCL_POSE_TOPIC", "/autopilot/amcl/initial_pose"),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.node.create_subscription(Odometry, "/robot/odom", self._odom, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, self.pose_topic, self._pose, qos_profile_sensor_data)
        self.node.create_subscription(LaserScan, "/autopilot/lidar/scan_filtered",
                                      self._scan, qos_profile_sensor_data)
        self.node.create_subscription(Path, "/autopilot/plan_twist",
                                      self._path, qos_profile_sensor_data)
        self.node.create_subscription(BatteryInfo, "/robot/battery/info",
                                      self._battery, qos_profile_sensor_data)
        self.node.create_subscription(NetworkTelemetry, "/robot/origin_network_telemetry/network",
                                      self._network, qos_profile_sensor_data)
        self.node.create_subscription(SystemStatus, "/autopilot/information_manager/system_status",
                                      self._system_status, 10)
        self.node.create_subscription(StatusString, "/autopilot/behaviortree_executor/status",
                                      self._bt_status, 10)
        # Volatile on purpose: the robot keeps 1000 old log lines per node
        # and a transient-local reader would replay all of them.
        self.node.create_subscription(Log, "/rosout", self._rosout,
                                      QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.node.create_subscription(ControlMode, "/robot/control_mode",
                                      self._control_mode, qos_profile_sensor_data)
        # zenoh-bridge-ros2dds caches one transient-local sample per topic, so
        # most of Chud's /tf_static never arrives (no base_link -> ouster_base
        # and therefore no laser overlay). The robot description is complete;
        # its fixed joints are added to the LOCAL buffer only, never published.
        self.node.create_subscription(
            String, "/robot/robot_description", self._description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.node.create_subscription(
            Bool, os.environ.get("BRAKE_TOPIC", "/robot/safety_supervisor/emergency_brake_active"),
            self._brake, qos_profile_sensor_data)
        # This timer is independent of HTTP/controller locks. A service timeout
        # or blocked browser handler cannot keep the last nonzero Twist alive.
        self.node.create_timer(0.1, self._publish)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _odom(self, msg):
        t = msg.twist.twist
        if not all(math.isfinite(v) for v in (t.linear.x, t.linear.y, t.angular.z)):
            return
        with self.lock:
            self.odom_stamp = msg.header.stamp
            self.values.update(odom_time=time.monotonic(),
                               speed=max(math.hypot(t.linear.x, t.linear.y), abs(t.angular.z)))

    def _pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        variance = [msg.pose.covariance[i] for i in (0, 7, 35)]
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w, *variance)):
            return
        with self.lock:
            self.values.update(pose_time=time.monotonic(), pose={
                "x": p.x, "y": p.y, "yaw": yaw_of(q), "frame": msg.header.frame_id,
                "variance": variance})

    def _description(self, msg):
        try:
            joints = fixed_joints(msg.data)
        except Exception:
            return
        for parent, child, xyz, quat in joints:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = quat
            self.tf_buffer.set_transform_static(t, "robot_description")

    def _brake(self, msg):
        with self.lock:
            self.values.update(brake=msg.data, brake_time=time.monotonic())

    def _battery(self, msg):
        if not 0 <= msg.state_of_charge <= 100 or not math.isfinite(msg.voltage):
            return
        with self.lock:
            self.values.update(battery={"percent": msg.state_of_charge, "voltage": msg.voltage},
                               battery_time=time.monotonic())

    def _network(self, msg):
        with self.lock:
            self.values.update(network={"ssid": msg.wifi_ssid, "strength": msg.wifi_strength},
                               network_time=time.monotonic())

    def _system_status(self, msg):
        with self.lock:
            self.values.update(autopilot={"accuracy": msg.position_accuracy_status,
                                          "behavior": msg.behavior_status,
                                          "operational": bool(msg.is_operational),
                                          "scheduling": bool(msg.scheduling_enabled),
                                          "mode": msg.mode_of_operation},
                               autopilot_time=time.monotonic())

    def _bt_status(self, msg):
        with self.lock:
            self.values.update(bt_status=msg.status, bt_status_time=time.monotonic())
            if "did not complete" in msg.status.lower():
                self.values["bt_failure_time"] = self.values["bt_status_time"]

    def _rosout(self, msg):
        # 30 = WARN, 40 = ERROR, 50 = FATAL. Our own node's lines are skipped.
        if msg.level >= 30 and not msg.name.startswith("coffe_real_avular"):
            with self.lock:
                self.robot_logs.append({"level": {30: "WARN", 40: "ERROR"}.get(msg.level, "FATAL"),
                                        "name": msg.name, "msg": msg.msg[:300]})

    def drain_logs(self):
        with self.lock:
            logs = list(self.robot_logs)
            self.robot_logs.clear()
        return logs

    def _control_mode(self, msg):
        with self.lock:
            self.values.update(control_mode=int(msg.mode), control_mode_time=time.monotonic())

    def feedback(self):
        with self.lock:
            return dict(self.values)

    def overlay(self):
        with self.lock:
            return dict(self.overlays)

    def _map_points(self, header, points):
        if header.frame_id == self.frame:
            return points
        transform = self.tf_buffer.lookup_transform(
            self.frame, header.frame_id, Time.from_msg(header.stamp)).transform
        q, t = transform.rotation, transform.translation
        result = []
        for x, y, z in points:
            # Quaternion rotation, including sensor roll/pitch.
            ux, uy, uz = 2 * (q.y*z-q.z*y), 2 * (q.z*x-q.x*z), 2 * (q.x*y-q.y*x)
            result.append((x+q.w*ux+q.y*uz-q.z*uy+t.x,
                           y+q.w*uy+q.z*ux-q.x*uz+t.y,
                           z+q.w*uz+q.x*uy-q.y*ux+t.z))
        return result

    def _store_overlay(self, kind, header, points):
        try:
            points = self._map_points(header, points)
            points = [[x, y] for x, y, z in points if all(map(math.isfinite, (x, y, z)))]
            with self.lock:
                self.overlays.update({kind: points, kind+"_time": time.monotonic(),
                                      kind+"_status": f"{len(points)} points in {self.frame}"})
        except Exception:
            with self.lock:
                self.overlays.update({kind: [], kind+"_time": -1e9,
                                      kind+"_status": f"No TF {header.frame_id} → {self.frame}"})

    def _scan(self, msg):
        points = []
        stride = max(1, math.ceil(len(msg.ranges) / 720))
        for i in range(0, len(msg.ranges), stride):
            r = msg.ranges[i]
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                a = msg.angle_min + i * msg.angle_increment
                points.append((r * math.cos(a), r * math.sin(a), 0.0))
        self._store_overlay("scan", msg.header, points)

    def _path(self, msg):
        stride = max(1, math.ceil(len(msg.poses) / 2000))
        points = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                  for p in msg.poses[::stride]]
        self._store_overlay("path", msg.header, points)

    def publish_velocity(self, linear, angular):
        with self.lock:
            self.command = linear, angular
            self.command_deadline = time.monotonic() + 0.25
        # Stops do not wait for the next timer tick.
        if not linear and not angular:
            self.velocity_pub.publish(Twist())

    def _publish(self):
        with self.lock:
            if not self.user_control:
                return
            linear, angular = self.command if time.monotonic() < self.command_deadline else (0.0, 0.0)
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.velocity_pub.publish(msg)

    def _call(self, service, kind, request, timeout=2.0):
        key = service, kind
        if key not in self.clients:
            self.clients[key] = self.node.create_client(kind, service)
        client = self.clients[key]
        if not client.wait_for_service(timeout_sec=1.0):
            raise ValueError(service + " unavailable")
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            # Cancelling our local future does NOT cancel execution on the robot.
            raise ValueError(service + " timed out; execution is uncertain, use physical stop")
        response = future.result()
        if response is None or not response.success:
            raise ValueError(service + ": " + str(getattr(response, "message", "no response")))
        return response

    def halt_autopilot(self):
        # Chud acknowledges stop_execution but advertises a nonresponsive
        # legacy abort endpoint. Never waive the required stop acknowledgment.
        self._call("/autopilot/behaviortree_executor/stop_execution",
                   AutonomyTrigger, AutonomyTrigger.Request())
        try:
            self._call("/autopilot/behaviortree_executor/abort", Trigger,
                       Trigger.Request(), timeout=0.5)
        except Exception:
            return "Onboard stop acknowledged; optional legacy abort did not acknowledge."
        return ""

    def take_control(self):
        request = SetControlMode.Request()
        request.mode.mode = 30
        # Publish zero even if the response is lost after the mode changed.
        with self.lock:
            self.user_control = True
            self.command = (0.0, 0.0)
        self._call("/robot/cmd_vel_controller/set_control_mode", SetControlMode, request)

    def release_control(self):
        request = ReturnControlMode.Request()
        request.mode_from.mode = 30
        self._call("/robot/cmd_vel_controller/reset_control_mode", ReturnControlMode, request)
        with self.lock:
            self.user_control = False
            self.command = (0.0, 0.0)

    def list_maps(self):
        response = self._call("/autopilot/information_manager/list_maps", ListMaps, ListMaps.Request(),
                              timeout=30.0)
        return [{"id": m.uuid, "name": m.name} for m in response.maps]

    def active_map_id(self):
        response = self._call("/autopilot/information_manager/resource/map/get_active_id",
                              GetActiveMapID, GetActiveMapID.Request(), timeout=30.0)
        return response.uuid

    def load_map(self, map_id):
        get = GetMap.Request()
        get.uuid = map_id
        response = self._call("/autopilot/information_manager/get_map", GetMap, get, timeout=30.0)
        grid = response.map.map
        width, height = grid.info.width, grid.info.height
        if not width or not height or width * height > 4_000_000:
            raise ValueError("Empty map or map larger than four million cells")
        if len(grid.data) != width * height or grid.info.resolution <= 0:
            raise ValueError("Invalid occupancy grid")
        origin = grid.info.origin
        result = {"width": width, "height": height, "resolution": grid.info.resolution,
                  "origin": [origin.position.x, origin.position.y, yaw_of(origin.orientation)],
                  "frame": grid.header.frame_id, "data": list(grid.data),
                  "name": response.map.name}
        if result["frame"] != self.frame:
            raise ValueError("Selected map frame differs from NAV_FRAME")
        # Reloading the active map restarts localization onboard, which throws
        # away a pose the robot already has. Only switch when it differs.
        try:
            active = self.active_map_id()
        except Exception:
            active = None
        norm = lambda v: str(v or "").strip("{}").lower()
        result["already_active"] = norm(active) == norm(map_id)
        if result["already_active"]:
            return result
        request = PublishLocalizationMap.Request()
        request.uuid = map_id
        self._call("/autopilot/information_manager/load_localization_map", PublishLocalizationMap, request,
                   timeout=30.0)
        return result

    def seed_pose(self, x, y, yaw, tight=True):
        types = dict(self.node.get_topic_names_and_types()).get(self.seed_topic, [])
        expected = "geometry_msgs/msg/PoseWithCovarianceStamped"
        subscribers = self.node.get_subscriptions_info_by_topic(self.seed_topic)
        # Behind Zenoh the subscriber is normally the bridge, not AMCL.
        # This proves route availability, not that onboard AMCL consumed it.
        if types != [expected] or not subscribers:
            raise ValueError("Initial pose route absent or wrong type; check INITIAL_POSE_TOPIC")
        with self.lock:
            stamp = self.odom_stamp
        if stamp is None:
            raise ValueError("No odometry stamp from the robot yet; wait for odometry")
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame
        # The robot's own clock (its uptime, not wall time): with the laptop's
        # time the pose filter ignores AMCL afterwards.
        msg.header.stamp = stamp
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        xy, heading = (0.005, 0.005) if tight else (0.05, 0.03)
        msg.pose.covariance[0] = msg.pose.covariance[7] = xy
        msg.pose.covariance[35] = heading

        def send(left):
            self.seed_pub.publish(msg)
            self.amcl_seed_pub.publish(msg)
            if left > 1:
                timer = threading.Timer(0.3, send, (left - 1,))
                timer.daemon = True
                timer.start()
        # Repeated because a single sample over Wi-Fi/Zenoh is sometimes lost.
        send(5)

    def navigate(self, x, y, yaw):
        behavior = Behavior()
        behavior.type = Behavior.MOVE_THROUGH_POSES
        pose = PoseStamped()
        pose.header.frame_id = self.frame
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        behavior.goals = [pose]
        request = SetAndExecuteBehaviors.Request()
        request.behaviors = [behavior]
        response = self._call("/autopilot/information_manager/set_and_execute_behaviors",
                              SetAndExecuteBehaviors, request)
        # Every enable/stop calls stop_execution, which leaves the executor
        # idle ("Waiting for behaviors to execute."): a stored goal then never
        # runs until the executor is started again.
        started = self._call("/autopilot/behaviortree_executor/start_execution",
                             AutonomyTrigger, AutonomyTrigger.Request())
        return (f"set_and_execute: {getattr(response, 'message', '') or 'ok'}; "
                f"start_execution: {getattr(started, 'message', '') or 'ok'}")

    def close(self):
        self.executor.shutdown(timeout_sec=2.0)
        self.node.destroy_node()
        rclpy.shutdown()
