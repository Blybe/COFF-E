"""Exercise the actual adapter methods with import-only ROS stand-ins."""

import importlib.util
import sys
import threading
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch


def load_adapter():
    modules = {}
    exports = {
        "rclpy": [],
        "autonomy_msgs.srv": ["Trigger"],
        "autonomy_msgs.msg": ["Path", "StatusString", "SystemStatus"],
        "rcl_interfaces.msg": ["Log"],
        "geometry_msgs.msg": ["PoseStamped", "PoseWithCovarianceStamped", "TransformStamped", "Twist"],
        "knowledge_base_msgs.msg": ["Behavior"],
        "knowledge_base_msgs.srv": ["GetActiveMapID", "GetMap", "ListMaps", "PublishLocalizationMap",
                                   "SetAndExecuteBehaviors"],
        "nav_msgs.msg": ["Odometry"],
        "origin_msgs.srv": ["ReturnControlMode", "SetControlMode"],
        "origin_msgs.msg": ["BatteryInfo", "ControlMode", "NetworkTelemetry"],
        "rclpy.executors": ["MultiThreadedExecutor"],
        "rclpy.node": ["Node"],
        "rclpy.qos": ["DurabilityPolicy", "QoSProfile", "ReliabilityPolicy", "qos_profile_sensor_data"],
        "rclpy.time": ["Time"],
        "sensor_msgs.msg": ["LaserScan"],
        "tf2_ros": ["Buffer", "TransformListener"],
        "std_msgs.msg": ["Bool", "String"],
        "std_srvs.srv": ["Trigger"],
    }
    for path, names in exports.items():
        modules[path] = ModuleType(path)
        if "." in path:
            modules.setdefault(path.split(".")[0], ModuleType(path.split(".")[0]))
        for name in names:
            setattr(modules[path], name, Mock())
    modules["geometry_msgs.msg"].Twist = lambda: SimpleNamespace(
        linear=SimpleNamespace(x=0.0), angular=SimpleNamespace(z=0.0))
    modules["origin_msgs.srv"].SetControlMode.Request = lambda: SimpleNamespace(
        mode=SimpleNamespace(mode=0))
    spec = importlib.util.spec_from_file_location(
        "adapter_under_test", Path(__file__).resolve().parents[1] / "app" / "ros_link.py")
    adapter = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(adapter)
    return adapter


adapter = load_adapter()


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.robot = adapter.Robot.__new__(adapter.Robot)
        self.robot.lock = threading.RLock()
        self.robot.frame = "map"
        self.robot.velocity_pub = Mock()
        self.robot.command = (0.1, 0.2)
        self.robot.command_deadline = 100.0
        self.robot.user_control = True

    def test_ros_timer_independently_expires_nonzero_velocity(self):
        with patch.object(adapter.time, "monotonic", return_value=99.9):
            self.robot._publish()
        msg = self.robot.velocity_pub.publish.call_args.args[0]
        self.assertEqual((msg.linear.x, msg.angular.z), (0.1, 0.2))
        with patch.object(adapter.time, "monotonic", return_value=100.1):
            self.robot._publish()
        msg = self.robot.velocity_pub.publish.call_args.args[0]
        self.assertEqual((msg.linear.x, msg.angular.z), (0.0, 0.0))

    def test_failure_timestamp_survives_next_waiting_status(self):
        self.robot.values = {}
        with patch.object(adapter.time, "monotonic", return_value=100.0):
            self.robot._bt_status(SimpleNamespace(status="Behavior did not complete."))
        with patch.object(adapter.time, "monotonic", return_value=100.1):
            self.robot._bt_status(SimpleNamespace(status="Waiting for behaviors to execute."))
        self.assertEqual(self.robot.values["bt_failure_time"], 100.0)
        self.assertEqual(self.robot.values["bt_status_time"], 100.1)

    def test_ros_timer_does_not_publish_on_read_only_startup(self):
        self.robot.user_control = False
        self.robot._publish()
        self.robot.velocity_pub.publish.assert_not_called()

    def test_zero_is_published_without_waiting_for_timer(self):
        self.robot.publish_velocity(0, 0)
        self.robot.velocity_pub.publish.assert_called_once()

    def test_lost_take_control_response_still_runs_zero_timer(self):
        self.robot._call = Mock(side_effect=ValueError("response lost"))
        with self.assertRaises(ValueError):
            self.robot.take_control()
        self.robot._publish()
        msg = self.robot.velocity_pub.publish.call_args.args[0]
        self.assertEqual((msg.linear.x, msg.angular.z), (0.0, 0.0))

    def test_seed_requires_a_subscriber_not_only_our_own_publisher(self):
        self.robot.node = Mock()
        self.robot.seed_topic = "/initial"
        self.robot.node.get_topic_names_and_types.return_value = [
            ("/initial", ["geometry_msgs/msg/PoseWithCovarianceStamped"])]
        self.robot.node.get_subscriptions_info_by_topic.return_value = []
        with self.assertRaisesRegex(ValueError, "route absent"):
            self.robot.seed_pose(1, 1, 0)

    def test_invalid_ros_pose_does_not_refresh_feedback(self):
        self.robot.values = {"pose_time": -1e9}
        msg = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=float("nan"), y=0),
            orientation=SimpleNamespace(x=0, y=0, z=0, w=1)), covariance=[0] * 36))
        self.robot._pose(msg)
        self.assertEqual(self.robot.values["pose_time"], -1e9)

    def test_optional_abort_timeout_does_not_mask_confirmed_stop(self):
        self.robot._call = Mock(side_effect=[SimpleNamespace(success=True), ValueError("timeout")])
        self.assertIn("stop acknowledged", self.robot.halt_autopilot())
        self.assertEqual(self.robot._call.call_args.kwargs["timeout"], 0.5)

    def test_required_stop_failure_still_fails_closed(self):
        self.robot._call = Mock(side_effect=ValueError("stop unavailable"))
        with self.assertRaisesRegex(ValueError, "stop unavailable"):
            self.robot.halt_autopilot()
        self.assertEqual(self.robot._call.call_count, 1)

    def test_active_map_id_uses_resource_service(self):
        self.robot._call = Mock(return_value=SimpleNamespace(uuid="map-id"))
        self.assertEqual(self.robot.active_map_id(), "map-id")
        self.assertTrue(self.robot._call.call_args.args[0].endswith("/resource/map/get_active_id"))

    def test_scan_filters_invalid_ranges_and_downsamples(self):
        self.robot._store_overlay = Mock()
        msg = SimpleNamespace(header=Mock(), ranges=[1.0, float("inf"), .01, 20.0],
                              range_min=.1, range_max=10, angle_min=0, angle_increment=.1)
        self.robot._scan(msg)
        self.assertEqual(self.robot._store_overlay.call_args.args[2], [(1.0, 0.0, 0.0)])
        msg.ranges = [1.0] * 4000
        self.robot._scan(msg)
        self.assertLessEqual(len(self.robot._store_overlay.call_args.args[2]), 720)

    def test_path_uses_pose_with_twist_positions(self):
        self.robot._store_overlay = Mock()
        msg = SimpleNamespace(header=Mock(), poses=[SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=1, y=2, z=0)))])
        self.robot._path(msg)
        self.assertEqual(self.robot._store_overlay.call_args.args[2], [(1, 2, 0)])

    def test_transform_applies_rotation_and_translation(self):
        self.robot.frame = "map"
        self.robot.tf_buffer = Mock()
        self.robot.tf_buffer.lookup_transform.return_value.transform = SimpleNamespace(
            rotation=SimpleNamespace(x=0, y=0, z=2**-.5, w=2**-.5),
            translation=SimpleNamespace(x=10, y=20, z=0))
        points = self.robot._map_points(SimpleNamespace(frame_id="laser", stamp=Mock()), [(1, 0, 0)])
        self.assertAlmostEqual(points[0][0], 10)
        self.assertAlmostEqual(points[0][1], 21)

    def test_missing_transform_clears_overlay_instead_of_guessing(self):
        self.robot.overlays = {"scan": [[1, 2]], "scan_time": 100}
        self.robot._map_points = Mock(side_effect=ValueError("no TF"))
        self.robot._store_overlay("scan", SimpleNamespace(frame_id="laser"), [(1, 2, 0)])
        self.assertEqual(self.robot.overlays["scan"], [])
        self.assertEqual(self.robot.overlays["scan_time"], -1e9)

    def test_battery_reports_percent_and_voltage_without_scaling(self):
        self.robot.values = {}
        self.robot._battery(SimpleNamespace(state_of_charge=95, voltage=48.8))
        self.assertEqual(self.robot.values["battery"], {"percent": 95, "voltage": 48.8})
        before = self.robot.values["battery_time"]
        self.robot._battery(SimpleNamespace(state_of_charge=255, voltage=48.8))
        self.assertEqual(self.robot.values["battery_time"], before)
        self.robot._battery(SimpleNamespace(state_of_charge=95, voltage=float("nan")))
        self.assertEqual(self.robot.values["battery_time"], before)

    def test_control_mode_is_recorded(self):
        self.robot.values = {}
        self.robot._control_mode(SimpleNamespace(mode=10))
        self.assertEqual(self.robot.values["control_mode"], 10)

    def test_robot_network_retains_disconnected_client_state(self):
        self.robot.values = {}
        self.robot._network(SimpleNamespace(wifi_ssid="Not connected", wifi_strength=0))
        self.assertEqual(self.robot.values["network"], {"ssid": "Not connected", "strength": 0})

    def seed_robot(self):
        self.robot.node = Mock()
        self.robot.seed_topic = "/initialpose"
        self.robot.node.get_topic_names_and_types.return_value = [
            ("/initialpose", ["geometry_msgs/msg/PoseWithCovarianceStamped"])]
        self.robot.node.get_subscriptions_info_by_topic.return_value = ["bridge"]
        self.robot.seed_pub, self.robot.amcl_seed_pub = Mock(), Mock()
        self.robot.odom_stamp = "robot-clock"

    def test_seed_goes_to_filter_and_amcl_with_robot_stamp_and_repeats(self):
        self.seed_robot()
        published = []
        adapter.PoseWithCovarianceStamped = lambda: SimpleNamespace(
            header=SimpleNamespace(frame_id="", stamp=None),
            pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0, y=0),
                                                      orientation=SimpleNamespace(z=0, w=1)),
                                 covariance=[0.0] * 36))
        with patch.object(adapter.threading, "Timer") as timer:
            self.robot.seed_pose(1, 2, 0, tight=True)
        msg = self.robot.seed_pub.publish.call_args.args[0]
        self.robot.amcl_seed_pub.publish.assert_called_once_with(msg)
        self.assertEqual(msg.header.stamp, "robot-clock")
        self.assertEqual((msg.pose.covariance[0], msg.pose.covariance[35]), (0.005, 0.005))
        self.assertEqual(timer.call_args.args[0], 0.3)

    def test_loose_seed_covariance_when_already_accurate(self):
        self.seed_robot()
        adapter.PoseWithCovarianceStamped = lambda: SimpleNamespace(
            header=SimpleNamespace(frame_id="", stamp=None),
            pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0, y=0),
                                                      orientation=SimpleNamespace(z=0, w=1)),
                                 covariance=[0.0] * 36))
        with patch.object(adapter.threading, "Timer"):
            self.robot.seed_pose(1, 2, 0, tight=False)
        msg = self.robot.seed_pub.publish.call_args.args[0]
        self.assertEqual((msg.pose.covariance[0], msg.pose.covariance[35]), (0.05, 0.03))

    def test_seed_without_odometry_stamp_is_refused(self):
        self.seed_robot()
        self.robot.odom_stamp = None
        with self.assertRaisesRegex(ValueError, "odometry stamp"):
            self.robot.seed_pose(1, 2, 0)

    def test_navigate_starts_the_executor_after_setting_the_goal(self):
        self.robot._call = Mock(return_value=SimpleNamespace(success=True, message="done"))
        with patch.object(adapter, "Behavior", Mock(return_value=SimpleNamespace(type=0, goals=[]),
                                                       MOVE_THROUGH_POSES=10)), \
                patch.object(adapter, "PoseStamped", lambda: SimpleNamespace(
                    header=SimpleNamespace(frame_id=""), pose=SimpleNamespace(
                        position=SimpleNamespace(x=0, y=0), orientation=SimpleNamespace(z=0, w=1)))):
            note = self.robot.navigate(1, 2, 0)
        services = [c.args[0] for c in self.robot._call.call_args_list]
        self.assertTrue(services[0].endswith("set_and_execute_behaviors"))
        self.assertTrue(services[1].endswith("start_execution"))
        self.assertIn("start_execution: done", note)

    def test_rosout_keeps_only_robot_warnings(self):
        from collections import deque
        self.robot.robot_logs = deque()
        for level, name in ((20, "planner"), (30, "planner"), (40, "coffe_real_avular_ab"), (40, "bt")):
            self.robot._rosout(SimpleNamespace(level=level, name=name, msg="m"))
        self.assertEqual([(e["level"], e["name"]) for e in self.robot.drain_logs()],
                         [("WARN", "planner"), ("ERROR", "bt")])
        self.assertEqual(self.robot.drain_logs(), [])

    def map_response(self):
        info = SimpleNamespace(width=2, height=1, resolution=0.05, origin=SimpleNamespace(
            position=SimpleNamespace(x=0, y=0), orientation=SimpleNamespace(x=0, y=0, z=0, w=1)))
        return SimpleNamespace(success=True, map=SimpleNamespace(name="M", map=SimpleNamespace(
            info=info, data=[0, 0], header=SimpleNamespace(frame_id="map"))))

    def test_active_map_is_not_reloaded(self):
        self.robot._call = Mock(return_value=self.map_response())
        self.robot.active_map_id = Mock(return_value="{ABC}")
        grid = self.robot.load_map("abc")
        self.assertTrue(grid["already_active"])
        self.assertEqual(self.robot._call.call_count, 1)
        self.robot.active_map_id = Mock(return_value="other")
        self.assertFalse(self.robot.load_map("abc")["already_active"])
        self.assertTrue(self.robot._call.call_args.args[0].endswith("load_localization_map"))

    def odom(self, sec):
        twist = SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0), angular=SimpleNamespace(z=0.0))
        self.robot._odom(SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=0)),
                                         twist=SimpleNamespace(twist=twist)))

    def test_robot_clock_restart_resets_tf_and_keeps_lidar_mount(self):
        self.robot.values, self.robot.odom_stamp = {}, None
        self.robot.tf_buffer, self.robot.description_tf = Mock(), ["ouster mount"]
        self.robot.overlays = {"scan": [[1, 2]], "scan_time": 100}
        for sec in (5000, 5001, 5000.5):
            self.odom(sec)
        self.robot.tf_buffer.clear.assert_not_called()
        self.odom(1437)
        self.robot.tf_buffer.clear.assert_called_once()
        self.robot.tf_buffer.set_transform_static.assert_called_once_with("ouster mount", "robot_description")
        self.assertEqual((self.robot.overlays["scan"], self.robot.overlays["scan_time"]), ([], -1e9))
        self.assertEqual(self.robot.odom_stamp.sec, 1437)
        self.odom(1438)
        self.robot.tf_buffer.clear.assert_called_once()

    def test_fixed_urdf_joints_give_lidar_mount(self):
        urdf = """<robot name="o"><link name="base_link"/><link name="main_body"/><link name="ouster_base"/>
          <link name="wheel"/>
          <joint name="base_body" type="fixed"><parent link="base_link"/><child link="main_body"/>
            <origin rpy="0 0 0" xyz="0.0 0 0.064"/></joint>
          <joint name="ouster" type="fixed"><parent link="main_body"/><child link="ouster_base"/>
            <origin rpy="0 0 1.5708" xyz="0.272 0 0.189"/></joint>
          <joint name="w" type="continuous"><parent link="main_body"/><child link="wheel"/></joint></robot>"""
        joints = adapter.fixed_joints(urdf)
        self.assertEqual([(p, c) for p, c, _, _ in joints], [("base_link", "main_body"), ("main_body", "ouster_base")])
        self.assertEqual(joints[1][2], [0.272, 0.0, 0.189])
        self.assertAlmostEqual(joints[1][3][2], 2 ** -.5, places=4)
        self.assertAlmostEqual(joints[1][3][3], 2 ** -.5, places=4)


if __name__ == "__main__":
    unittest.main()
