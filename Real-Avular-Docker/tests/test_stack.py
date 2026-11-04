"""Runs on the host without ROS, Docker, Flask or hardware."""

import copy
import base64
import json
import math
import sys
import threading
import tempfile
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "app"))
from control import Controller, clearance, drive_hint, number, open_spot, validate_point, world_to_cell
from server import make_handler
from sim_map import Source
from places import Store


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class FakeRobot:
    frame = "map"

    def __init__(self, clock):
        self.clock = clock
        self.calls = []
        self.halt_error = False
        self.nav_error = False
        self.seed_error = False
        self.scan = []
        self.values = {"odom_time": clock(), "brake_time": clock(), "pose_time": clock(),
                       "brake": False, "speed": 0.0, "pose": {
                           "x": 1.0, "y": 1.0, "yaw": 0.0, "frame": "map",
                           "variance": [0.01, 0.01, 0.01]}}
        self.grid = {"width": 10, "height": 10, "resolution": 1.0,
                     "origin": [0, 0, 0], "frame": "map", "name": "Lab", "data": [0] * 100}

    def feedback(self):
        return copy.deepcopy(self.values)

    def fresh(self):
        for key in ("odom_time", "brake_time", "pose_time"):
            self.values[key] = self.clock()

    def halt_autopilot(self):
        self.calls.append("halt")
        if self.halt_error:
            raise ValueError("Stop service unavailable")

    def take_control(self):
        self.calls.append("take")

    def release_control(self):
        self.calls.append("release")

    def publish_velocity(self, linear, angular):
        self.calls.append((linear, angular))

    def list_maps(self):
        return [{"id": "lab", "name": "Lab"}]

    def active_map_id(self):
        return "lab"

    def overlay(self):
        return {"scan": list(self.scan), "path": [], "frame": "map", "scan_time": self.clock()}

    def load_map(self, _):
        return copy.deepcopy(self.grid)

    def seed_pose(self, *_, **kw):
        if self.seed_error:
            raise ValueError("No seed route")
        self.calls.append(("seed", kw.get("tight")))

    def navigate(self, *_):
        self.calls.append("navigate")
        if self.nav_error:
            raise ValueError("Request timeout")


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.robot = FakeRobot(self.clock)
        self.c = Controller(self.robot, self.clock)

    def enable(self):
        self.c.enable("browser", True)

    def localize(self):
        self.c.load_map("lab")
        self.c.seed(1, 1, 0)
        self.clock.now += 0.1
        self.robot.fresh()
        self.c.confirm_localization(True)

    def advance(self, seconds):
        self.clock.now += seconds
        self.robot.fresh()
        self.c.ping("browser")

    def test_startup_is_read_only(self):
        self.c.tick()
        self.assertEqual(self.robot.calls, [])
        self.assertFalse(self.c.enabled)

    def test_checklist_and_owner_required(self):
        with self.assertRaises(ValueError):
            self.c.enable("browser", False)
        self.enable()
        with self.assertRaises(ValueError):
            self.c.drive("other", 0.1, 0)
        with self.assertRaises(ValueError):
            self.c.enable("other", True)

    def test_enable_aborts_old_work_before_user_mode(self):
        self.enable()
        self.assertEqual(self.robot.calls[:2], ["halt", "take"])

    def test_manual_limits_and_finite_values(self):
        self.enable()
        for linear, angular in [(0.61, 0), (0, 1.01), (float("nan"), 0), (0, float("inf"))]:
            with self.assertRaises(ValueError):
                self.c.drive("browser", linear, angular)
        with self.assertRaises(ValueError):
            number(True)

    def test_manual_deadman_independent_of_heartbeat(self):
        self.enable()
        self.c.drive("browser", 0.1, 0)
        self.c.tick()
        self.assertEqual(self.robot.calls[-1], (0.1, 0.0))
        self.advance(0.36)
        self.c.tick()
        self.assertEqual(self.robot.calls[-1], (0.0, 0.0))
        self.assertTrue(self.c.enabled)

    def test_heartbeat_loss_latches_stop_without_release(self):
        self.enable()
        self.c.drive("browser", 0.1, 0)
        self.clock.now += 1.6
        self.robot.fresh()
        self.c.tick()
        self.assertFalse(self.c.enabled)
        self.assertTrue(self.c.holding)
        self.assertNotIn("release", self.robot.calls)
        self.assertEqual(self.robot.calls[-1], (0.0, 0.0))

    def test_stale_odom_and_brake_stop(self):
        for key in ("odom_time", "brake_time"):
            with self.subTest(key=key):
                self.robot.fresh()
                self.enable()
                self.robot.values[key] = -1e9
                self.c.tick()
                self.assertFalse(self.c.enabled)
        self.robot.fresh()
        self.enable()
        self.robot.values["brake"] = True
        self.c.tick()
        self.assertFalse(self.c.enabled)

    def test_stop_takes_zero_control_before_waiting_on_autopilot(self):
        self.c.stop()
        self.assertLess(self.robot.calls.index("take"), self.robot.calls.index("halt"))

    def test_unconfirmed_stop_cannot_release(self):
        self.robot.halt_error = True
        self.assertFalse(self.c.stop())
        self.assertEqual(self.c.mode, "STOP UNCONFIRMED")
        with self.assertRaises(ValueError):
            self.c.release()

    def test_enable_failure_still_holds_zero(self):
        self.robot.halt_error = True
        with self.assertRaises(ValueError):
            self.enable()
        self.assertFalse(self.c.enabled)
        self.assertIn("take", self.robot.calls)

    def test_successful_stop_allows_explicit_release_only(self):
        self.enable()
        with self.assertRaises(ValueError):
            self.c.release()
        self.c.stop()
        self.c.release()
        self.assertEqual(self.robot.calls[-1], "release")
        self.assertFalse(self.c.holding)

    def test_map_load_requires_disabled_motion_and_clears_localization(self):
        self.localize()
        self.assertTrue(self.c.localized)
        self.enable()
        with self.assertRaises(ValueError):
            self.c.load_map("other")
        self.c.stop()
        self.c.load_map("lab")
        self.assertFalse(self.c.localized)
        self.assertLess(self.robot.calls.index("halt"), next(i for i, c in enumerate(self.robot.calls) if c[0] == "seed"))

    def test_localization_can_be_confirmed_while_enabled_but_not_while_driving(self):
        self.c.load_map("lab")
        self.c.seed(1, 1, 0)
        self.enable()
        self.c.drive("browser", 0, 0.2)
        self.advance(0.1)
        with self.assertRaisesRegex(ValueError, "Release"):
            self.c.confirm_localization(True)
        self.advance(0.4)
        self.c.tick()
        self.c.confirm_localization(True)
        self.assertTrue(self.c.localized)

    def test_already_localized_robot_can_be_confirmed_without_new_seed(self):
        self.c.load_map("lab")
        with self.assertRaisesRegex(ValueError, "No new estimated pose"):
            self.c.confirm_localization(True)
        self.advance(0.1)
        self.c.confirm_localization(True)
        self.assertTrue(self.c.localized)

    def test_high_uncertainty_message_says_what_to_do(self):
        self.c.load_map("lab")
        self.advance(0.1)
        self.robot.values["pose"]["variance"][0] = 0.25004
        with self.assertRaisesRegex(ValueError, "turn in place"):
            self.c.confirm_localization(True)
        self.assertFalse(self.c.localized)

    def test_seed_allowed_while_enabled_but_not_while_moving(self):
        self.c.load_map("lab")
        self.enable()
        self.c.seed(2, 2, 0.5)
        self.assertEqual(self.c.snapshot()["seed_pose"], {"x": 2.0, "y": 2.0, "yaw": 0.5})
        self.c.drive("browser", 0, 0.2)
        with self.assertRaisesRegex(ValueError, "Release"):
            self.c.seed(1, 1, 0)
        self.advance(0.4)
        self.c.tick()
        self.robot.values["speed"] = 0.1
        with self.assertRaisesRegex(ValueError, "standing still"):
            self.c.seed(1, 1, 0)
        self.robot.values["speed"] = 0.0
        self.c.seed(1, 1, 0)

    def test_seed_requires_zero_hold(self):
        self.c.load_map("lab")
        self.c.release()
        with self.assertRaisesRegex(ValueError, "zero speed"):
            self.c.seed(1, 1, 0)

    def test_manual_fast_level_is_accepted(self):
        self.enable()
        self.c.drive("browser", 0.30, 0.60)
        self.assertEqual(self.c.velocity, (0.30, 0.60))

    def walls(self, on_wall):
        """Occupied border on the 10x10 test map; 'on_wall' laser points hit it."""
        for i in range(10):
            self.robot.grid["data"][i] = 100
        self.c.grid["data"] = list(self.robot.grid["data"])
        self.robot.scan = [(5.5, 0.5)] * on_wall + [(5.5, 5.5)] * (200 - on_wall)

    def test_good_laser_match_overrides_unreliable_covariance(self):
        self.c.load_map("lab")
        self.walls(160)
        self.advance(0.1)
        self.robot.values["pose"]["variance"] = [2.0, 7.0, 0.09]
        self.c.confirm_localization(True)
        self.assertTrue(self.c.localized)
        self.assertEqual(self.c.snapshot()["laser_match"], (0.8, 200))

    def test_poor_laser_match_refuses_even_with_low_covariance(self):
        self.c.load_map("lab")
        self.walls(60)
        self.advance(0.1)
        with self.assertRaisesRegex(ValueError, "30% of the laser"):  # below 40%
            self.c.confirm_localization(True)
        self.assertFalse(self.c.localized)

    def test_navigation_tolerates_brief_but_not_sustained_laser_mismatch(self):
        self.c.load_map("lab")
        self.walls(160)
        self.robot.values["pose"]["y"] = 2.0
        self.advance(0.1)
        self.c.confirm_localization(True)
        self.enable()
        self.c.navigate("browser", 5, 5, 0)
        self.walls(20)
        for _ in range(25):
            self.advance(0.1)
            self.c.tick()
        self.assertTrue(self.c.enabled)
        self.walls(160)
        self.advance(0.1)
        self.c.tick()
        self.walls(20)
        for _ in range(35):
            self.advance(0.1)
            self.c.tick()
        self.assertFalse(self.c.enabled)
        self.assertIn("Localization lost", self.c.last_run["ended"])

    def status(self, accuracy):
        self.robot.values.update(autopilot={"accuracy": accuracy, "behavior": "Waiting for proper position"},
                                 autopilot_time=self.clock())

    def test_seed_is_tight_until_autopilot_is_accurate(self):
        self.c.load_map("lab")
        self.status("inaccurate_position")
        self.c.seed(1, 1, 0)
        self.status("accurate")
        self.c.seed(1, 1, 0)
        self.assertEqual([c for c in self.robot.calls if c[0] == "seed"], [("seed", True), ("seed", False)])

    def test_confirm_and_navigate_refused_until_autopilot_accurate(self):
        self.c.load_map("lab")
        self.advance(0.1)
        self.status("inaccurate_position")
        with self.assertRaisesRegex(ValueError, "inaccurate_position"):
            self.c.confirm_localization(True)
        self.status("accurate")
        self.c.confirm_localization(True)
        self.enable()
        self.status("inaccurate_position")
        with self.assertRaisesRegex(ValueError, "inaccurate_position"):
            self.c.navigate("browser", 5, 5, 0)
        self.assertNotIn("navigate", self.robot.calls)

    def test_auto_fit_finds_offset_pose_without_commanding_robot(self):
        grid = {"width": 60, "height": 60, "resolution": 0.1, "origin": [0, 0, 0], "frame": "map",
                "name": "Room", "data": [0] * 3600}
        for i in range(60):
            for r, c in ((0, i), (59, i), (i, 0), (i, 59), (30, i if i < 20 else 0)):
                grid["data"][r * 60 + c] = 100
        self.robot.grid = grid
        self.c.load_map("room")
        # Laser as seen from the true pose (3.0, 2.0, 0.3), reported while the
        # estimate is wrong at (2.5, 2.4, 0.0); scan points are in map frame.
        import math as m
        true = (3.0, 2.0, 0.3)
        local = []
        for k in range(180):
            a = k * 2 * m.pi / 180
            for d in [i * 0.05 for i in range(1, 200)]:
                wx = true[0] + d * m.cos(true[2] + a)
                wy = true[1] + d * m.sin(true[2] + a)
                cell = int(wy / 0.1) * 60 + int(wx / 0.1)
                if not (0 <= wx < 6 and 0 <= wy < 6) or grid["data"][cell] >= 50:
                    local.append((d * m.cos(a), d * m.sin(a)))
                    break
        est = (2.5, 2.4, 0.0)
        self.robot.values["pose"].update(x=est[0], y=est[1], yaw=est[2])
        self.robot.scan = [(est[0] + m.cos(est[2]) * x - m.sin(est[2]) * y,
                            est[1] + m.sin(est[2]) * x + m.cos(est[2]) * y) for x, y in local]
        before = list(self.robot.calls)
        r = self.c.fit_pose({"x": 2.6, "y": 2.3, "yaw": 0.1})
        self.assertEqual(self.robot.calls, before)
        self.assertGreater(r["match"], 0.9)
        self.assertLess(m.hypot(r["x"] - 3.0, r["y"] - 2.0), 0.1)
        self.assertLess(abs(r["yaw"] - 0.3), 0.05)
        self.assertIsNone(r["rival"])

    def test_autopilot_status_and_robot_warnings_reach_the_log(self):
        self.robot.logs = [{"level": "WARN", "name": "planner", "msg": "Starting point in lethal space"}] * 3
        self.robot.drain_logs = lambda: [self.robot.logs.pop()] if self.robot.logs else []
        self.robot.values.update(bt_status="Waiting for behaviors to execute.", bt_status_time=self.clock())
        for _ in range(3):
            self.c.tick()
        lines = [e.split("  ", 1)[1] for e in self.c.events]
        self.assertIn("Autopilot: Waiting for behaviors to execute.", lines)
        self.assertEqual(lines.count("Robot WARN [planner]: Starting point in lethal space"), 1)
        self.assertEqual(self.robot.calls, [])

    def test_goal_that_does_not_move_is_reported(self):
        self.localize()
        self.enable()
        self.c.navigate("browser", 5, 5, 0)
        for _ in range(22):
            self.advance(1.0)
            self.c.tick()
        self.assertTrue(any("has not moved" in e for e in self.c.events))

    def test_start_in_map_wall_refuses_without_releasing_user_hold(self):
        self.localize()
        self.enable()
        self.c.grid["data"][11] = 100
        before = list(self.robot.calls)
        with self.assertRaisesRegex(ValueError, "Starting point in lethal space"):
            self.c.navigate("browser", 5, 5, 0)
        self.assertEqual(self.robot.calls, before)
        self.assertIsNone(self.c.goal)
        self.assertTrue(self.c.holding)
        cl = self.c.snapshot()["clearance"]
        self.assertFalse(cl["ok"])
        self.assertEqual(cl["metres"], 0)
        self.assertIn("open map floor", cl["hint"])

    def test_clearance_uses_cell_edges_and_rotated_origins(self):
        grid = copy.deepcopy(self.robot.grid)
        grid["data"][11] = 100
        self.assertAlmostEqual(clearance(grid, 2.1, 1.5), 0.1)
        self.assertEqual(clearance(grid, 1.5, 1.5), 0)
        grid["origin"] = [10, 20, math.pi / 2]
        self.assertAlmostEqual(clearance(grid, 8.5, 22.1), 0.1)
        self.assertEqual(clearance(grid, 9.5, 19), 0)

    def test_open_spot_is_map_only_and_hint_is_in_robot_frame(self):
        grid = copy.deepcopy(self.robot.grid)
        grid["data"][11] = 100
        spot = open_spot(grid, 1.5, 1.5, math.pi / 2)
        self.assertIsNotNone(spot)
        ahead, left = spot
        px, py = 1.5 - left, 1.5 + ahead
        validate_point(grid, px, py)
        self.assertGreaterEqual(clearance(grid, px, py), 0.4)
        self.assertEqual(drive_hint((-0.45, 0.35)), "open map floor is 0.45 m back and 0.35 m to its left")
        grid["data"] = [100] * 100
        self.assertIsNone(open_spot(grid, 1.5, 1.5, 0))

    def test_autopilot_failure_stops_immediately_with_current_warning(self):
        self.localize()
        self.enable()
        self.c.navigate("browser", 5, 5, 0)
        self.advance(0.1)
        self.robot.values.update(bt_status="Behavior did not complete.", bt_status_time=self.clock())
        self.robot.drain_logs = lambda: [{"level": "WARN", "name": "planner", "msg": "Starting point in lethal space"}]
        self.c.tick()
        self.assertIsNone(self.c.goal)
        self.assertFalse(self.c.enabled)
        self.assertTrue(self.c.holding)
        self.assertIn("Starting point in lethal space", self.c.last_run["ended"])
        self.assertEqual(self.robot.calls[-1], (0.0, 0.0))

    def test_failure_transition_is_not_lost_when_followed_by_waiting(self):
        self.localize()
        self.enable()
        self.c.navigate("browser", 5, 5, 0)
        self.advance(0.1)
        self.robot.values.update(bt_status="Waiting for behaviors to execute.",
                                 bt_status_time=self.clock(), bt_failure_time=self.clock())
        self.c.tick()
        self.assertIsNone(self.c.goal)
        self.assertIn("gave up", self.c.last_run["ended"])

    def test_old_failure_does_not_stop_new_goal(self):
        self.localize()
        self.enable()
        self.robot.values.update(bt_status="Behavior did not complete.",
                                 bt_status_time=self.clock() - 1, bt_failure_time=self.clock() - 1)
        self.c.navigate("browser", 5, 5, 0)
        self.advance(0.1)
        self.status("accurate")
        self.c.tick()
        self.assertIsNotNone(self.c.goal)
        self.assertTrue(self.c.enabled)

    def test_manual_speed_cap_is_configurable_but_bounded(self):
        c = Controller(self.robot, self.clock, max_linear=5, max_angular=0.4)
        self.assertEqual((c.max_linear, c.max_angular), (1.0, 0.4))
        c.enable("browser", True)
        c.drive("browser", 1.0, 0.4)
        with self.assertRaises(ValueError):
            c.drive("browser", 0, 0.5)

    def test_seed_is_not_localization(self):
        self.c.load_map("lab")
        self.c.seed(1, 1, 0)
        self.assertFalse(self.c.localized)
        with self.assertRaises(ValueError):
            self.c.confirm_localization(True)

    def test_failed_seed_cannot_be_confirmed(self):
        self.c.load_map("lab")
        self.robot.seed_error = True
        with self.assertRaises(ValueError):
            self.c.seed(1, 1, 0)
        self.advance(0.1)
        with self.assertRaises(ValueError):
            self.c.confirm_localization(True)

    def test_localization_requires_matching_frame_and_low_covariance(self):
        self.c.load_map("lab")
        self.c.seed(1, 1, 0)
        self.advance(0.1)
        self.robot.values["pose"]["frame"] = "odom"
        with self.assertRaises(ValueError):
            self.c.confirm_localization(True)
        self.robot.values["pose"]["frame"] = "map"
        self.robot.values["pose"]["variance"][0] = 99
        with self.assertRaises(ValueError):
            self.c.confirm_localization(True)
        self.assertFalse(self.c.localized)

    def test_navigation_requires_confirmed_localization(self):
        self.enable()
        with self.assertRaises(ValueError):
            self.c.navigate("browser", 2, 2, 0)

    def test_navigation_switch_and_manual_exclusion(self):
        self.localize()
        self.enable()
        self.c.navigate("browser", 2, 2, 0)
        self.assertEqual(self.robot.calls[-2:], ["release", "navigate"])
        with self.assertRaises(ValueError):
            self.c.drive("browser", 0.1, 0)

    def test_navigation_timeout_stops_and_takes_user_control(self):
        self.localize()
        self.enable()
        self.c.navigate("browser", 2, 2, 0)
        self.advance(181)
        self.c.tick()
        self.assertFalse(self.c.enabled)
        self.assertTrue(self.c.holding)

    def test_ambiguous_navigation_request_stops(self):
        self.localize()
        self.enable()
        self.robot.nav_error = True
        with self.assertRaises(ValueError):
            self.c.navigate("browser", 2, 2, 0)
        self.assertFalse(self.c.enabled)
        self.assertTrue(self.c.holding)

    def test_localization_loss_stops_navigation(self):
        self.localize()
        self.enable()
        self.c.navigate("browser", 2, 2, 0)
        self.robot.values["pose_time"] = -1e9
        self.c.tick()
        self.assertFalse(self.c.enabled)

    def test_stale_pose_cannot_release_user_for_new_goal(self):
        self.localize()
        self.enable()
        self.robot.values["pose_time"] = -1e9
        with self.assertRaises(ValueError):
            self.c.navigate("browser", 2, 2, 0)
        self.assertNotIn("release", self.robot.calls)

    def test_invalid_pose_cannot_be_used_to_navigate(self):
        self.localize()
        self.enable()
        self.robot.values["pose"]["x"] = float("nan")
        with self.assertRaises(ValueError):
            self.c.navigate("browser", 2, 2, 0)
        self.assertNotIn("release", self.robot.calls)

    def test_arrival_requires_heading_and_standstill_dwell(self):
        self.localize()
        self.enable()
        self.c.navigate("browser", 1, 1, 1)
        self.c.tick()
        self.advance(1.1)
        self.c.tick()
        self.assertTrue(self.c.enabled)  # wrong heading, not arrived
        self.robot.values["pose"]["yaw"] = 1
        self.c.tick()
        self.advance(1.1)
        self.c.tick()
        self.assertFalse(self.c.enabled)
        self.assertIn("Arrival observed", self.c.events[-1])

    def test_points_reject_occupied_unknown_and_outside(self):
        for x, y in [(-0.1, 1), (10, 1)]:
            with self.assertRaises(ValueError):
                validate_point(self.robot.grid, x, y)
        for occupancy in (-1, 100):
            self.robot.grid["data"][11] = occupancy
            with self.assertRaises(ValueError):
                validate_point(self.robot.grid, 1, 1)

    def test_rotated_grid_coordinates(self):
        self.robot.grid["origin"] = [10, 20, math.pi / 2]
        self.assertEqual(world_to_cell(self.robot.grid, 8.5, 22.5), (2, 1))

    def test_map_listing_includes_active_identity(self):
        self.assertEqual(self.c.list_maps()["active"], "lab")
        self.assertEqual(self.c.list_maps()["maps"], self.robot.list_maps())

    def test_unmatched_simulation_map_is_read_only_preview(self):
        class Sim:
            def info(s):
                return {"id": "sim-id", "name": "Simulation"}
            def grid(s, frame):
                return copy.deepcopy(self.robot.grid)
        self.c = Controller(self.robot, self.clock, Sim())
        grid = self.c.load_sim_map()
        self.assertTrue(grid["preview"])
        self.assertFalse(self.c.localized)
        self.assertEqual(self.robot.calls, [])
        with self.assertRaisesRegex(ValueError, "preview"):
            self.c.seed(1, 1, 0)
        with self.assertRaises(ValueError):
            self.c.confirm_localization(True)
        self.enable()
        with self.assertRaisesRegex(ValueError, "Preview"):
            self.c.navigate("browser", 2, 2, 0)

    def test_matching_simulation_map_loads_onboard(self):
        class Sim:
            def info(s):
                return {"id": "lab", "name": "Lab"}
        self.c = Controller(self.robot, self.clock, Sim())
        self.c.load_sim_map()
        self.assertEqual(self.c.map_id, "lab")
        self.assertIn("halt", self.robot.calls)
        self.assertFalse(self.c.snapshot()["preview"])

    def test_simulation_map_change_refused_while_enabled(self):
        self.enable()
        with self.assertRaisesRegex(ValueError, "Stop"):
            self.c.load_sim_map()

    def test_navigation_records_estimated_driven_and_remaining_distance(self):
        self.localize()
        self.enable()
        self.c.navigate("browser", 4, 1, 0)
        self.robot.values["pose"]["x"] = 2
        self.c.tick()
        self.assertEqual(self.c.goal["driven"], 1)
        self.assertEqual(self.c.goal["remaining"], 2)
        self.c.stop()
        self.assertEqual(self.c.last_run["driven"], 1)
        self.assertEqual(self.c.last_run["ended"], "Operator stop")


class RemoteModeAndPlacesTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.robot = FakeRobot(self.clock)
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.c = Controller(self.robot, self.clock, places=Store(str(Path(self.folder.name) / "p.json")),
                            default_map={"id": "lab", "name": "Lab"})

    def test_manual_remote_mode_blocks_enable_before_any_robot_call(self):
        self.robot.values.update(control_mode=10, control_mode_time=self.clock())
        with self.assertRaisesRegex(ValueError, "MANUAL"):
            self.c.enable("browser", True)
        self.assertEqual(self.robot.calls, [])
        self.assertEqual(self.c.snapshot()["remote_mode"], "MANUAL (remote)")

    def test_stale_or_user_mode_does_not_block(self):
        self.robot.values.update(control_mode=10, control_mode_time=self.clock() - 10)
        self.assertIsNone(self.c.remote_mode())
        self.robot.values.update(control_mode=30, control_mode_time=self.clock())
        self.c.enable("browser", True)
        self.assertTrue(self.c.enabled)

    def test_unconfirmed_stop_names_the_remote(self):
        self.robot.halt_error = True
        self.robot.values.update(control_mode=10, control_mode_time=self.clock())
        self.assertFalse(self.c.stop())
        self.assertIn("remote has control", self.c.events[-1])

    def test_places_need_onboard_map_and_free_space(self):
        with self.assertRaisesRegex(ValueError, "Load an onboard map"):
            self.c.save_place("Coffee", 1, 1, 0)
        self.c.load_map("lab")
        with self.assertRaises(ValueError):
            self.c.save_place("Wall", 50, 50, 0)
        self.c.save_place("Coffee", 2, 3, 1.5)
        self.c.save_place("Coffee", 4, 3, 0)
        self.assertEqual(self.c.list_places(), [{"name": "Coffee", "x": 4, "y": 3, "yaw": 0}])
        self.assertEqual(self.c.delete_place("Coffee"), [])

    def test_places_are_per_map_and_persist(self):
        self.c.load_map("lab")
        self.c.save_place("Home", 1, 1, 0)
        store = Store(str(Path(self.folder.name) / "p.json"))
        self.assertEqual(store.list("lab")[0]["name"], "Home")
        self.assertEqual(store.list("other"), [])
        with self.assertRaises(ValueError):
            store.save("lab", "", 1, 1, 0)

    def test_snapshot_reports_seed_and_default_map(self):
        self.assertFalse(self.c.snapshot()["seeded"])
        self.c.load_map("lab")
        self.c.seed(1, 1, 0)
        snap = self.c.snapshot()
        self.assertTrue(snap["seeded"])
        self.assertEqual(snap["default_map"]["id"], "lab")


class SimMapTests(unittest.TestCase):
    def test_cerebra_grid_dimensions_origin_unknown_and_wrapped_resource(self):
        with tempfile.TemporaryDirectory() as folder:
            data = {"id": "{ABC}", "name": "Sim", "size": {
                "width_in_pixels": 2, "height_in_pixels": 2, "width_in_meters": .1},
                "origin": {"x": -1, "y": -2},
                "occupancy_grid": base64.b64encode(bytes([0, 100, 255, 50])).decode()}
            path = Path(folder) / "abc.json"
            path.write_text(json.dumps({"resource": data}), encoding="utf-8")
            source = Source(folder, "Sim")
            self.assertEqual(source.info()["id"], "abc")
            grid = source.grid("map")
            self.assertEqual(grid["data"], [0, 100, -1, 50])
            self.assertEqual(grid["origin"], [-1, -2, 0])
            self.assertEqual(grid["resolution"], .05)
            self.assertEqual(Source(folder, "abc").info()["name"], "Sim")

    def test_missing_map_is_actionable(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError, "not found"):
                Source(folder, "missing").grid("map")


class HttpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = FakeRobot(Clock())
        cls.c = Controller(cls.robot, Clock())
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(cls.c, "test-token"))
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.url = "http://127.0.0.1:" + str(cls.server.server_port)

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()

    def request(self, path, body=None, token=None, headers=None):
        data = None if body is None else json.dumps(body).encode()
        headers = dict(headers or {})
        if token is not None:
            headers["X-COFFE-Token"] = token
        req = urllib.request.Request(self.url + path, data=data, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=2) as response:
                return response.status, response.read()
        except urllib.error.HTTPError as exc:
            return exc.code, exc.read()

    def test_page_and_state_are_read_only(self):
        self.robot.calls.clear()
        code, page = self.request("/")
        self.assertEqual(code, 200)
        self.assertIn(b"test-token", page)
        self.assertNotIn(b"__TOKEN__", page)
        code, state = self.request("/api/state")
        self.assertEqual(code, 200)
        self.assertFalse(json.loads(state)["enabled"])
        self.assertEqual(self.robot.calls, [])

    def test_command_requires_token(self):
        self.assertEqual(self.request("/api/enable", {"owner": "a", "confirmed": True})[0], 403)

    def test_cross_site_commands_rejected(self):
        self.assertEqual(self.request("/api/stop", {"owner": "a"}, "test-token",
                                      {"Sec-Fetch-Site": "cross-site"})[0], 403)

    def test_invalid_body_does_not_touch_hardware(self):
        self.robot.calls.clear()
        self.assertEqual(self.request("/api/stop", [], "test-token")[0], 400)
        self.assertEqual(self.robot.calls, [])

    def test_unknown_route(self):
        self.assertEqual(self.request("/unknown")[0], 404)
        self.assertEqual(self.request("/api/unknown", {"owner": "a"}, "test-token")[0], 404)

    def test_overlays_and_map_listing_are_read_only(self):
        self.robot.calls.clear()
        code, body = self.request("/api/overlay")
        self.assertEqual(code, 200)
        self.assertEqual(json.loads(body)["scan"], [])
        code, body = self.request("/api/maps")
        self.assertEqual(code, 200)
        self.assertEqual(json.loads(body)["active"], "lab")
        self.assertEqual(self.robot.calls, [])

    def test_places_listing_is_read_only_and_save_needs_token(self):
        self.robot.calls.clear()
        self.assertEqual(json.loads(self.request("/api/places")[1]), [])
        self.assertEqual(self.request("/api/places/save", {"owner": "a", "name": "x",
                                                         "x": 1, "y": 1, "yaw": 0})[0], 403)
        self.assertEqual(self.robot.calls, [])

    def test_sim_map_load_requires_token(self):
        self.assertEqual(self.request("/api/load-sim-map", {"owner": "a"})[0], 403)


if __name__ == "__main__":
    unittest.main()
