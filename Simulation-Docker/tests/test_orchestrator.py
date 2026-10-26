"""Unit tests for the COFF-E orchestrator (standard library only).

Tests the mission sequencer - the part that runs the steps, CHECKS every
step and connects the two simulations - with fakes instead of real
hardware, so no Docker, ROS 2 or xArm SDK is needed.

Run from the Simulation-Docker folder:
  py -m unittest tests/test_orchestrator.py
"""

import json
import os
import sys
import tempfile
import unittest

ORCH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "orchestrator")
sys.path.insert(0, ORCH)

import engine  # noqa: E402
import mission_model  # noqa: E402


class FakeRobot:
    """Stands in for ros_robot.Robot; time only advances when the engine
    publishes, polls Nav2 or sleeps, so the tests are fast and exact."""

    def __init__(self, moves=True):
        self.t = 1000.0           # wall-clock seconds
        self.sim = 500.0          # Gazebo seconds
        self.pos = [0.0, 0.0]
        self.moves = moves        # False = blocked (e.g. pushed against a wall)
        self.cmds = []            # (linear, angular) pairs
        self.last_cmd = None
        self.nav_ok = True
        self.last_nav_goal = None
        self.nav_cancelled = False
        self._nav_dist = 0.0
        self.hook = None          # called on every publish_cmd (test hook)

    # clocks
    def wall_now(self):
        return self.t

    def sim_seconds(self):
        return self.sim

    def sim_fresh(self):
        return True

    def time_factor(self):
        return 1.0

    # The robot keeps driving between commands, like the real sim does.
    def _apply_motion(self, dt):
        if self.moves and self.last_cmd is not None:
            self.pos[0] += self.last_cmd[0] * dt

    def sleep(self, seconds):
        self._apply_motion(seconds)
        self.t += seconds
        self.sim += seconds  # like Gazebo: sim time also passes while waiting

    # driving
    def odom(self):
        speed = self.cmds[-1][0] if self.cmds else 0.0
        return {"x": self.pos[0], "y": self.pos[1], "yaw": 0.0,
                "speed": speed, "fresh": True}

    def publish_cmd(self, linear, angular):
        self._apply_motion(0.1)  # 10 Hz publishing
        self.cmds.append((linear, angular))
        self.last_cmd = (linear, angular)
        self.t += 0.1
        self.sim += 0.1
        if self.hook is not None:
            self.hook()

    def stop(self):
        self.cmds.append((0.0, 0.0))
        self.last_cmd = (0.0, 0.0)

    # Nav2
    def nav_ready(self):
        return self.nav_ok

    def waypoint(self, name):
        return {"Home": (2.06, -1.55, 0.0), "Corner": (8.0, 3.0, 1.57)}.get(name)

    def waypoints(self):
        return ["Home", "Corner"]

    def nav_send(self, x, y, yaw):
        self.last_nav_goal = (x, y, yaw)
        self._nav_dist = 3.0
        return object()

    def nav_status(self, goal):
        self.t += 0.5
        self.sim += 0.3
        self._nav_dist = max(0.0, self._nav_dist - 1.0)
        if self._nav_dist <= 0.0:
            return "succeeded", 0.0
        return "active", self._nav_dist

    def nav_cancel(self, goal):
        self.nav_cancelled = True


class FakeArm:
    """Stands in for arm_control.Arm; joints crawl to the target."""

    def __init__(self, step=10.0):
        self.current = [0.0] * 6
        self.step = step          # degrees per angles() call (0 = never moves)
        self.target = None
        self.commanded = None

    def angles(self):
        if self.target is not None:
            self.current = [a + max(-self.step, min(self.step, t - a))
                            for a, t in zip(self.current, self.target)]
        return list(self.current)

    def command_joints(self, angles_deg, speed):
        self.commanded = (list(angles_deg), speed)
        self.target = list(angles_deg)
        return 0


def make_engine(robot=None, arm=None):
    robot = robot or FakeRobot()
    arm = arm or FakeArm()
    return engine.MissionEngine(robot, arm, sleep=robot.sleep), robot, arm


def mission(**kwargs):
    """Minimal normalized mission for direct engine testing."""
    steps = []
    for step in kwargs.pop("steps", []):
        s = {"timeout": None}
        s.update(step)
        steps.append(s)
    doc = {"name": "test mission", "clock": "sim", "steps": steps}
    return mission_model.normalize_mission(doc)


# --------------------------------------------------------------------- model

class TestMissionModel(unittest.TestCase):
    def test_drive_defaults(self):
        m = mission(steps=[{"type": "drive", "duration": 60, "linear": 0.3}])
        s = m["steps"][0]
        self.assertEqual(s["angular"], 0.0)
        self.assertIsNone(s["min_distance"])
        self.assertIsNone(s["timeout"])  # the engine sets a sim-speed-aware one
        self.assertEqual(m["clock"], "sim")

    def test_arm_single_joint_expands_to_six(self):
        m = mission(steps=[{"type": "arm_joint", "joint": 1, "angle_deg": 90}])
        self.assertEqual(m["steps"][0]["angles"], [90, None, None, None, None, None])
        self.assertEqual(m["steps"][0]["tolerance_deg"], 3.0)
        self.assertEqual(m["steps"][0]["speed"], 30)

    def test_arm_home_is_all_zero(self):
        m = mission(steps=[{"type": "arm_home"}])
        self.assertEqual(m["steps"][0]["angles"], [0.0] * 6)

    def test_unknown_type_rejected(self):
        with self.assertRaises(mission_model.MissionError):
            mission(steps=[{"type": "dance"}])

    def test_bad_joint_number_rejected(self):
        with self.assertRaises(mission_model.MissionError):
            mission(steps=[{"type": "arm_joint", "joint": 7, "angle_deg": 90}])

    def test_goto_needs_waypoint_or_xy(self):
        with self.assertRaises(mission_model.MissionError):
            mission(steps=[{"type": "goto"}])

    def test_goto_waypoint_stores_name(self):
        m = mission(steps=[{"type": "goto", "waypoint": "Home"}])
        s = m["steps"][0]
        self.assertEqual(s["waypoint"], "Home")
        self.assertIsNone(s["timeout"])  # engine default: 900 s

    def test_duration_must_be_positive(self):
        with self.assertRaises(mission_model.MissionError):
            mission(steps=[{"type": "wait", "duration": 0}])

    def test_json_mission_round_trip(self):
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "demo.json")
            with open(path, "w", encoding="utf-8") as f:
                json.dump({"name": "json demo", "steps": [
                    {"type": "wait", "duration": 1},
                    {"type": "arm_home"},
                ]}, f)
            m = mission_model.load_mission(path)
            self.assertEqual(m["name"], "json demo")
            self.assertEqual(len(m["steps"]), 2)

    def test_yaml_mission_round_trip(self):
        try:
            import yaml  # noqa: F401
        except ImportError:
            self.skipTest("PyYAML not installed on this machine")
        text = ('name: "yaml demo"\nsteps:\n'
                '  - {type: wait, duration: 2}\n'
                '  - {type: arm_joint, joint: 2, angle_deg: -45}\n')
        m = mission_model.load_mission_text(text, "demo.yaml")
        self.assertEqual(m["name"], "yaml demo")
        self.assertEqual(m["steps"][1]["angles"][1], -45.0)

    def test_broken_mission_reports_syntax_error(self):
        with self.assertRaises(mission_model.MissionError):
            mission_model.load_mission_text("name: [unclosed", "broken.yaml")

    def test_list_missions_shows_broken_files(self):
        with tempfile.TemporaryDirectory() as folder:
            with open(os.path.join(folder, "good.json"), "w", encoding="utf-8") as f:
                json.dump({"name": "good", "steps": [{"type": "wait", "duration": 1}]}, f)
            with open(os.path.join(folder, "bad.json"), "w", encoding="utf-8") as f:
                f.write("{ not json ]")
            with open(os.path.join(folder, "ignored.txt"), "w", encoding="utf-8") as f:
                f.write("not a mission")
            listed = mission_model.list_missions(folder)
            self.assertEqual([e["file"] for e in listed], ["bad.json", "good.json"])
            self.assertIsNone(listed[0]["name"])
            self.assertTrue(listed[0]["error"])
            self.assertEqual(listed[1]["name"], "good")


# -------------------------------------------------------------------- engine

class TestEngineDrive(unittest.TestCase):
    def test_drive_passes_distance_check(self):
        eng, robot, _ = make_engine()
        state = eng.run_blocking(mission(steps=[
            {"type": "drive", "duration": 6, "linear": 0.3,
             "check": {"min_distance": 1.0}},
            {"type": "wait", "duration": 1},
        ]))
        self.assertEqual(state, engine.PASSED)
        self.assertEqual(eng.snapshot()["steps"][0]["status"], engine.STEP_PASSED)
        self.assertIn((0.3, 0.0), robot.cmds)
        # the robot is stopped again after the step: last command is zero
        self.assertEqual(robot.cmds[-1], (0.0, 0.0))

    def test_drive_fails_when_robot_does_not_move(self):
        eng, robot, _ = make_engine(robot=FakeRobot(moves=False))
        state = eng.run_blocking(mission(steps=[
            {"type": "drive", "duration": 4, "linear": 0.3,
             "check": {"min_distance": 1.0}},
            {"type": "wait", "duration": 1},
        ]))
        self.assertEqual(state, engine.FAILED)
        snap = eng.snapshot()
        self.assertEqual(snap["steps"][0]["status"], engine.STEP_FAILED)
        self.assertIn("only moved", snap["steps"][0]["detail"])
        # a failed step means the rest never runs:
        self.assertEqual(snap["steps"][1]["status"], engine.SKIPPED)
        self.assertIn("failed", snap["reason"])
        self.assertEqual(robot.cmds[-1], (0.0, 0.0))  # safety stop

    def test_drive_without_check_passes_without_moving(self):
        eng, robot, _ = make_engine(robot=FakeRobot(moves=False))
        state = eng.run_blocking(mission(steps=[
            {"type": "drive", "duration": 2, "linear": 0.3}]))
        self.assertEqual(state, engine.PASSED)

    def test_drive_uses_wall_clock_when_configured(self):
        eng, robot, _ = make_engine()
        m = mission(steps=[{"type": "drive", "duration": 2, "linear": 0.0}])
        m["clock"] = "wall"
        eng.run_blocking(m)
        self.assertEqual(eng.snapshot()["state"], engine.PASSED)


class TestEngineArm(unittest.TestCase):
    def test_arm_joint_moves_and_passes(self):
        eng, _, arm = make_engine(arm=FakeArm(step=10.0))
        state = eng.run_blocking(mission(steps=[
            {"type": "arm_joint", "joint": 1, "angle_deg": 90, "speed": 60},
        ]))
        self.assertEqual(state, engine.PASSED)
        self.assertEqual(arm.commanded[0], [90.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(arm.commanded[1], 60.0)
        self.assertGreaterEqual(arm.angles()[0], 90.0 - 3.0)

    def test_arm_timeout_fails(self):
        eng, _, arm = make_engine(arm=FakeArm(step=0.0))  # arm never moves
        state = eng.run_blocking(mission(steps=[
            {"type": "arm_joint", "joint": 1, "angle_deg": 90, "timeout": 2.0},
        ]))
        self.assertEqual(state, engine.FAILED)
        self.assertIn("timed out", eng.snapshot()["steps"][0]["detail"])

    def test_arm_unreachable_fails_without_command(self):
        eng, _, arm = make_engine(arm=FakeArm())
        arm.angles = lambda: None  # simulate "no feedback"
        state = eng.run_blocking(mission(steps=[
            {"type": "arm_home", "timeout": 1.0},
        ]))
        self.assertEqual(state, engine.FAILED)
        self.assertIsNone(arm.commanded)
        self.assertIn("arm not connected", eng.snapshot()["steps"][0]["detail"])


class TestEngineGoto(unittest.TestCase):
    def test_goto_waypoint_uses_its_coordinates(self):
        eng, robot, _ = make_engine()
        state = eng.run_blocking(mission(steps=[
            {"type": "goto", "waypoint": "Home"},
        ]))
        self.assertEqual(state, engine.PASSED)
        self.assertEqual(robot.last_nav_goal, (2.06, -1.55, 0.0))

    def test_goto_unknown_waypoint_fails(self):
        eng, robot, _ = make_engine()
        state = eng.run_blocking(mission(steps=[
            {"type": "goto", "waypoint": "Nowhere"},
        ]))
        self.assertEqual(state, engine.FAILED)
        self.assertIsNone(robot.last_nav_goal)
        self.assertIn("not found", eng.snapshot()["steps"][0]["detail"])

    def test_goto_without_nav2_fails(self):
        eng, robot, _ = make_engine(robot=FakeRobot())
        robot.nav_ok = False
        state = eng.run_blocking(mission(steps=[
            {"type": "goto", "x": 1.0, "y": 2.0},
        ]))
        self.assertEqual(state, engine.FAILED)
        self.assertIn("Nav2", eng.snapshot()["steps"][0]["detail"])

    def test_goto_timeout_cancels_goal(self):
        class NeverArrives(FakeRobot):
            def nav_status(self, goal):
                return "active", 42.0
        eng, robot, _ = make_engine(robot=NeverArrives())
        state = eng.run_blocking(mission(steps=[
            {"type": "goto", "waypoint": "Home", "timeout": 1.0},
        ]))
        self.assertEqual(state, engine.FAILED)
        self.assertTrue(robot.nav_cancelled)
        self.assertIn("timed out", eng.snapshot()["steps"][0]["detail"])


class TestEngineSequencing(unittest.TestCase):
    def test_drive_then_arm_then_drive(self):
        """The flow from the request: drive, arm 90 deg, drive further -
        and each step only starts because the previous one passed."""
        eng, robot, arm = make_engine()
        state = eng.run_blocking(mission(steps=[
            {"type": "drive", "duration": 4, "linear": 0.3,
             "check": {"min_distance": 0.5}},
            {"type": "arm_joint", "joint": 1, "angle_deg": 90},
            {"type": "drive", "duration": 4, "linear": -0.3,
             "check": {"min_distance": 0.5}},
        ]))
        self.assertEqual(state, engine.PASSED)
        statuses = [s["status"] for s in eng.snapshot()["steps"]]
        self.assertEqual(statuses, [engine.STEP_PASSED] * 3)
        self.assertEqual(arm.angles()[0], 90.0)

    def test_abort_stops_between_steps(self):
        eng, robot, _ = make_engine()
        robot.hook = eng.abort  # abort as soon as the robot starts driving
        state = eng.run_blocking(mission(steps=[
            {"type": "drive", "duration": 2, "linear": 0.3},
            {"type": "arm_home"},
        ]))
        self.assertEqual(state, engine.ABORTED)
        snap = eng.snapshot()
        self.assertEqual(snap["steps"][0]["status"], engine.STEP_ABORTED)
        self.assertEqual(snap["steps"][1]["status"], "aborted")
        self.assertEqual(robot.cmds[-1], (0.0, 0.0))  # safety stop

    def test_start_rejects_second_mission(self):
        eng, robot, _ = make_engine()
        eng.start(mission(steps=[{"type": "wait", "duration": 100, "timeout": 5.0}]))
        try:
            with self.assertRaises(RuntimeError):
                eng.start(mission(steps=[{"type": "wait", "duration": 1}]))
        finally:
            eng.abort()
            eng._thread.join(timeout=5)
        self.assertEqual(eng.snapshot()["state"], engine.ABORTED)

    def test_mission_can_restart_after_finish(self):
        eng, robot, _ = make_engine()
        eng.run_blocking(mission(steps=[{"type": "wait", "duration": 1}]))
        # after a finished mission a new one may start
        eng.run_blocking(mission(steps=[{"type": "wait", "duration": 1}]))
        self.assertEqual(eng.snapshot()["state"], engine.PASSED)

    def test_snapshot_has_log(self):
        eng, _, _ = make_engine()
        eng.run_blocking(mission(steps=[{"type": "wait", "duration": 1}]))
        snap = eng.snapshot()
        self.assertTrue(any("mission start" in line["text"] for line in snap["log"]))
        self.assertTrue(any("PASSED" in line["text"] for line in snap["log"]))


if __name__ == "__main__":
    unittest.main()
