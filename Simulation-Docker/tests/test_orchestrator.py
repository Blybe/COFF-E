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

import commands  # noqa: E402
import engine  # noqa: E402
import mission_model  # noqa: E402
import targets  # noqa: E402


class FakeRobot:
    """Stands in for ros_robot.Robot; time only advances when the engine
    publishes, polls Nav2 or sleeps, so the tests are fast and exact."""

    def __init__(self, moves=True):
        self.t = 1000.0           # wall-clock seconds
        self.sim = 500.0          # Gazebo seconds
        self.pos = [0.0, 0.0]
        self.moves = moves        # False = blocked (e.g. pushed against a wall)
        self.cmds = []            # (linear, angular) pairs
        self.cmd_topics = []      # topic of every drive message (None = default)
        self.twists = []          # (message, topic) of every publish_twist
        self.control = []         # "take"/"release" calls
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

    def publish_cmd(self, linear, angular, topic=None):
        self._apply_motion(0.1)  # 10 Hz publishing
        self.cmds.append((linear, angular))
        self.cmd_topics.append(topic)
        self.last_cmd = (linear, angular)
        self.t += 0.1
        self.sim += 0.1
        if self.hook is not None:
            self.hook()

    def publish_twist(self, msg, topic=None):
        self.twists.append((msg, topic))
        self.publish_cmd((msg.get("linear") or {}).get("x", 0.0),
                         (msg.get("angular") or {}).get("z", 0.0), topic=topic)

    def stop(self):
        self.cmds.append((0.0, 0.0))
        self.last_cmd = (0.0, 0.0)

    def odom_topic(self):
        return "/robot/odom"

    def nav_description(self):
        return "Nav2 action /navigate_to_pose"

    # control mode (USER mode on the real Origin One)
    def take_control(self, topic=None):
        self.control.append("take")
        self.control_topic = topic
        return [("sim", True, "direct cmd_vel")]

    def release_control(self, topic=None):
        self.control.append("release")
        return [("sim", True, "no control to release")]

    def pose(self):
        return self.odom()

    # Nav2
    def nav_ready(self, action=None):
        return self.nav_ok

    def waypoint(self, name):
        return {"Home": (2.06, -1.55, 0.0), "Corner": (8.0, 3.0, 1.57)}.get(name)

    def waypoints(self):
        return ["Home", "Corner"]

    def nav_send(self, x, y, yaw, frame=None, action=None):
        self.last_nav_goal = (x, y, yaw)
        self.last_nav_frame = frame
        self.last_nav_action = action
        self._nav_dist = 3.0
        return object()

    def nav_error(self):
        return ""

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

    def command_joints(self, angles_deg, speed, acc=None, wait=False):
        self.commanded = (list(angles_deg), speed)
        self.acc = acc
        self.wait = wait
        self.target = list(angles_deg)
        return {"sim": 0}

    def angles_all(self):
        return [("sim", self.angles())]


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


class TestLiveEdit(unittest.TestCase):
    """The step editor: upcoming steps can be changed while a mission runs."""

    def _doc(self, *steps):
        return {"name": "live", "steps": list(steps)}

    def test_upcoming_steps_are_replaced_while_running(self):
        eng, robot, arm = make_engine()
        drive = {"type": "drive", "duration": 2, "linear": 0.3}
        first = mission_model.normalize_mission(self._doc(drive, {"type": "wait", "duration": 1}))
        edited = mission_model.normalize_mission(self._doc(
            drive, {"type": "arm_joint", "joint": 2, "angle_deg": 45},
            {"type": "wait", "duration": 1}))
        done = []

        def edit_once():
            if not done:
                done.append(True)
                eng.update_pending(edited)
        robot.hook = edit_once
        state = eng.run_blocking(first)
        self.assertEqual(state, engine.PASSED)
        snap = eng.snapshot()
        self.assertEqual([s["type"] for s in snap["steps"]], ["drive", "arm_joint", "wait"])
        self.assertEqual(arm.angles()[1], 45.0)
        self.assertTrue(any("live edit" in line["text"] for line in snap["log"]))

    def test_running_step_cannot_be_changed(self):
        eng, robot, _ = make_engine()
        first = mission_model.normalize_mission(self._doc(
            {"type": "drive", "duration": 2, "linear": 0.3}))
        changed = mission_model.normalize_mission(self._doc(
            {"type": "drive", "duration": 9, "linear": 0.3}))
        errors = []

        def try_edit():
            if not errors:
                try:
                    eng.update_pending(changed)
                except RuntimeError as exc:
                    errors.append(str(exc))
        robot.hook = try_edit
        eng.run_blocking(first)
        self.assertIn("cannot be changed", errors[0])
        self.assertEqual(eng.snapshot()["state"], engine.PASSED)

    def test_live_edit_needs_a_running_mission(self):
        eng, _, _ = make_engine()
        with self.assertRaises(RuntimeError):
            eng.update_pending(mission(steps=[{"type": "wait", "duration": 1}]))

    def test_snapshot_exposes_raw_steps_and_run_id(self):
        eng, _, _ = make_engine()
        raw = {"type": "wait", "duration": 1, "name": "pause"}
        eng.run_blocking(mission_model.normalize_mission({"name": "x", "steps": [raw]}))
        snap = eng.snapshot()
        self.assertEqual(snap["steps"][0]["raw"], raw)
        self.assertEqual(snap["run_id"], 1)
        self.assertEqual(snap["current"], 0)


class TestSaving(unittest.TestCase):
    def test_safe_filename(self):
        self.assertEqual(mission_model.safe_filename("demo"), "demo.yaml")
        self.assertEqual(mission_model.safe_filename("a_b-1.JSON"), "a_b-1.json")
        for bad in ("../evil.yaml", "a/b.yaml", "", "x y.yaml"):
            with self.assertRaises(mission_model.MissionError):
                mission_model.safe_filename(bad)

    def test_dump_and_read_round_trip(self):
        doc = {"name": "saved", "steps": [
            {"name": "drive", "type": "drive", "duration": 5, "linear": 0.2,
             "check": {"min_distance": 0.5}},
            {"type": "arm_joint", "angles": [10, None, None, None, None, None]}]}
        with tempfile.TemporaryDirectory() as folder:
            for fname in ("m.json", "m.yaml"):
                if fname.endswith(".yaml"):
                    try:
                        import yaml  # noqa: F401
                    except ImportError:
                        continue
                path = os.path.join(folder, fname)
                with open(path, "w", encoding="utf-8") as f:
                    f.write(mission_model.dump_mission(doc, fname))
                self.assertEqual(mission_model.read_mission_doc(path), doc)
                self.assertEqual(len(mission_model.load_mission(path)["steps"]), 2)


class TestArmOnRobot(unittest.TestCase):
    """Named arm poses and the 'only drive with the arm tucked in' rule."""

    def test_arm_pose_uses_named_angles(self):
        m = mission(steps=[{"type": "arm_pose", "pose": "transport"}])
        self.assertEqual(m["steps"][0]["angles"], mission_model.ARM_POSES["transport"])

    def test_unknown_pose_rejected(self):
        with self.assertRaises(mission_model.MissionError):
            mission(steps=[{"type": "arm_pose", "pose": "dance"}])

    def test_mission_own_poses(self):
        m = mission_model.normalize_mission({"name": "x", "arm_poses": {"grab": [0, 50, -60, 0, 10, 0]},
                                             "steps": [{"type": "arm_pose", "pose": "grab"}]})
        self.assertEqual(m["steps"][0]["angles"], [0, 50, -60, 0, 10, 0])
        with self.assertRaises(mission_model.MissionError):
            mission_model.normalize_mission({"name": "x", "arm_poses": {"bad": [1, 2]},
                                             "steps": [{"type": "wait", "duration": 1}]})

    def test_drive_refused_when_arm_not_tucked(self):
        eng, robot, _ = make_engine()
        m = mission_model.normalize_mission({"name": "x", "require_arm_tucked": True, "steps": [
            {"type": "drive", "duration": 2, "linear": 0.3},
            {"type": "wait", "duration": 1}]})
        self.assertEqual(eng.run_blocking(m), engine.FAILED)
        snap = eng.snapshot()
        self.assertIn("not tucked", snap["steps"][0]["detail"])
        self.assertEqual(snap["steps"][1]["status"], engine.SKIPPED)
        self.assertNotIn((0.3, 0.0), robot.cmds)  # never drove

    def test_drive_allowed_after_transport_pose(self):
        eng, robot, arm = make_engine()
        m = mission_model.normalize_mission({"name": "x", "require_arm_tucked": True, "steps": [
            {"type": "arm_pose", "pose": "transport", "check": {"tolerance_deg": 0.5}},
            {"type": "drive", "duration": 2, "linear": 0.3, "check": {"min_distance": 0.3}}]})
        self.assertEqual(eng.run_blocking(m), engine.PASSED)
        self.assertIn((0.3, 0.0), robot.cmds)

    def test_without_rule_drive_ignores_arm(self):
        eng, _, _ = make_engine()
        self.assertEqual(eng.run_blocking(mission(steps=[
            {"type": "drive", "duration": 2, "linear": 0.3}])), engine.PASSED)

    def test_sample_missions_are_valid(self):
        try:
            import yaml  # noqa: F401
        except ImportError:
            self.skipTest("PyYAML not installed on this machine")
        folder = os.path.join(ORCH, "missions")
        listed = mission_model.list_missions(folder)
        self.assertGreaterEqual(len([e for e in listed if e["file"].startswith("sample_")]), 6)
        for entry in listed:
            self.assertIsNone(entry["error"], entry["file"])

    def test_options_survive_save(self):
        doc = {"name": "x", "require_arm_tucked": True, "arm_poses": {"p": [1, 2, 3, 4, 5, 6]},
               "steps": [{"type": "arm_pose", "pose": "p"}]}
        self.assertEqual(json.loads(mission_model.dump_mission(doc, "m.json")), doc)


# ------------------------------------------------- raw steps and overrides

class TestCommandStepModel(unittest.TestCase):
    """The free-text "command" step and the raw overrides in the step model."""

    def test_command_step_needs_a_command(self):
        with self.assertRaises(mission_model.MissionError):
            mission(steps=[{"type": "command"}])
        with self.assertRaises(mission_model.MissionError):
            mission(steps=[{"type": "command", "run": "   "}])

    def test_command_step_defaults(self):
        m = mission(steps=[{"type": "command", "run": "ros2 topic list"}])
        s = m["steps"][0]
        self.assertEqual(s["run"], "ros2 topic list")
        self.assertIsNone(s["sim_run"])
        self.assertEqual(s["expect"], 0)
        self.assertEqual(s["timeout"], mission_model.COMMAND_TIMEOUT)

    def test_command_step_per_target_variant(self):
        m = mission(steps=[{"type": "command", "sim_run": "a", "real_run": "b"}])
        self.assertEqual((m["steps"][0]["sim_run"], m["steps"][0]["real_run"]), ("a", "b"))

    def test_command_expect_must_be_whole(self):
        with self.assertRaises(mission_model.MissionError):
            mission(steps=[{"type": "command", "run": "x", "expect": "yes"}])

    def test_drive_ros_override(self):
        m = mission(steps=[{"type": "drive", "duration": 1, "linear": 0.2,
                            "ros": {"topic": "/other/cmd_vel", "rate": 5,
                                    "linear": {"y": 0.1}, "angular": {"z": 0.3}}}])
        self.assertEqual(m["steps"][0]["ros"]["topic"], "/other/cmd_vel")
        self.assertEqual(m["steps"][0]["ros"]["rate"], 5.0)
        self.assertEqual(m["steps"][0]["ros"]["linear"], {"y": 0.1})

    def test_goto_ros_override(self):
        m = mission(steps=[{"type": "goto", "x": 1, "y": 2,
                            "ros": {"action": "/nav", "frame": "odom", "yaw": 0.5}}])
        self.assertEqual(m["steps"][0]["ros"]["action"], "/nav")
        self.assertEqual(m["steps"][0]["ros"]["yaw"], 0.5)

    def test_unknown_ros_option_rejected(self):
        with self.assertRaises(mission_model.MissionError):
            mission(steps=[{"type": "drive", "duration": 1, "ros": {"topicx": "/x"}}])
        with self.assertRaises(mission_model.MissionError):
            mission(steps=[{"type": "drive", "duration": 1, "ros": {"linear": {"w": 1}}}])
        with self.assertRaises(mission_model.MissionError):
            mission(steps=[{"type": "drive", "duration": 1, "ros": {"rate": 0}}])
        with self.assertRaises(mission_model.MissionError):
            mission(steps=[{"type": "goto", "x": 1, "y": 1, "ros": {"rate": 5}}])
        with self.assertRaises(mission_model.MissionError):
            mission(steps=[{"type": "wait", "duration": 1, "ros": {"topic": "/x"}}])

    def test_arm_override(self):
        m = mission(steps=[{"type": "arm_home", "arm": {"acc": 1000, "wait": True}}])
        self.assertEqual(m["steps"][0]["arm"], {"acc": 1000.0, "wait": True})
        with self.assertRaises(mission_model.MissionError):
            mission(steps=[{"type": "arm_home", "arm": {"speedx": 1}}])
        with self.assertRaises(mission_model.MissionError):
            mission(steps=[{"type": "arm_home", "arm": {"wait": "yes"}}])

    def test_raw_steps_keep_the_override_for_the_editor(self):
        raw = {"type": "drive", "duration": 1, "ros": {"topic": "/x"}}
        m = mission(steps=[raw])
        self.assertEqual(m["steps"][0]["raw"]["ros"], {"topic": "/x"})
        self.assertEqual(m["steps"][0]["raw"]["duration"], 1)


class TestTargets(unittest.TestCase):
    """targets.py: where the commands go (simulation, real robots, both)."""

    def test_sim_drives_cmd_vel_directly(self):
        cfg = targets.sim_config({})
        self.assertEqual(cfg["cmd_vel_topic"], "/robot/cmd_vel")
        self.assertIsNone(cfg["mode_service"])
        self.assertEqual(cfg["nav_kind"], "action")
        self.assertEqual(cfg["domain_id"], 0)

    def test_sim_user_mode_mirrors_the_real_robot(self):
        cfg = targets.sim_config({"SIM_DRIVE_MODE": "user"})
        self.assertEqual(cfg["cmd_vel_topic"], "/robot/cmd_vel_user")
        self.assertEqual(cfg["mode_service"], "/robot/cmd_vel_controller/set_control_mode")
        self.assertEqual(cfg["mode_reset_service"],
                         "/robot/cmd_vel_controller/reset_control_mode")
        self.assertEqual(cfg["control_mode"], 30)

    def test_real_robot_defaults(self):
        cfg = targets.real_config({})
        self.assertEqual(cfg["cmd_vel_topic"], "/robot/cmd_vel_user")
        self.assertEqual(cfg["domain_id"], 42)
        self.assertEqual(cfg["nav_kind"], "behavior")
        self.assertEqual(cfg["arm_host"], "")
        self.assertIsNone(cfg["clock_topic"])

    def test_real_robot_endpoints_come_from_the_environment(self):
        cfg = targets.real_config({"REAL_ROS_DOMAIN_ID": "7",
                                   "REAL_CMD_VEL_TOPIC": "/robot/cmd_vel_joy",
                                   "REAL_XARM_HOST": "10.0.0.5"})
        self.assertEqual(cfg["domain_id"], 7)
        self.assertEqual(cfg["cmd_vel_topic"], "/robot/cmd_vel_joy")
        self.assertEqual(cfg["arm_host"], "10.0.0.5")

    def test_configs_for(self):
        self.assertEqual([c["name"] for c in targets.configs_for("sim", {})], ["sim"])
        self.assertEqual([c["name"] for c in targets.configs_for("real", {})], ["real"])
        # the simulation comes first, so it stays the one whose feedback is checked
        self.assertEqual([c["name"] for c in targets.configs_for("both", {})], ["sim", "real"])

    def test_mission_target_is_validated_and_saved(self):
        m = mission_model.normalize_mission({"name": "x", "target": "real",
                                             "steps": [{"type": "wait", "duration": 1}]})
        self.assertEqual(m["target"], "real")
        self.assertEqual(mission_model.normalize_mission(
            {"name": "x", "steps": [{"type": "wait", "duration": 1}]})["target"], "sim")
        with self.assertRaises(mission_model.MissionError):
            mission_model.normalize_mission({"name": "x", "target": "moon",
                                             "steps": [{"type": "wait", "duration": 1}]})
        doc = {"name": "x", "target": "both", "steps": [{"type": "wait", "duration": 1}]}
        self.assertEqual(json.loads(mission_model.dump_mission(doc, "m.json")), doc)


class TestRawCommands(unittest.TestCase):
    """commands.py: the copy-ready command text behind the RAW button."""

    def setUp(self):
        self.sim = targets.sim_config({})
        self.real = targets.real_config({})

    def test_drive_shows_topic_type_and_fields(self):
        m = mission(steps=[{"type": "drive", "duration": 5, "linear": 0.25,
                            "check": {"min_distance": 0.5}}])
        block = commands.step_commands(m["steps"][0], self.sim)
        line = block["lines"][-1]
        self.assertIn("/robot/cmd_vel", line)
        self.assertIn("geometry_msgs/msg/Twist", line)
        self.assertIn("linear: {x: 0.25, y: 0, z: 0}", line)
        self.assertIn("angular: {x: 0, y: 0, z: 0}", line)
        self.assertIn("0.5 m", block["note"])

    def test_drive_on_real_takes_and_releases_control(self):
        m = mission(steps=[{"type": "drive", "duration": 5, "linear": 0.2}])
        text = "\n".join(commands.step_commands(m["steps"][0], self.real)["lines"])
        self.assertIn("/robot/cmd_vel_user", text)
        self.assertIn("origin_msgs/srv/SetControlMode", text)
        self.assertIn("{mode: {mode: 30}}", text)
        self.assertIn("origin_msgs/srv/ReturnControlMode", text)

    def test_drive_on_the_user_topic_shows_the_control_handshake(self):
        """Overriding the topic to /robot/cmd_vel_user needs control mode
        USER, also on a target that normally drives on plain cmd_vel."""
        m = mission(steps=[{"type": "drive", "duration": 5, "linear": 0.2,
                            "ros": {"topic": "/robot/cmd_vel_user"}}])
        text = "\n".join(commands.step_commands(m["steps"][0], self.sim)["lines"])
        self.assertIn("origin_msgs/srv/SetControlMode", text)
        self.assertIn("origin_msgs/srv/ReturnControlMode", text)

    def test_drive_raw_override_changes_topic_rate_and_fields(self):
        m = mission(steps=[{"type": "drive", "duration": 5, "linear": 0.2,
                            "ros": {"topic": "/other/cmd_vel", "rate": 5,
                                    "linear": {"y": 0.1}}}])
        text = "\n".join(commands.step_commands(m["steps"][0], self.sim)["lines"])
        self.assertIn("-r 5 /other/cmd_vel", text)
        self.assertIn("y: 0.1", text)

    def test_goto_on_sim_is_a_nav2_action(self):
        m = mission(steps=[{"type": "goto", "x": 2.0, "y": -1.5, "yaw": 1.5707963}])
        text = "\n".join(commands.step_commands(m["steps"][0], self.sim)["lines"])
        self.assertIn("ros2 action send_goal /navigate_to_pose", text)
        self.assertIn("nav2_msgs/action/NavigateToPose", text)
        self.assertIn("frame_id: map", text)
        self.assertIn("z: 0.707107", text)      # sin(45 deg)
        self.assertIn("w: 0.707107", text)      # cos(45 deg)

    def test_goto_on_real_is_an_autopilot_behavior(self):
        m = mission(steps=[{"type": "goto", "x": 2.0, "y": -1.5}])
        text = "\n".join(commands.step_commands(m["steps"][0], self.real)["lines"])
        self.assertIn("set_and_execute_behaviors", text)
        self.assertIn("knowledge_base_msgs/srv/SetAndExecuteBehaviors", text)
        self.assertIn("type: 10", text)         # MOVE_THROUGH_POSES

    def test_goto_waypoint_is_resolved(self):
        m = mission(steps=[{"type": "goto", "waypoint": "Home"}])
        block = commands.step_commands(m["steps"][0], self.sim,
                                       lambda name: (2.06, -1.55, 0.0))
        self.assertIn("x: 2.06", "\n".join(block["lines"]))
        self.assertIn('waypoint "Home"', block["note"])

    def test_goto_with_an_unknown_waypoint_says_so(self):
        m = mission(steps=[{"type": "goto", "waypoint": "Nowhere"}])
        block = commands.step_commands(m["steps"][0], self.sim, lambda name: None)
        self.assertIn("not in the Cerebra scene", block["note"])

    def test_arm_shows_the_sdk_call_and_the_xarm_ros2_equivalent(self):
        m = mission(steps=[{"type": "arm_joint", "joint": 1, "angle_deg": 90, "speed": 30}])
        block = commands.step_commands(m["steps"][0], self.sim)
        self.assertIn("XArmAPI(\"192.168.100.200\").set_servo_angle(", block["lines"][1])
        self.assertIn("angle=[90, None, None, None, None, None]", block["lines"][1])
        self.assertIn("speed=30", block["lines"][1])
        alt = block["alternatives"][0]["lines"][0]
        self.assertIn("ros2 service call /xarm/set_servo_angle", alt)
        self.assertIn("xarm_msgs/srv/MoveJoint", alt)
        self.assertIn("angles: [1.570796", alt)   # 90 deg in radians
        self.assertIn("speed: 0.523599", alt)     # 30 deg/s in rad/s

    def test_arm_pose_names_the_pose_angles(self):
        m = mission(steps=[{"type": "arm_pose", "pose": "transport"}])
        block = commands.step_commands(m["steps"][0], self.sim)
        self.assertIn("angle=[0, -45, -30, 0, 75, 0]", block["lines"][1])

    def test_arm_override_shows_mvacc(self):
        m = mission(steps=[{"type": "arm_home", "arm": {"acc": 500, "wait": True}}])
        block = commands.step_commands(m["steps"][0], self.sim)
        self.assertIn("mvacc=500", block["lines"][1])
        self.assertIn("wait=True", block["lines"][1])
        self.assertIn("acc: 8.726646", block["alternatives"][0]["lines"][0])

    def test_command_step_shows_its_own_text(self):
        m = mission(steps=[{"type": "command", "run": "ros2 topic list\nros2 topic list"}])
        block = commands.step_commands(m["steps"][0], self.sim)
        self.assertEqual(block["lines"], ["ros2 topic list", "ros2 topic list"])
        self.assertIn("ROS_DOMAIN_ID=0", block["note"])

    def test_mission_commands_lists_every_step_and_both_targets(self):
        m = mission(steps=[{"type": "wait", "duration": 1},
                           {"type": "drive", "duration": 1, "linear": 0.1}])
        out = commands.mission_commands(m, [self.sim, self.real])
        self.assertEqual(len(out["steps"]), 2)
        self.assertEqual([b["target"] for b in out["steps"][1]["blocks"]], ["sim", "real"])
        self.assertEqual(out["steps"][0]["blocks"][0]["lines"], [])


class FakeRunner:
    """Stands in for runner.Runner: records the commands and answers with a
    scripted exit code (or never finishes, for the timeout test)."""

    def __init__(self, code=0, output="done", never_finishes=False, on_poll=None):
        self.code = code
        self.output = output
        self.never_finishes = never_finishes
        self.on_poll = on_poll     # e.g. eng.abort, to test aborting a command
        self.started = []          # (command, domain_id)
        self.killed = 0
        self.env = None

    def start(self, command, domain_id=None):
        self.started.append((command, domain_id))
        return FakeCommand(self)


class FakeCommand:
    def __init__(self, runner):
        self._runner = runner
        self.killed = False

    def poll(self):
        if self._runner.on_poll is not None:
            self._runner.on_poll()
        return None if self._runner.never_finishes else self._runner.code

    def output(self):
        return self._runner.output

    def tail(self):
        return self._runner.output

    def kill(self):
        if not self.killed:
            self._runner.killed += 1
        self.killed = True


class TestCommandStep(unittest.TestCase):
    """The free-text command step: it runs, it is checked, and it is stopped."""

    def _engine(self, runner):
        eng, robot, arm = make_engine()
        eng.runner = runner
        return eng, robot, arm

    def test_command_step_runs_on_the_target_domain(self):
        runner = FakeRunner(code=0, output="topics listed")
        eng, _, _ = self._engine(runner)
        state = eng.run_blocking(mission(steps=[
            {"type": "command", "run": "ros2 topic list"}]))
        self.assertEqual(state, engine.PASSED)
        self.assertEqual(runner.started, [("ros2 topic list", 0)])
        self.assertIn("exit 0", eng.snapshot()["steps"][0]["detail"])
        self.assertIn("topics listed", eng.snapshot()["steps"][0]["detail"])

    def test_command_step_fails_on_another_exit_code(self):
        runner = FakeRunner(code=1, output="boom")
        eng, _, _ = self._engine(runner)
        state = eng.run_blocking(mission(steps=[
            {"type": "command", "run": "false"},
            {"type": "wait", "duration": 1}]))
        self.assertEqual(state, engine.FAILED)
        snap = eng.snapshot()
        self.assertIn("exit 1", snap["steps"][0]["detail"])
        self.assertIn("boom", snap["steps"][0]["detail"])
        self.assertEqual(snap["steps"][1]["status"], engine.SKIPPED)

    def test_command_step_expect(self):
        runner = FakeRunner(code=2)
        eng, _, _ = self._engine(runner)
        self.assertEqual(eng.run_blocking(mission(steps=[
            {"type": "command", "run": "x", "expect": 2}])), engine.PASSED)

    def test_command_step_times_out_and_is_killed(self):
        runner = FakeRunner(never_finishes=True)
        eng, _, _ = self._engine(runner)
        state = eng.run_blocking(mission(steps=[
            {"type": "command", "run": "sleep 100", "timeout": 1.0}]))
        self.assertEqual(state, engine.FAILED)
        self.assertIn("timed out", eng.snapshot()["steps"][0]["detail"])
        self.assertEqual(runner.killed, 1)

    def test_command_step_is_killed_on_abort(self):
        runner = FakeRunner(never_finishes=True)
        eng, robot, _ = self._engine(runner)
        runner.on_poll = eng.abort       # abort as soon as the command started
        state = eng.run_blocking(mission(steps=[
            {"type": "command", "run": "ros2 topic pub -r 10 /x", "timeout": 60}]))
        self.assertEqual(state, engine.ABORTED)
        self.assertEqual(runner.killed, 1)

    def test_command_step_for_another_target_is_reported(self):
        runner = FakeRunner()
        eng, _, _ = self._engine(runner)
        state = eng.run_blocking(mission(steps=[
            {"type": "command", "real_run": "ros2 topic list"}]))
        self.assertEqual(state, engine.FAILED)
        self.assertIn("no command for the current target",
                      eng.snapshot()["steps"][0]["detail"])
        self.assertEqual(runner.started, [])


class TestRawOverridesInTheEngine(unittest.TestCase):
    """The overrides reach the hardware: the shown command is the sent one."""

    def test_drive_override_publishes_on_another_topic(self):
        eng, robot, _ = make_engine()
        state = eng.run_blocking(mission(steps=[
            {"type": "drive", "duration": 2, "linear": 0.3,
             "ros": {"topic": "/robot/cmd_vel_user"}}]))
        self.assertEqual(state, engine.PASSED)
        self.assertEqual(set(robot.cmd_topics), {"/robot/cmd_vel_user"})

    def test_drive_override_sets_extra_twist_fields(self):
        eng, robot, _ = make_engine()
        eng.run_blocking(mission(steps=[
            {"type": "drive", "duration": 1, "linear": 0.2,
             "ros": {"linear": {"y": 0.5}, "angular": {"x": 0.1}}}]))
        msg = robot.twists[0][0]
        self.assertEqual(msg["linear"], {"x": 0.2, "y": 0.5, "z": 0.0})
        self.assertEqual(msg["angular"], {"x": 0.1, "y": 0.0, "z": 0.0})

    def test_drive_takes_and_releases_control(self):
        eng, robot, _ = make_engine()
        eng.run_blocking(mission(steps=[{"type": "drive", "duration": 1, "linear": 0.1}]))
        self.assertEqual(robot.control, ["take", "release"])
        self.assertIsNone(robot.control_topic)   # the target's own topic

    def test_drive_control_uses_the_overridden_topic(self):
        """A raw override can point the drive at the USER topic; the control
        handshake then has to know which topic it is for."""
        eng, robot, _ = make_engine()
        eng.run_blocking(mission(steps=[
            {"type": "drive", "duration": 1, "linear": 0.1,
             "ros": {"topic": "/robot/cmd_vel_user"}}]))
        self.assertEqual(robot.control_topic, "/robot/cmd_vel_user")

    def test_goto_override_uses_the_given_goal(self):
        eng, robot, _ = make_engine()
        eng.run_blocking(mission(steps=[
            {"type": "goto", "x": 1, "y": 2,
             "ros": {"x": 3.5, "y": -4.5, "frame": "odom", "action": "/nav"}}]))
        self.assertEqual(robot.last_nav_goal, (3.5, -4.5, 0.0))
        self.assertEqual(robot.last_nav_frame, "odom")
        self.assertEqual(robot.last_nav_action, "/nav")

    def test_arm_override_passes_acc_and_wait(self):
        eng, _, arm = make_engine()
        state = eng.run_blocking(mission(steps=[
            {"type": "arm_home", "arm": {"acc": 500, "wait": True}}]))
        self.assertEqual(state, engine.PASSED)
        self.assertEqual(arm.acc, 500.0)
        self.assertTrue(arm.wait)

    def test_arm_without_override_uses_the_sdk_defaults(self):
        eng, _, arm = make_engine()
        eng.run_blocking(mission(steps=[{"type": "arm_home"}]))
        self.assertIsNone(arm.acc)
        self.assertFalse(arm.wait)

    def test_a_mission_for_another_target_is_refused_without_links(self):
        eng, robot, _ = make_engine()
        with self.assertRaises(RuntimeError):
            eng.run_blocking(mission_model.normalize_mission({
                "name": "x", "target": "real",
                "steps": [{"type": "wait", "duration": 1}]}))
        self.assertNotIn("take", robot.control)

    def test_snapshot_reports_the_target(self):
        eng, _, _ = make_engine()
        eng.run_blocking(mission(steps=[{"type": "wait", "duration": 1}]))
        snap = eng.snapshot()
        self.assertEqual(snap["target"], "sim")
        self.assertEqual(snap["targets"], ["sim"])
        self.assertIn("target", snap["log"][0]["text"])


if __name__ == "__main__":
    unittest.main()
