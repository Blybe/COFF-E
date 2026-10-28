"""COFF-E orchestrator: the mission sequencer.

Runs the steps of a mission one by one on the robot (Origin One, ROS 2) and
the arm (xArm 6, SDK), checks every step, and keeps a live status for the
dashboard:

    1. drive forward for 60 s   -> PASS (odometry moved >= min_distance)
    2. arm joint 1 to 90 deg    -> PASS (joint angle within tolerance)
    3. drive further            -> only started because step 2 passed
    ... any FAIL stops the mission, later steps are skipped.

The sequencer only talks to the two small interfaces below (robot + arm), so
the whole state machine, including the checks and the stop-on-failure
behaviour, is unit tested with fakes - no Docker, ROS or hardware needed.

A mission also carries a *target* (sim / real / both, see targets.py): the
engine hands it to the hardware object at the start of a run and every command
then goes to the targets of that run. The raw overrides of the dashboard's raw
editor (step["ros"], step["arm"]) are applied here, so the commands the
dashboard shows are the commands that actually go out, and a "command" step is
free text that is run as a shell command (runner.py).

Robot interface (see targets.py and ros_robot.py for the real one, the unit
tests for a fake):
    wall_now()                  wall-clock seconds
    sim_seconds(), sim_fresh()  Gazebo /clock time, if the target publishes it
    odom()                      {"x", "y", "yaw", "speed", "fresh"}
    pose()                      map-frame pose (the real robot's global pose)
    publish_twist(msg, topic)   publish one Twist, any field
    publish_cmd(linear, ang)    publish a Twist with only x and z set
    stop(topic)                 publish zero velocity (safety)
    take_control()              [(target, ok, message)] - USER mode on the real
                                robot; release_control() gives it back
    odom_topic(), nav_description()   for the messages in the log
    nav_ready(action)           True if the navigation of the target is there
    nav_send(x, y, yaw, ...)    goal wrapper (Nav2 action or autopilot behavior)
    nav_status(handle)          (state, distance_remaining_m); state is one
                                of active/succeeded/failed/cancelled/rejected
    nav_cancel(handle)          cancel a running goal
    nav_error()                 why the last navigation goal was refused
    waypoint(name)              (x, y, yaw) of a Cerebra waypoint, or None

Arm interface (see arm_control.py):
    angles()                    list of 6 joint angles in degrees, or None
    angles_all()                [(target, angles)] for every arm of the run
    command_joints(angles, speed, acc, wait)   {target: SDK code}, 0 = accepted;
                                a None angle keeps that arm's own joint angle
    arm_faults()                {target: controller error} (optional)
"""

import collections
import math
import threading
import time

import commands
import mission_model
from runner import Runner

IDLE = "idle"
RUNNING = "running"
PASSED = "passed"
FAILED = "failed"
ABORTED = "aborted"

# Status values of a single step.
PENDING = "pending"
STEP_RUNNING = "running"
STEP_PASSED = "passed"
STEP_FAILED = "failed"
SKIPPED = "skipped"
STEP_ABORTED = "aborted"


class StepAborted(Exception):
    """Raised inside a step when the user pressed Abort."""


class SingleTarget:
    """Target information for an engine built without a targets.Hardware:
    one implicit simulation target, no ROS domains to switch. The unit tests
    use this; the server always passes the real Hardware object."""

    target = mission_model.TARGETS[0]

    def names(self):
        return [self.target]

    def label(self):
        return "Simulation"

    def domain_for(self, name):
        return 0

    def use(self, target):
        if target not in mission_model.TARGETS:
            raise RuntimeError(f"unknown target {target!r}")
        if target != self.target:
            raise RuntimeError("this orchestrator has no links for target "
                               f"{target!r} - only the simulation")
        return self

    def snapshot(self):
        return {}


class MissionEngine:
    """Runs one mission at a time; thread-safe; start()/abort()/snapshot()."""

    def __init__(self, robot, arm, sleep=time.sleep, log_size=400, hw=None, runner=None):
        self.robot = robot
        self.arm = arm
        # hw = the targets.Hardware of this orchestrator (which targets exist,
        # which ROS domain they use). The engine only needs it for that.
        self.hw = hw if hw is not None else SingleTarget()
        self.runner = runner if runner is not None else Runner()
        self._running_cmd = None
        self._sleep = sleep  # injectable for unit tests
        self._lock = threading.RLock()
        self._log = collections.deque(maxlen=log_size)
        self._abort = threading.Event()
        self._thread = None
        self._state = IDLE
        self._mission = None      # normalized mission dict or None
        self._mission_file = ""
        self._steps = []         # per-step status dicts (for the dashboard)
        self._started_wall = 0.0
        self._ended_wall = 0.0
        self._reason = ""
        self._run_id = 0         # increases with every start; lets the dashboard
                                 # tell its own run apart from an older one
        self._current = -1       # index of the step that runs (or just ran)

    # ------------------------------------------------------------- control

    def _begin(self, mission, mission_file):
        if self._state == RUNNING:
            raise RuntimeError("a mission is already running")
        self._mission = mission
        self._mission_file = mission_file
        self._steps = [self._blank_step(step) for step in mission["steps"]]
        self._reason = ""
        self._abort.clear()
        self._state = RUNNING
        self._run_id += 1
        self._current = -1
        self._started_wall = self.robot.wall_now()
        self._ended_wall = 0.0
        self.hw.use(mission.get("target") or mission_model.TARGETS[0])
        self._say(f"mission start: {mission['name']} ({len(mission['steps'])} steps, "
                  f"clock: {mission['clock']}, target: {self.hw.label()} -> "
                  f"{', '.join(self.hw.names())})")

    def start(self, mission, mission_file=""):
        """Start a mission in the background. Raises RuntimeError if busy,
        so the server can answer with a clear error message."""
        with self._lock:
            self._begin(mission, mission_file)
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
            return self._run_id

    def run_blocking(self, mission, mission_file=""):
        """Run a mission in the calling thread (used by the unit tests)."""
        with self._lock:
            self._begin(mission, mission_file)
        self._run()
        return self._state

    def update_pending(self, mission):
        """Live edit: replace every step that has not started yet.

        `mission` is the full edited mission. Steps that already ran, and
        the one running now, must be unchanged in it; everything after them
        replaces the remaining plan. Raises RuntimeError with a message for
        the user when that is not possible."""
        with self._lock:
            if self._state != RUNNING:
                raise RuntimeError("no mission is running - just start the edited one")
            keep = self._current + 1
            old = self._mission["steps"]
            new = mission["steps"]
            if len(new) < keep:
                raise RuntimeError("the edit removes steps that already ran")
            for j in range(keep):
                if old[j]["raw"] != new[j]["raw"]:
                    raise RuntimeError(f"step {j + 1} already ran or is running "
                                       "and cannot be changed any more")
            self._mission["steps"] = old[:keep] + new[keep:]
            for key in ("require_arm_tucked", "poses", "arm_poses_doc"):
                if key in mission:
                    self._mission[key] = mission[key]
            self._steps = self._steps[:keep] + [self._blank_step(s) for s in new[keep:]]
            self._say(f"live edit: steps {keep + 1}..{len(new)} updated "
                      f"({len(new)} steps in total now)")

    def abort(self):
        """Ask the running mission to stop at the next check."""
        self._abort.set()

    # ------------------------------------------------------------- status

    def snapshot(self):
        with self._lock:
            elapsed = 0.0
            if self._started_wall:
                elapsed = (self._ended_wall or self.robot.wall_now()) - self._started_wall
            return {
                "state": self._state,
                "reason": self._reason,
                "mission_name": self._mission["name"] if self._mission else None,
                "mission_file": self._mission_file,
                "clock": self._mission["clock"] if self._mission else None,
                "target": self._mission["target"] if self._mission else self.hw.target,
                "targets": self.hw.names(),
                "target_label": self.hw.label(),
                "options": {
                    "require_arm_tucked": self._mission.get("require_arm_tucked", False),
                    "arm_poses": self._mission.get("arm_poses_doc") or {},
                } if self._mission else {},
                "started_wall": self._started_wall,
                "run_id": self._run_id,
                "current": self._current,
                "elapsed_s": elapsed,
                "steps": [dict(s) for s in self._steps],
                "log": list(self._log),
            }

    # ------------------------------------------------------------- helpers

    def _say(self, text, level="info"):
        stamp = time.strftime("%H:%M:%S")
        with self._lock:
            self._log.append({"t": stamp, "level": level, "text": text})

    @staticmethod
    def _blank_step(step):
        return {
            "name": step["name"],
            "type": step["type"],
            "status": PENDING,
            "detail": "",
            "progress": 0.0,
            "duration_s": None,
            "started_wall": 0.0,
            "ended_wall": 0.0,
            "targets": None,  # arm steps: the 6 target angles
            "raw": step.get("raw"),
        }

    def _now(self):
        """Mission clock: Gazebo time when selected and fresh, else wall."""
        if self._mission["clock"] == "sim" and self.robot.sim_fresh():
            return self.robot.sim_seconds()
        return self.robot.wall_now()

    def _set_step(self, index, **fields):
        with self._lock:
            self._steps[index].update(fields)

    def _progress(self, index, done, total):
        self._set_step(index, progress=max(0.0, min(1.0, done / total if total else 1.0)))

    def _pump(self, index, deadline, abort_nav=None, interval=0.1):
        """Sleep a bit while checking abort and timeout. Returns True when
        the step should keep going. Raises StepAborted on abort. Does not
        touch the step progress: each step computes its own."""
        if self._abort.is_set():
            if abort_nav is not None:
                self.robot.nav_cancel(abort_nav)
            raise StepAborted("aborted by the user")
        if self.robot.wall_now() > deadline:
            if abort_nav is not None:
                self.robot.nav_cancel(abort_nav)
            return False
        self._sleep(interval)
        return True

    # ------------------------------------------------------------- runner

    def _run(self):
        mission = self._mission
        i = 0
        outcome = PASSED
        try:
            # A while loop instead of for: update_pending() may replace the
            # steps after the current one while the mission runs.
            while True:
                with self._lock:
                    if i >= len(mission["steps"]):
                        break
                    step = mission["steps"][i]
                    total = len(mission["steps"])
                    self._current = i
                if self._abort.is_set():
                    self._skip_rest(i, "aborted")
                    outcome = ABORTED
                    self._reason = "mission aborted by the user"
                    break
                self._set_step(i, status=STEP_RUNNING, started_wall=self.robot.wall_now())
                self._say(f"step {i + 1}/{total} start: {step['name']}")
                ok, detail, duration = self._run_step(i, step)
                ended = self.robot.wall_now()
                if isinstance(ok, StepAborted):
                    self._set_step(i, status=STEP_ABORTED, detail=str(ok.args[0]),
                                   ended_wall=ended, duration_s=ended - duration)
                    self._skip_rest(i + 1, "aborted")
                    outcome = ABORTED
                    self._reason = f"aborted during step {i + 1} ({step['name']})"
                    break
                self._set_step(i, status=STEP_PASSED if ok else STEP_FAILED,
                               detail=detail, ended_wall=ended, duration_s=ended - duration)
                if ok:
                    self._say(f"step {i + 1}/{total} PASS: {detail}", "pass")
                    i += 1
                    continue
                self._skip_rest(i + 1, "skipped after failure")
                outcome = FAILED
                self._reason = f'step {i + 1} ("{step["name"]}") failed: {detail}'
                self._say(self._reason, "fail")
                break
            self._state = outcome
            if outcome == PASSED:
                self._say(f"mission PASSED: {mission['name']}", "pass")
        except Exception as exc:  # never let the worker thread die silently
            outcome = FAILED
            self._state = FAILED
            self._reason = f"internal error: {exc}"
            self._say(self._reason, "fail")
        finally:
            self._ended_wall = self.robot.wall_now()
            self.robot.stop()  # safety: stop driving no matter what happened
            self._say(f"mission {self._state} after "
                      f"{self._ended_wall - self._started_wall:.0f} s (wall clock)",
                      "pass" if self._state == PASSED else "fail")

    def _skip_rest(self, from_index, status):
        with self._lock:
            count = len(self._steps)
        for j in range(from_index, count):
            self._set_step(j, status=SKIPPED if status == "skipped after failure" else status,
                           detail="not started")

    def _run_step(self, i, step):
        """Runs one step, returns (ok|StepAborted, detail, started_wall)."""
        started = self.robot.wall_now()
        deadline = self._deadline(step)
        step["_budget"] = deadline - started  # for the "timed out" messages
        runner = {
            "drive": self._step_drive,
            "wait": self._step_wait,
            "goto": self._step_goto,
            "arm_joint": self._step_arm,
            "arm_home": self._step_arm,
            "arm_pose": self._step_arm,
            "command": self._step_command,
        }[step["type"]]
        try:
            if step["type"] in ("drive", "goto") and self._mission.get("require_arm_tucked"):
                ok, detail = self._check_tucked()
                if not ok:
                    return False, detail, started
            ok, detail = runner(i, step, deadline)
            return ok, detail, started
        except StepAborted as exc:
            return exc, str(exc.args[0]), started

    def _check_tucked(self):
        """Safety rule for the arm mounted on the robot: only drive when the
        arm is folded into the transport pose."""
        pose = self._mission.get("poses", mission_model.ARM_POSES)[mission_model.TUCK_POSE]
        angles = self.arm.angles()
        if angles is None:
            return False, "safety: cannot see the arm, so the robot will not drive"
        gap = max(abs(a - p) for a, p in zip(angles, pose))
        if gap > mission_model.TUCK_TOLERANCE_DEG:
            return False, (f"safety: the arm is not tucked in ({gap:.0f} deg away from the "
                           f"'{mission_model.TUCK_POSE}' pose), so the robot will not drive. "
                           f"Add an arm_pose 'transport' step before driving.")
        return True, ""

    def _deadline(self, step):
        """Wall-clock deadline for a step. An explicit timeout from the
        mission wins; timed steps (drive/wait) otherwise get one that adapts
        to the measured simulation speed, so a mission keeps working when the
        sim runs at 0.5x, 0.2x or 0.1x real time."""
        now = self.robot.wall_now()
        if step.get("timeout") is not None:
            return now + step["timeout"]
        if step["type"] in ("drive", "wait"):
            # 0 = no /clock at all (the real robot): assume real time.
            factor = getattr(self.robot, "time_factor", lambda: 1.0)() or 1.0
            factor = min(max(factor, 0.05), 1.0)  # sane bounds
            return now + step["duration"] / factor * 1.5 + 60.0
        return now + (240 if step["type"].startswith("arm") else 900)

    # ------------------------------------------------------------- steps

    def _step_drive(self, i, step, deadline):
        """Publish cmd_vel for <duration> mission-clock seconds, then check
        the odometry displacement against check.min_distance."""
        linear, angular = commands.twist_fields(step)
        ros = step.get("ros") or {}
        topic = ros.get("topic")
        rate = ros.get("rate") or mission_model.CMD_RATE
        start = self.robot.odom()
        if not start["fresh"]:
            # Give the robot a moment to publish its odometry before starting.
            while not start["fresh"] and self.robot.wall_now() < deadline:
                self._pump(i, deadline)
                start = self.robot.odom()
            if not start["fresh"]:
                return False, (f"no odometry on {self.robot.odom_topic()} - "
                               f"is {self.hw.label()} running?")

        taken, message = self._take_control(topic)
        if not taken:
            return False, f"cannot take control of the robot: {message}"
        try:
            ok, detail = self._drive_loop(i, step, deadline, start, topic,
                                          linear, angular, rate)
        finally:
            # Always stop driving again, like letting go of the joystick.
            self._stop_cmd(topic)
            self._release_control(topic)
        if not ok:
            return False, detail
        return self._check_distance(start, step)

    def _drive_loop(self, i, step, deadline, start, topic, linear, angular, rate):
        """Publish the drive message (raw override included) until the
        duration is over."""
        duration = step["duration"]
        message = {"linear": linear, "angular": angular}
        t0 = self._now()
        while True:
            self.robot.publish_twist(message, topic=topic)
            elapsed = self._now() - t0
            odom = self.robot.odom()
            dist = math.hypot(odom["x"] - start["x"], odom["y"] - start["y"])
            self._set_step(i, duration_s=elapsed,
                           detail=f"{elapsed:.0f}/{duration:.0f} s | {dist:.2f} m driven")
            self._progress(i, elapsed, duration)
            if elapsed >= duration:
                return True, f"drove for {elapsed:.0f} s"
            if not self._pump(i, deadline, interval=1.0 / float(rate)):
                return False, (f"timed out after {step['_budget']:.0f} s "
                               f"(odometry last moved {dist:.2f} m)")

    def _check_distance(self, start, step):
        odom = self.robot.odom()
        dist = math.hypot(odom["x"] - start["x"], odom["y"] - start["y"])
        need = step["min_distance"]
        if not odom["fresh"]:
            return False, "no odometry on /robot/odom - cannot verify the robot moved"
        if need is None:
            return True, f"moved {dist:.2f} m"
        if dist < need:
            return False, f"only moved {dist:.2f} m, need {need} m (blocked or too slow?)"
        return True, f"moved {dist:.2f} m (>= {need} m)"

    def _stop_cmd(self, topic=None):
        for _ in range(5):  # a few zero messages, 10 Hz for half a second
            self.robot.publish_cmd(0.0, 0.0, topic=topic)
            self._sleep(0.1)

    def _step_wait(self, i, step, deadline):
        t0 = self._now()
        while True:
            left = step["duration"] - (self._now() - t0)
            self._set_step(i, duration_s=self._now() - t0, detail=f"{left:.0f} s left")
            self._progress(i, self._now() - t0, step["duration"])
            if left <= 0:
                return True, f"waited {step['duration']:.0f} s"
            if not self._pump(i, deadline):
                return False, f"timed out after {step['_budget']:.0f} s"

    def _step_goto(self, i, step, deadline):
        """Navigate to a Cerebra waypoint or map coordinate: Nav2 on the
        simulation, an autopilot behavior on the real robot (targets.py)."""
        if step["waypoint"]:
            wp = self.robot.waypoint(step["waypoint"])
            if wp is None:
                return False, f'waypoint "{step["waypoint"]}" not found; available: ' \
                             f'{", ".join(self.robot.waypoints()) or "none"}'
            x, y, yaw = wp
        else:
            x, y, yaw = step["x"], step["y"], step["yaw"]
        ros = step.get("ros") or {}
        x, y, yaw = ros.get("x", x), ros.get("y", y), ros.get("yaw", yaw)
        action = ros.get("action")
        if not self.robot.nav_ready(action):
            return False, (f"{self.robot.nav_description()} is not reachable on "
                           f"{self.hw.label()} - for the simulation: is the "
                           f"origin-nav container running?")
        # Navigating is the autopilot's job: never hold the robot's velocity
        # control (USER mode) while a route is being planned and driven.
        self._release_control()
        goal = self.robot.nav_send(x, y, yaw, frame=ros.get("frame"), action=action)
        if goal is None:
            return False, (self.robot.nav_error()
                           or f"{self.hw.label()} rejected the goal")
        first_dist = None
        while True:
            state, dist = self.robot.nav_status(goal)
            if first_dist is None and dist:
                first_dist = dist
            detail = "waiting for the planner..."
            if state == "active":
                if dist is not None:
                    done = 1.0 - dist / first_dist if first_dist else 0.0
                    self._progress(i, done, 1.0)
                    detail = f"{dist:.1f} m to go"
            self._set_step(i, detail=detail)
            if state == "succeeded":
                return True, f"arrived at ({x:.2f}, {y:.2f})"
            if state in ("failed", "aborted"):
                return False, (f"navigation {state} - no route to "
                               f"({x:.2f}, {y:.2f}) or the goal was unreachable")
            if state == "cancelled":
                return False, "the navigation goal was cancelled"
            if state == "rejected":
                return False, "the navigation goal was rejected (inside a no-go zone?)"
            if not self._pump(i, deadline, abort_nav=goal):
                if dist is not None:
                    return False, (f"timed out after {step['_budget']:.0f} s, "
                                   f"{dist:.1f} m to go")
                return False, f"timed out after {step['_budget']:.0f} s"
            self._sleep(0.15)  # Nav2 feedback is slow; poll at ~4 Hz

    def _step_arm(self, i, step, deadline):
        """Move the arm joints and check the reached angles."""
        if not self._wait_for_arm(i, deadline):
            return False, ("arm not connected or no feedback yet - is the "
                           "ufactory-studio container running (see the Arm panel)?")
        current = self.arm.angles()
        target = [t if t is not None else current[j] for j, t in enumerate(step["angles"])]
        moving = [j for j in range(len(target))
                  if step["angles"][j] is not None]
        raw = step.get("arm") or {}
        # One arm in an error state (an emergency stop on the real arm) must
        # stop the step before any arm moves, or a "both" run leaves the
        # simulated arm half-way while the real one never started.
        faults = self.arm.arm_faults() if hasattr(self.arm, "arm_faults") else {}
        if faults:
            listed = "; ".join(f"{name}: {text}" for name, text in sorted(faults.items()))
            return False, (f"xArm in an error state, nothing was moved - {listed}. "
                           "Fix the cause, then clear the error in UFACTORY Studio")
        self._set_step(i, targets=target)
        codes = self.arm.command_joints(step["angles"], step["speed"],
                                        acc=raw.get("acc"), wait=bool(raw.get("wait")))
        refused = sorted(f"{name} (SDK code {code})"
                         for name, code in codes.items() if code != 0)
        if refused:
            return False, f"xArm rejected the command: {', '.join(refused)}"
        gap0 = max(abs(target[j] - current[j]) for j in moving) if moving else 0.0
        tol = step["tolerance_deg"]
        while True:
            # Every arm of the run is checked, so a mission on "both" fails
            # when the real arm did not reach the pose either.
            gaps = {}
            reached = {}
            for name, angles in self.arm.angles_all():
                if angles is None:
                    continue
                reached[name] = angles
                gaps[name] = max((abs(angles[j] - target[j]) for j in moving), default=0.0)
            if not gaps:
                return False, "lost the arm connection while moving"
            gap = max(gaps.values())
            where = (" [" + ", ".join(f"{n} {g:.1f} deg" for n, g in sorted(gaps.items())) + "]"
                     if len(gaps) > 1 else "")
            self._set_step(i, detail=f"within {gap:.1f} deg of target "
                                     f"(tolerance {tol:.0f} deg){where}")
            self._progress(i, gap0 - gap, gap0)
            if gap <= tol:
                primary = self.hw.names()[0]
                angles = reached.get(primary, next(iter(reached.values())))
                moved = ", ".join(f"J{j + 1}={angles[j]:.1f}" for j in moving)
                return True, f"reached {moved} deg (within {tol:.0f} deg){where}"
            if not self._pump(i, deadline):
                return False, (f"timed out after {step['_budget']:.0f} s, "
                               f"still {gap:.1f} deg from the target")
            self._sleep(0.1)

    def _wait_for_arm(self, i, deadline):
        """Wait a moment for the arm monitor thread to produce feedback."""
        while self.arm.angles() is None:
            if not self._pump(i, deadline):
                return False
        return True

    # ------------------------------------------------------- raw commands

    def _target_runs(self, step):
        """[(target, command line)] of a command step: the per-target variant
        when the step has one, otherwise the shared `run`."""
        runs = []
        for name in self.hw.names():
            line = step.get(f"{name}_run") or step.get("run")
            if line:
                runs.append((name, line))
        return runs

    def _run_command(self, i, step, name, line, deadline):
        """Run one raw command: (exit code, output), exit code None on
        timeout. The command is killed when the step is aborted or when the
        step is over, so nothing keeps publishing after the mission stopped."""
        self._say(f"[{name}] $ {line}")
        command = self.runner.start(line, domain_id=self.hw.domain_for(name))
        self._running_cmd = command
        try:
            while command.poll() is None:
                if not self._pump(i, deadline):
                    return None, command.output()
                self._set_step(i, detail=f"[{name}] running: {line.splitlines()[0][:70]}")
            return command.poll(), command.output()
        finally:
            command.kill()          # a no-op when it already exited
            self._running_cmd = None

    def _step_command(self, i, step, deadline):
        """Run a free-text command, once per target of the run."""
        runs = self._target_runs(step)
        if not runs:
            return False, "this command step has no command for the current target"
        expect = step.get("expect", 0)
        done = []
        for name, line in runs:
            code, output = self._run_command(i, step, name, line, deadline)
            tail = output.strip().splitlines()[-1][:120] if output.strip() else ""
            if code is None:
                return False, (f"[{name}] timed out after {step['_budget']:.0f} s"
                               + (f": {tail}" if tail else ""))
            if code != expect:
                return False, (f"[{name}] exit {code}, expected {expect}"
                               + (f": {tail}" if tail else ": no output"))
            done.append(f"[{name}] exit {code}" + (f" ({tail})" if tail else ""))
        return True, "; ".join(done)

    # ------------------------------------------------------------- control

    def _take_control(self, topic=None):
        """Take over the mobile base where the target needs it: the real
        Origin One only drives from /robot/cmd_vel_user in USER mode."""
        results = self.robot.take_control(topic=topic)
        bad = [f"{name}: {message}" for name, ok, message in results if not ok]
        if bad:
            return False, "; ".join(bad)
        return True, "; ".join(message for _, _, message in results if message)

    def _release_control(self, topic=None):
        for name, ok, message in self.robot.release_control(topic=topic):
            if not ok:
                self._say(f"could not give control back on {name}: {message}", "fail")
