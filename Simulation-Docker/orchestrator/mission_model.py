"""COFF-E orchestrator: mission files.

A mission is a YAML (or JSON) file that describes what the robot and the arm
should do, one step after another:

    drive 60 s -> move the arm 90 degrees -> drive further -> ...

The engine runs the steps in order and CHECKS every one of them: a step has
to actually achieve what it says within its timeout, otherwise the mission
stops with FAILED and no further step runs.

Step types:

  drive      publish /robot/cmd_vel for <duration> seconds, then check that
             the odometry moved at least <min_distance> metres
  goto       drive to a Cerebra waypoint (by name) or to map coordinates
             with Nav2 (the planner of the origin-nav container)
  arm_joint  move the xArm 6 to <angles> (6 values in degrees) or a single
             <joint> to <angle_deg>, then check the reached angles
  arm_home   shortcut for arm_joint with all joints at 0 degrees (simulation
             only: the real xArm has a gripper at the front and cannot take
             the all-zero position, so a mission on the real robots parks in
             the "standard" pose instead)
  arm_pose   move the xArm 6 to a named pose (see ARM_POSES, or the
             mission's own arm_poses), e.g. "transport" before driving
  wait       do nothing for <duration> seconds
  command    run a raw shell command in the orchestrator container (ROS 2 is
             sourced first), e.g. a `ros2 topic pub` or a service call that
             has no step type of its own. run = both targets, or sim_run /
             real_run for a per-target variant; expect = the exit code that
             counts as success (0)

Raw overrides (the dashboard's raw editor): a drive step takes
ros: {topic, rate, linear: {x, y, z}, angular: {x, y, z}} and a goto step takes
ros: {action, frame, x, y, z, yaw} - the message the orchestrator sends, with
the fields it actually publishes. An arm step takes arm: {acc, wait} (the raw
xArm SDK options next to the structured angles and speed). The message *type*
is fixed: the orchestrator publishes typed messages, so use a command step to
send anything else.

Mission options:

  target:              sim | real | both - where the mission runs (default
                       sim; see targets.py). "real" drives the real Avular
                       Origin One (control mode USER + /robot/cmd_vel_user)
                       and the real xArm, "both" sends everything to both and
                       checks the simulation
  arm_poses:           {name: [6 angles]} - extra or changed named poses
  require_arm_tucked:  true - every drive/goto step first checks that the
                       arm is in the "transport" pose, and fails otherwise
                       (never drive around with the arm sticking out)

Example:

    name: "COFF-E demo"
    steps:
      - name: "Drive forward for 60 s"
        type: drive
        duration: 60
        linear: 0.25
        check: {min_distance: 2.0}
      - name: "Arm: joint 1 to 90 degrees"
        type: arm_joint
        joint: 1
        angle_deg: 90
        speed: 30

Pure Python on purpose (PyYAML is imported lazily, only for .yaml files) so
this module can be unit tested on the host without Docker, ROS or the SDK.
"""

import copy
import json
import os
import re

STEP_TYPES = ("drive", "goto", "arm_joint", "arm_home", "arm_pose", "wait", "command")

# Where a mission runs: the simulation, the real robots, or both (targets.py).
# "sim" is the default, so a mission file stays portable.
TARGETS = ("sim", "real", "both")

CMD_RATE = 10.0          # Hz drive steps publish at, unless the step overrides it
COMMAND_TIMEOUT = 30.0   # s a "command" step may take, unless it sets its own

# Raw overrides: the fields of the message the orchestrator sends, which the
# dashboard step editor can change without leaving the step model.
ROS_OVERRIDE_KEYS = {
    "drive": ("topic", "rate", "linear", "angular"),
    "goto": ("action", "frame", "x", "y", "z", "yaw"),
}
ARM_OVERRIDE_KEYS = ("acc", "wait")

ARM_JOINTS = 6  # xArm 6

# Named xArm 6 joint poses (degrees, J1..J6). The poses below "standard" are
# checked on the simulated arm; J1 turns the whole arm around its base, so
# with the arm mounted on the Origin One, J1 = 0 points forward and +/-90 to
# the sides. "standard" is the REAL xArm's initial position: it has a gripper
# mounted at the front, so the all-zero "home" is not a position it can take,
# and a mission that runs on the real arm parks and drives in "standard"
# instead (normalize_mission refuses all-zero arm steps on target real/both).
ARM_POSES = {
    "home": [0, 0, 0, 0, 0, 0],             # straight up (simulation only)
    "standard": [0, -90, 0, 0, 90, 0],      # the real xArm's initial position
    "transport": [0, -45, -30, 0, 75, 0],   # folded back over the robot, for driving
    "reach_front": [0, 30, -40, 0, 10, 0],  # reach out in front (serve / pick)
    "serve_left": [90, 20, -50, 0, 30, 0],  # hand something to the left side
    "serve_right": [-90, 20, -50, 0, 30, 0],
    "look_down": [0, 10, -90, 0, 80, 0],    # wrist camera looking at the floor
    "wave_up": [0, -20, -120, 0, 40, 0],    # high up, for waving / showing
}
# Shown next to the pose names in the dashboard step editor.
ARM_POSE_INFO = {
    "home": "straight up (simulation only - the real arm has a front gripper)",
    "standard": "the real xArm's initial position: J2 -90, J5 90",
    "transport": "folded back over the robot - the only pose to drive in",
    "reach_front": "reach out in front of the robot (serve / pick up)",
    "serve_left": "turned to the left side, arm out",
    "serve_right": "turned to the right side, arm out",
    "look_down": "wrist (camera) pointing at the floor in front",
    "wave_up": "high up, to wave or show something",
}
TUCK_POSE = "transport"
TUCK_TOLERANCE_DEG = 5.0


class MissionError(ValueError):
    """A mission file is invalid; the message is shown to the user."""


def _step_where(step, index):
    name = step.get("name") or step.get("type", "?")
    return f'step {index + 1} ("{name}")'


def _number(value, what, ctx, allow_none=False):
    if value is None and allow_none:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MissionError(f"{ctx}: {what} must be a number")
    return float(value)


def _check_unknown_keys(step, known, ctx):
    unknown = sorted(set(step) - set(known))
    if unknown:
        raise MissionError(f"{ctx}: unknown option(s) {', '.join(unknown)}")


def _ros_override(raw, ctx, stype):
    """The raw ROS override of a step, or None when it has none.

    drive:  {topic, rate, linear: {x, y, z}, angular: {x, y, z}}
    goto:   {action, frame, x, y, z, yaw}
    """
    ros = raw.get("ros")
    if ros is None:
        return None
    if not isinstance(ros, dict):
        raise MissionError(f"{ctx}: ros must be a mapping of raw message fields")
    unknown = sorted(set(ros) - set(ROS_OVERRIDE_KEYS[stype]))
    if unknown:
        raise MissionError(f"{ctx}: unknown ros option(s) {', '.join(unknown)}")
    out = {}
    for key in ("topic", "action", "frame"):
        if key in ros:
            value = ros[key]
            if not isinstance(value, str) or not value.strip():
                raise MissionError(f"{ctx}: ros.{key} must be a non-empty string")
            out[key] = value.strip()
    if "rate" in ros:
        out["rate"] = _number(ros["rate"], "ros.rate", ctx)
        if out["rate"] <= 0:
            raise MissionError(f"{ctx}: ros.rate must be > 0")
    for key in ("x", "y", "z", "yaw"):
        if key in ros:
            out[key] = _number(ros[key], f"ros.{key}", ctx)
    for key in ("linear", "angular"):
        if key in ros:
            vector = ros[key]
            if not isinstance(vector, dict):
                raise MissionError(f"{ctx}: ros.{key} must be a mapping of x, y and z")
            bad = sorted(set(vector) - {"x", "y", "z"})
            if bad:
                raise MissionError(f"{ctx}: unknown ros.{key} field(s) {', '.join(bad)}")
            out[key] = {k: _number(v, f"ros.{key}.{k}", ctx) for k, v in vector.items()}
    return out


def _arm_override(raw, ctx):
    """The raw xArm SDK override of an arm step: {acc, wait}. The angles and
    the speed are structured fields of the step itself."""
    arm = raw.get("arm")
    if arm is None:
        return None
    if not isinstance(arm, dict):
        raise MissionError(f"{ctx}: arm must be a mapping of SDK options")
    unknown = sorted(set(arm) - set(ARM_OVERRIDE_KEYS))
    if unknown:
        raise MissionError(f"{ctx}: unknown arm option(s) {', '.join(unknown)}")
    out = {}
    if "acc" in arm:
        out["acc"] = _number(arm["acc"], "arm.acc", ctx)
        if out["acc"] <= 0:
            raise MissionError(f"{ctx}: arm.acc must be > 0")
    if "wait" in arm:
        if not isinstance(arm["wait"], bool):
            raise MissionError(f"{ctx}: arm.wait must be true or false")
        out["wait"] = arm["wait"]
    return out


def _normalize_step(raw, index, poses=None):
    step = _normalize_step_fields(raw, index, poses if poses is not None else ARM_POSES)
    # The step exactly as written, so the dashboard editor can show and edit
    # it, and live edits can tell which steps were left unchanged.
    step["raw"] = copy.deepcopy(raw)
    return step


def _normalize_step_fields(raw, index, poses):
    if not isinstance(raw, dict):
        raise MissionError(f"step {index + 1}: must be a mapping, got {type(raw).__name__}")

    stype = raw.get("type")
    if stype not in STEP_TYPES:
        raise MissionError(
            f"step {index + 1}: type must be one of {', '.join(STEP_TYPES)}, got {stype!r}")
    step = {
        "type": stype,
        "name": str(raw.get("name") or stype),
        "timeout": _number(raw.get("timeout"), "timeout", f"step {index + 1}", allow_none=True),
    }
    ctx = _step_where(raw, index)

    if stype == "drive" or stype == "wait":
        _check_unknown_keys(raw, ("type", "name", "timeout", "duration", "linear", "angular",
                                  "check", "ros"), ctx)
        step["duration"] = _number(raw.get("duration"), "duration", ctx)
        if step["duration"] <= 0:
            raise MissionError(f"{ctx}: duration must be > 0")
        if stype == "wait":
            if raw.get("ros") is not None:
                raise MissionError(f"{ctx}: a wait step sends no ROS message, so it has "
                                   "no ros override")
            return step
        step["ros"] = _ros_override(raw, ctx, stype)
        step["linear"] = _number(raw.get("linear", 0.0), "linear", ctx)
        step["angular"] = _number(raw.get("angular", 0.0), "angular", ctx)
        check = raw.get("check") or {}
        if not isinstance(check, dict):
            raise MissionError(f"{ctx}: check must be a mapping")
        unknown = sorted(set(check) - {"min_distance"})
        if unknown:
            raise MissionError(f"{ctx}: unknown check option(s) {', '.join(unknown)}")
        step["min_distance"] = _number(
            check.get("min_distance"), "check.min_distance", ctx, allow_none=True)
        if step["min_distance"] is not None and step["min_distance"] < 0:
            raise MissionError(f"{ctx}: check.min_distance must be >= 0")

    elif stype == "goto":
        _check_unknown_keys(raw, ("type", "name", "timeout", "waypoint", "x", "y", "yaw",
                                  "ros"), ctx)
        step["ros"] = _ros_override(raw, ctx, stype)
        waypoint = raw.get("waypoint")
        if waypoint is not None:
            if not isinstance(waypoint, str):
                raise MissionError(f"{ctx}: waypoint must be a name (string)")
            if raw.get("x") is not None or raw.get("y") is not None:
                raise MissionError(f"{ctx}: use either waypoint or x/y, not both")
            step["waypoint"] = waypoint
            step["x"] = step["y"] = step["yaw"] = None
        else:
            step["waypoint"] = None
            step["x"] = _number(raw.get("x"), "x", ctx)
            step["y"] = _number(raw.get("y"), "y", ctx)
            if step["x"] is None or step["y"] is None:
                raise MissionError(f"{ctx}: goto needs either a waypoint or x and y")
            step["yaw"] = _number(raw.get("yaw", 0.0), "yaw", ctx)

    elif stype in ("arm_joint", "arm_home", "arm_pose"):
        _check_unknown_keys(
            raw, ("type", "name", "timeout", "angles", "joint", "angle_deg", "speed",
                  "check", "pose", "arm"), ctx)
        step["arm"] = _arm_override(raw, ctx)
        if stype != "arm_pose" and raw.get("pose") is not None:
            raise MissionError(f"{ctx}: pose only belongs to an arm_pose step")
        step["speed"] = _number(raw.get("speed", 30), "speed", ctx)
        if step["speed"] <= 0:
            raise MissionError(f"{ctx}: speed must be > 0")
        check = raw.get("check") or {}
        if not isinstance(check, dict):
            raise MissionError(f"{ctx}: check must be a mapping")
        unknown = sorted(set(check) - {"tolerance_deg"})
        if unknown:
            raise MissionError(f"{ctx}: unknown check option(s) {', '.join(unknown)}")
        step["tolerance_deg"] = _number(
            check.get("tolerance_deg", 3.0), "check.tolerance_deg", ctx)
        if stype == "arm_home":
            if raw.get("angles") is not None or raw.get("joint") is not None:
                raise MissionError(f"{ctx}: arm_home takes no angles (it goes to all-zero)")
            step["angles"] = [0.0] * ARM_JOINTS
        elif stype == "arm_pose":
            if raw.get("angles") is not None or raw.get("joint") is not None:
                raise MissionError(f"{ctx}: arm_pose takes a pose name, not angles")
            pose = raw.get("pose")
            if pose not in poses:
                raise MissionError(f"{ctx}: pose must be one of {', '.join(poses)}, got {pose!r}")
            step["pose"] = pose
            step["angles"] = [float(a) for a in poses[pose]]
        else:
            angles = raw.get("angles")
            joint = raw.get("joint")
            if angles is None and joint is None:
                raise MissionError(f"{ctx}: arm_joint needs angles or joint + angle_deg")
            if angles is not None and joint is not None:
                raise MissionError(f"{ctx}: use either angles or joint + angle_deg, not both")
            if angles is not None:
                if (not isinstance(angles, (list, tuple))
                        or len(angles) != ARM_JOINTS):
                    raise MissionError(
                        f"{ctx}: angles must be a list of {ARM_JOINTS} numbers (degrees)")
                step["angles"] = [
                    _number(a, f"angles[{i}]", ctx, allow_none=True)
                    for i, a in enumerate(angles)]
                if all(a is None for a in step["angles"]):
                    raise MissionError(f"{ctx}: angles must set at least one joint")
            else:
                if not isinstance(joint, int) or isinstance(joint, bool):
                    raise MissionError(f"{ctx}: joint must be a whole number 1..{ARM_JOINTS}")
                if not 1 <= joint <= ARM_JOINTS:
                    raise MissionError(f"{ctx}: joint must be 1..{ARM_JOINTS}")
                angle = _number(raw.get("angle_deg"), "angle_deg", ctx)
                if angle is None:
                    raise MissionError(f"{ctx}: angle_deg is required with joint")
                step["angles"] = [None] * ARM_JOINTS
                step["angles"][joint - 1] = angle

    elif stype == "command":
        _check_unknown_keys(
            raw, ("type", "name", "timeout", "run", "sim_run", "real_run", "expect"), ctx)
        for key in ("run", "sim_run", "real_run"):
            line = raw.get(key)
            if line is None:
                step[key] = None
                continue
            if not isinstance(line, str) or not line.strip():
                raise MissionError(f"{ctx}: {key} must be a non-empty command")
            step[key] = line.strip()
        if not any(step[key] for key in ("run", "sim_run", "real_run")):
            raise MissionError(f"{ctx}: a command step needs run, sim_run or real_run")
        expect = raw.get("expect", 0)
        if isinstance(expect, bool) or not isinstance(expect, int):
            raise MissionError(f"{ctx}: expect must be a whole number (the exit code to expect)")
        step["expect"] = expect
        if step["timeout"] is None:
            step["timeout"] = COMMAND_TIMEOUT
    return step


PARK_SPEED = 10.0   # deg/s - the clean-up moves are slow on purpose


def park_arms_mission(target):
    """The built-in mission behind the dashboard's "Park arms" button: every
    arm of the target back to the standard position.

    J2 goes back first, on its own: from a serving pose that lifts the arm
    up and away from whatever it reached towards (a table) before the other
    joints turn. Each arm keeps its own other joints during that first move."""
    standard = ARM_POSES["standard"]
    lift = [None] * ARM_JOINTS
    lift[1] = standard[1]
    return normalize_mission({
        "name": "Park the arms in the standard position",
        "target": target, "clock": "wall",
        "steps": [
            {"name": "Arm: lift J2 back (away from the table)", "type": "arm_joint",
             "angles": lift, "speed": PARK_SPEED},
            {"name": "Arm: the standard position", "type": "arm_pose",
             "pose": "standard", "speed": PARK_SPEED},
        ]}, origin="park arms")


def home_mission(target, waypoint="Home"):
    """The built-in mission behind the "Avular home" button: navigate back
    to the home waypoint. The arms must be parked first - the driving rule
    checks them against the standard position, the pose they rest in."""
    return normalize_mission({
        "name": f"Avular back to {waypoint}",
        "target": target,
        # the real robot has no Gazebo /clock
        "clock": "wall" if target == "real" else "sim",
        "require_arm_tucked": True,
        "arm_poses": {TUCK_POSE: list(ARM_POSES["standard"])},
        "steps": [{"name": f"Navigate back to {waypoint}", "type": "goto",
                   "waypoint": waypoint}],
    }, origin="avular home")


def normalize_mission(data, origin="mission"):
    """Validate a parsed mission document and fill in all defaults."""
    if not isinstance(data, dict):
        raise MissionError(f"{origin}: must be a mapping with 'name' and 'steps'")
    unknown = sorted(set(data) - {"name", "clock", "steps", "arm_poses", "require_arm_tucked",
                                  "target"})
    if unknown:
        raise MissionError(f"{origin}: unknown option(s) {', '.join(unknown)}")
    poses = dict(ARM_POSES)
    extra = data.get("arm_poses") or {}
    if not isinstance(extra, dict):
        raise MissionError(f"{origin}: arm_poses must be a mapping of name: [6 angles]")
    for pname, angles in extra.items():
        if (not isinstance(angles, (list, tuple)) or len(angles) != ARM_JOINTS
                or any(isinstance(a, bool) or not isinstance(a, (int, float)) for a in angles)):
            raise MissionError(f"{origin}: arm_poses.{pname} must be {ARM_JOINTS} numbers (degrees)")
        poses[str(pname)] = [float(a) for a in angles]
    tucked = data.get("require_arm_tucked", False)
    if not isinstance(tucked, bool):
        raise MissionError(f"{origin}: require_arm_tucked must be true or false")
    clock = data.get("clock", "sim")
    if clock not in ("sim", "wall"):
        raise MissionError(f"{origin}: clock must be 'sim' or 'wall', got {clock!r}")
    target = data.get("target", "sim")
    if target not in TARGETS:
        raise MissionError(
            f"{origin}: target must be one of {', '.join(TARGETS)}, got {target!r}")
    raw_steps = data.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        raise MissionError(f"{origin}: 'steps' must be a non-empty list")
    if target != "sim":
        # Safety: the real xArm 6 has a gripper mounted at its front, so the
        # all-zero home position is not a place it can go. A mission that
        # runs on the real arm (target real, or both) parks and drives in the
        # "standard" pose instead - see ARM_POSES.
        for i, raw in enumerate(raw_steps):
            if not isinstance(raw, dict):
                continue
            all_zero = (raw.get("type") == "arm_home"
                        or (raw.get("type") == "arm_joint"
                            and isinstance(raw.get("angles"), (list, tuple))
                            and len(raw["angles"]) == ARM_JOINTS
                            and all(a == 0 for a in raw["angles"])))
            if all_zero:
                raise MissionError(
                    f"{origin}: step {i + 1}: this step drives the arm to all-zero "
                    f"angles, which the real xArm cannot take (gripper at the front). "
                    f"Use arm_pose with a safe pose, e.g. 'standard'.")
    # An explicit timeout is kept as-is; when none is given the engine
    # computes a deadline from the step type and the live sim speed
    # (engine.MissionEngine._deadline).
    return {
        "name": str(data.get("name") or "unnamed mission"),
        "clock": clock,
        "target": target,
        "require_arm_tucked": tucked,
        "poses": poses,
        "arm_poses_doc": copy.deepcopy(extra),  # as written, for the editor
        "steps": [_normalize_step(raw, i, poses) for i, raw in enumerate(raw_steps)],
    }


def _parse(text, ext):
    ext = (ext or "").lower().lstrip(".")
    if ext in ("yaml", "yml"):
        try:
            import yaml  # lazy: not needed for .json missions / unit tests
        except ImportError as exc:
            raise MissionError("PyYAML is required for .yaml missions "
                               "(pip install pyyaml)") from exc
        return yaml.safe_load(text)
    return json.loads(text)


def read_mission_doc(path):
    """The mission file as written (not normalized), for the editor."""
    with open(path, encoding="utf-8") as f:
        text = f.read()
    try:
        return _parse(text, os.path.splitext(path)[1])
    except Exception as exc:
        raise MissionError(f"{os.path.basename(path)}: {exc}") from exc


def clean_doc(doc):
    """Only the keys a mission file may contain, in a readable order."""
    out = {"name": doc.get("name") or "unnamed mission"}
    if doc.get("clock", "sim") != "sim":
        out["clock"] = doc["clock"]
    if doc.get("target", "sim") != "sim":
        out["target"] = doc["target"]
    if doc.get("require_arm_tucked"):
        out["require_arm_tucked"] = doc["require_arm_tucked"]
    if doc.get("arm_poses"):
        out["arm_poses"] = doc["arm_poses"]
    out["steps"] = doc.get("steps") or []
    return out


def dump_mission(doc, filename):
    """Text for a mission file; YAML for .yaml/.yml, JSON otherwise."""
    doc = clean_doc(doc)
    if os.path.splitext(filename)[1].lower() in (".yaml", ".yml"):
        import yaml
        return ("# Saved from the COFF-E dashboard step editor.\n"
                + yaml.safe_dump(doc, sort_keys=False, allow_unicode=True,
                                 default_flow_style=None))
    return json.dumps(doc, indent=2) + "\n"


def safe_filename(name):
    """Turns user input into a plain mission file name, or raises."""
    name = (name or "").strip()
    stem, ext = os.path.splitext(name)
    if ext.lower() not in (".yaml", ".yml", ".json"):
        stem, ext = name, ".yaml"
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,60}", stem):
        raise MissionError("file name may only use letters, digits, '-', '_' and '.'")
    return stem + ext.lower()


def load_mission_text(text, filename="mission.yaml"):
    try:
        data = _parse(text, os.path.splitext(filename)[1])
    except Exception as exc:  # yaml/json syntax errors
        raise MissionError(f"{filename}: {exc}") from exc
    return normalize_mission(data, origin=filename)


def load_mission(path):
    with open(path, encoding="utf-8") as f:
        return load_mission_text(f.read(), os.path.basename(path))


def list_missions(folder):
    """All mission files in a folder, with their display names.

    Returns [{"file", "name", "error"}]; a broken file still shows up so the
    dashboard can tell the user what is wrong with it.
    """
    missions = []
    if not os.path.isdir(folder):
        return missions
    for fname in sorted(os.listdir(folder)):
        if os.path.splitext(fname)[1].lower() not in (".yaml", ".yml", ".json"):
            continue
        entry = {"file": fname, "name": None, "error": None}
        try:
            entry["name"] = load_mission(os.path.join(folder, fname))["name"]
        except MissionError as exc:
            entry["error"] = str(exc)
        except OSError as exc:
            entry["error"] = f"{fname}: {exc}"
        missions.append(entry)
    return missions
