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
  arm_home   shortcut for arm_joint with all joints at 0 degrees
  wait       do nothing for <duration> seconds

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

STEP_TYPES = ("drive", "goto", "arm_joint", "arm_home", "wait")

ARM_JOINTS = 6  # xArm 6


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


def _normalize_step(raw, index):
    step = _normalize_step_fields(raw, index)
    # The step exactly as written, so the dashboard editor can show and edit
    # it, and live edits can tell which steps were left unchanged.
    step["raw"] = copy.deepcopy(raw)
    return step


def _normalize_step_fields(raw, index):
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
                                  "check"), ctx)
        step["duration"] = _number(raw.get("duration"), "duration", ctx)
        if step["duration"] <= 0:
            raise MissionError(f"{ctx}: duration must be > 0")
        if stype == "wait":
            return step
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
        _check_unknown_keys(raw, ("type", "name", "timeout", "waypoint", "x", "y", "yaw"), ctx)
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

    elif stype in ("arm_joint", "arm_home"):
        _check_unknown_keys(
            raw, ("type", "name", "timeout", "angles", "joint", "angle_deg", "speed",
                  "check"), ctx)
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
    return step


def normalize_mission(data, origin="mission"):
    """Validate a parsed mission document and fill in all defaults."""
    if not isinstance(data, dict):
        raise MissionError(f"{origin}: must be a mapping with 'name' and 'steps'")
    unknown = sorted(set(data) - {"name", "clock", "steps"})
    if unknown:
        raise MissionError(f"{origin}: unknown option(s) {', '.join(unknown)}")
    clock = data.get("clock", "sim")
    if clock not in ("sim", "wall"):
        raise MissionError(f"{origin}: clock must be 'sim' or 'wall', got {clock!r}")
    raw_steps = data.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        raise MissionError(f"{origin}: 'steps' must be a non-empty list")
    # An explicit timeout is kept as-is; when none is given the engine
    # computes a deadline from the step type and the live sim speed
    # (engine.MissionEngine._deadline).
    return {
        "name": str(data.get("name") or "unnamed mission"),
        "clock": clock,
        "steps": [_normalize_step(raw, i) for i, raw in enumerate(raw_steps)],
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
