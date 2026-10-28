"""Copy-ready command text for every mission step.

The dashboard shows this next to a step (the "RAW" button): the messages the
orchestrator sends, written out as `ros2` command lines - and, for the arm,
as the xArm Python SDK call plus the equivalent xarm_ros2 service call. That
way the same mission can be run on the real Avular Origin One and the real
xArm 6 by hand, and the raw editor can be filled in with values that are known
to be right.

Everything here is pure text generation (no ROS, no hardware), so it is unit
tested; `step_commands` therefore takes waypoint coordinates as a resolver
argument instead of reading them from the robot.
"""

import math

CMD_RATE = 10.0          # Hz: the rate drive steps publish at (see engine.py)
BEHAVIOR_MOVE_THROUGH_POSES = 10   # knowledge_base_msgs/msg/Behavior
XARM_SERVO_SERVICE = "/xarm/set_servo_angle"
XARM_SERVO_TYPE = "xarm_msgs/srv/MoveJoint"
USER_MODE_SERVICE = "/robot/cmd_vel_controller/set_control_mode"
USER_MODE_RESET_SERVICE = "/robot/cmd_vel_controller/reset_control_mode"


def num(value, digits=6):
    """A number for a command line: 30.0 -> "30", 0.25 -> "0.25"."""
    if value is None:
        return "0"
    text = f"{float(value):.{digits}f}".rstrip("0").rstrip(".")
    return text if text not in ("", "-") else "0"


def deg2rad(deg):
    return float(deg) * math.pi / 180.0


def yaw_quaternion(yaw):
    """(z, w) of a rotation about z, for a geometry_msgs orientation."""
    return math.sin(float(yaw) / 2.0), math.cos(float(yaw) / 2.0)


def twist_fields(step):
    """The Twist a drive step publishes: the structured speed/turn rate plus
    whatever the raw override adds (x/y/z and roll/pitch/yaw)."""
    ros = step.get("ros") or {}
    linear = {"x": float(step.get("linear") or 0.0), "y": 0.0, "z": 0.0}
    angular = {"x": 0.0, "y": 0.0, "z": float(step.get("angular") or 0.0)}
    linear.update({k: float(v) for k, v in (ros.get("linear") or {}).items()})
    angular.update({k: float(v) for k, v in (ros.get("angular") or {}).items()})
    return linear, angular


def twist_yaml(linear, angular):
    return (f'{{linear: {{x: {num(linear["x"])}, y: {num(linear["y"])}, '
            f'z: {num(linear["z"])}}}, angular: {{x: {num(angular["x"])}, '
            f'y: {num(angular["y"])}, z: {num(angular["z"])}}}}}')


def goal_fields(step, resolve_waypoint=None):
    """(x, y, yaw, z) of a goto step, after raw overrides."""
    ros = step.get("ros") or {}
    if step.get("waypoint"):
        wp = resolve_waypoint(step["waypoint"]) if resolve_waypoint else None
        x, y, yaw = wp if wp else (0.0, 0.0, 0.0)
        known = wp is not None
    else:
        x, y, yaw = step.get("x") or 0.0, step.get("y") or 0.0, step.get("yaw") or 0.0
        known = True
    x = ros.get("x", x)
    y = ros.get("y", y)
    yaw = ros.get("yaw", yaw)
    z = ros.get("z", 0.0)
    return float(x), float(y), float(yaw), float(z), known


def pose_yaml(x, y, yaw, z=0.0, frame="map"):
    qz, qw = yaw_quaternion(yaw)
    return (f'{{header: {{frame_id: {frame}}}, pose: {{position: '
            f'{{x: {num(x)}, y: {num(y)}, z: {num(z)}}}, orientation: '
            f'{{x: 0, y: 0, z: {num(qz)}, w: {num(qw)}}}}}')


# --------------------------------------------------------------- one step

def drive_block(step, cfg):
    ros = step.get("ros") or {}
    linear, angular = twist_fields(step)
    topic = ros.get("topic") or cfg["cmd_vel_topic"]
    rate = ros.get("rate") or CMD_RATE
    duration = step.get("duration") or 0.0
    # The USER topic only works in control mode USER, so a step that overrides
    # the topic to it needs the same handshake as the real robot - also on a
    # target that normally drives on the plain cmd_vel topic.
    service = cfg.get("mode_service")
    reset = cfg.get("mode_reset_service")
    if not service and topic.endswith("_user"):
        service, reset = USER_MODE_SERVICE, USER_MODE_RESET_SERVICE
    lines = []
    if service:
        lines.append(f'# take control of the robot (control mode {cfg["control_mode"]} = USER)')
        lines.append(f'ros2 service call {service} {cfg["mode_service_type"]} '
                     f'"{{mode: {{mode: {cfg["control_mode"]}}}}}"')
    lines.append(f"# publish for {num(duration)} s at {num(rate)} Hz, then send a zero Twist")
    lines.append(f'ros2 topic pub -r {num(rate)} {topic} {cfg["cmd_vel_type"]} '
                 f'"{twist_yaml(linear, angular)}"')
    if reset:
        lines.append("# ... and give control back")
        lines.append(f'ros2 service call {reset} {cfg["mode_reset_type"]} '
                     f'"{{mode_from: {{mode: {cfg["control_mode"]}}}}}"')
    note = f"the orchestrator checks that the odometry moved {num(step.get('min_distance'))} m" \
        if step.get("min_distance") is not None else \
        "the orchestrator keeps publishing for the whole duration, then stops the robot"
    return {"target": cfg["name"], "label": cfg["label"], "domain_id": cfg["domain_id"],
            "lines": lines, "note": note, "alternatives": []}


def goto_block(step, cfg, resolve_waypoint=None):
    x, y, yaw, z, known = goal_fields(step, resolve_waypoint)
    frame = (step.get("ros") or {}).get("frame") or cfg["nav_frame"]
    action = (step.get("ros") or {}).get("action") or cfg["nav_name"]
    pose = pose_yaml(x, y, yaw, z, frame)
    lines = []
    if cfg["nav_kind"] == "action":
        lines.append("# Nav2 (only the simulation runs Nav2; the real robot navigates itself)")
        lines.append(f'ros2 action send_goal {action} {cfg["nav_type"]} "{{pose: {pose}}}"')
        note = "the orchestrator waits for the action result and shows the remaining distance"
    else:
        lines.append("# the autopilot plans and drives the route itself "
                     "(the robot needs a loaded localization map)")
        lines.append(f'ros2 service call {action} {cfg["nav_type"]} '
                     f'"{{behaviors: [{{type: {BEHAVIOR_MOVE_THROUGH_POSES}, '
                     f'goals: [{pose}]}}]}}"')
        note = (f"type {BEHAVIOR_MOVE_THROUGH_POSES} = MOVE_THROUGH_POSES, yaw in the "
                f"quaternion is {num(math.degrees(yaw), 3)}°; the orchestrator then waits "
                f"until the robot is within {num(cfg['arrival_radius'])} m")
    if step.get("waypoint"):
        if known:
            note += f'; waypoint "{step["waypoint"]}" = ({num(x)}, {num(y)})'
        else:
            note += f'; waypoint "{step["waypoint"]}" is not in the Cerebra scene'
    return {"target": cfg["name"], "label": cfg["label"], "domain_id": cfg["domain_id"],
            "lines": lines, "note": note, "alternatives": []}


def _angle_list(angles):
    return "[" + ", ".join("None" if a is None else num(a) for a in angles) + "]"


def arm_block(step, cfg):
    arm = step.get("arm") or {}
    angles = step.get("angles") or []
    speed = step.get("speed") or 30.0
    host = cfg["arm_host"] or "<xarm host>"
    call = (f'XArmAPI("{host}").set_servo_angle(angle={_angle_list(angles)}, '
            f'speed={num(speed)}, ')
    if arm.get("acc") is not None:
        call += f"mvacc={num(arm['acc'])}, "
    call += f"wait={bool(arm.get('wait'))})"
    lines = [f"# the xArm Python SDK, the same call the orchestrator makes (angles in degrees)",
             call]
    radians = "[" + ", ".join(
        "null" if a is None else num(deg2rad(a)) for a in angles) + "]"
    acc = num(deg2rad(arm["acc"])) if arm.get("acc") is not None else "0"
    alt = [f'ros2 service call {XARM_SERVO_SERVICE} {XARM_SERVO_TYPE} '
           f'"{{angles: {radians}, speed: {num(deg2rad(speed))}, acc: {acc}}}"']
    note = "the orchestrator polls the joint angles until the move is within tolerance"
    if any(a is None for a in angles):
        note += "; joints left as None keep their current angle (filled in at run time)"
    if arm.get("wait"):
        note += "; the SDK call waits for the move to finish"
    return {"target": cfg["name"], "label": cfg["label"], "domain_id": cfg["domain_id"],
            "lines": lines, "note": note,
            "alternatives": [{"title": "xarm_ros2 (radians, on the robot)",
                              "lines": alt}]}


def command_block(step, cfg, name=None):
    line = step.get(f"{name}_run") if name else None
    line = line or step.get("run") or ""
    lines = [ln for ln in line.splitlines() if ln.strip()]
    note = (f"runs in the orchestrator container with ROS 2 sourced "
            f"(ROS_DOMAIN_ID={cfg['domain_id']}); exit code "
            f"{step.get('expect', 0)} is expected")
    if name and step.get(f"{name}_run"):
        note += f" (the {name} variant of this step)"
    return {"target": cfg["name"], "label": cfg["label"], "domain_id": cfg["domain_id"],
            "lines": lines, "note": note, "alternatives": []}


def step_commands(step, cfg, resolve_waypoint=None):
    """The commands of one step on one target (a dict the dashboard renders)."""
    kind = step.get("type")
    if kind == "drive":
        return drive_block(step, cfg)
    if kind == "goto":
        return goto_block(step, cfg, resolve_waypoint)
    if kind in ("arm_joint", "arm_home", "arm_pose"):
        return arm_block(step, cfg)
    if kind == "command":
        return command_block(step, cfg)
    return {"target": cfg["name"], "label": cfg["label"], "domain_id": cfg["domain_id"],
            "lines": [], "note": f"a {kind} step sends no ROS command",
            "alternatives": []}


def mission_commands(mission, cfgs, resolve_waypoint=None):
    """Every step of a mission, with its commands per target."""
    return {
        "target": mission.get("target", "sim"),
        "steps": [{
            "index": i,
            "name": step.get("name") or step.get("type"),
            "type": step.get("type"),
            "blocks": [step_commands(step, cfg, resolve_waypoint) for cfg in cfgs],
        } for i, step in enumerate(mission["steps"])],
    }
