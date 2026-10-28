"""Where the orchestrator sends its commands: the simulation, the real
robots, or both at once.

A mission runs on a *target*:

  sim    the two simulation containers (Gazebo Origin One + UFACTORY xArm)
  real   the real Avular Origin One and the real xArm 6
  both   the same commands go to both, so a mission can be rehearsed in the
         simulation while the real robots do it

Everything the engine does goes through one Hardware object: it fans the
commands out to the targets of the current run and reads the feedback from the
first one (the simulation when both are enabled), so the mission engine does
not have to know where the commands go.

Real robot endpoints come from the environment, so the lab network can change
without touching the code:

  REAL_ROBOT_IP           IP of the real Origin One. Empty [default] = the
                          real target has no mobile base (yet): drive/goto
                          steps then have nothing to drive and fail clearly,
                          and a "both" mission drives the simulation while
                          the real arm does the arm steps. Set it (and start
                          the zenoh bridge, compose profile "real") once the
                          Avular is connected.
  REAL_ROS_DOMAIN_ID      ROS 2 domain of the real robot [42]. The orchestrator
                          joins that domain with its own ROS context; the zenoh
                          bridge (compose profile "real") is the only thing in
                          it that talks to the robot itself.
  REAL_CMD_VEL_TOPIC      /robot/cmd_vel_user
  REAL_MODE_SERVICE       /robot/cmd_vel_controller/set_control_mode
  REAL_MODE_RESET_SERVICE /robot/cmd_vel_controller/reset_control_mode
  REAL_CONTROL_MODE       30 (USER)
  REAL_ODOM_TOPIC         /robot/odom
  REAL_POSE_TOPIC         /autopilot/estimated_pose (global pose)
  REAL_BEHAVIOR_SERVICE   /autopilot/information_manager/set_and_execute_behaviors
  REAL_XARM_HOST          IP of the real xArm control box (empty = no arm)
  SIM_DRIVE_MODE          direct [default] = publish /robot/cmd_vel like the
                          origin-nav container does; user = take control and
                          publish /robot/cmd_vel_user, exactly like the real
                          robot, so the real command path can be tested in the
                          simulation.

The Avular Origin One does not drive from /robot/cmd_vel: control has to be
taken first (set_control_mode USER, see origin_msgs/srv/SetControlMode) and
then Twist messages go to /robot/cmd_vel_user. Navigation is done by the
autopilot ("move through poses" behavior), not by Nav2 - Nav2 only exists in
the simulation.
"""

import os
import time

SIM = "sim"
REAL = "real"
BOTH = "both"
TARGETS = (SIM, REAL, BOTH)
TARGET_LABEL = {SIM: "Simulation", REAL: "Real robots", BOTH: "Simulation + real"}


def _text(env, key, default):
    value = (env.get(key) or "").strip()
    return value or default


def _whole(env, key, default):
    try:
        return int(str(env.get(key, "")).strip())
    except (TypeError, ValueError):
        return default


def sim_config(env=None):
    """The simulation: Gazebo Origin One + UFACTORY Studio xArm."""
    env = env if env is not None else os.environ
    user_mode = _text(env, "SIM_DRIVE_MODE", "direct").lower() == "user"
    return {
        "name": SIM,
        "label": "Simulation (Gazebo + UFACTORY Studio)",
        "domain_id": _whole(env, "SIM_ROS_DOMAIN_ID", 0),
        "cmd_vel_topic": _text(env, "SIM_CMD_VEL_TOPIC",
                               "/robot/cmd_vel_user" if user_mode else "/robot/cmd_vel"),
        "cmd_vel_type": "geometry_msgs/msg/Twist",
        "mode_service": (_text(env, "SIM_MODE_SERVICE",
                               "/robot/cmd_vel_controller/set_control_mode")
                         if user_mode else None),
        "mode_service_type": "origin_msgs/srv/SetControlMode",
        "mode_reset_service": (_text(env, "SIM_MODE_RESET_SERVICE",
                                     "/robot/cmd_vel_controller/reset_control_mode")
                               if user_mode else None),
        "mode_reset_type": "origin_msgs/srv/ReturnControlMode",
        "control_mode": _whole(env, "SIM_CONTROL_MODE", 30),
        "odom_topic": _text(env, "SIM_ODOM_TOPIC", "/robot/odom"),
        "clock_topic": _text(env, "SIM_CLOCK_TOPIC", "/clock"),
        "pose_topic": None,
        "nav_kind": "action",
        "nav_name": _text(env, "SIM_NAV_ACTION", "/navigate_to_pose"),
        "nav_type": "nav2_msgs/action/NavigateToPose",
        "nav_frame": _text(env, "SIM_NAV_FRAME", "map"),
        "arrival_radius": 0.6,
        "arm_host": _text(env, "XARM_HOST", "192.168.100.200"),
        "arm_label": "simulated xArm 6 (UFACTORY Studio)",
        "scene_file": _text(env, "CEREBRA_SCENE", "/tmp/cerebra/scene.json"),
    }


def real_config(env=None):
    """The real Avular Origin One and the real xArm 6 (see the module
    docstring for the environment variables)."""
    env = env if env is not None else os.environ
    return {
        "name": REAL,
        "label": "Real Avular Origin One + xArm 6",
        # No IP yet = the real target exists as arm only: drive/goto steps
        # have no mobile base to drive (see NoRobot), and a "both" mission
        # drives the simulation while the real arm does the arm steps.
        "robot_ip": _text(env, "REAL_ROBOT_IP", ""),
        "domain_id": _whole(env, "REAL_ROS_DOMAIN_ID", 42),
        "cmd_vel_topic": _text(env, "REAL_CMD_VEL_TOPIC", "/robot/cmd_vel_user"),
        "cmd_vel_type": "geometry_msgs/msg/Twist",
        "mode_service": _text(env, "REAL_MODE_SERVICE",
                              "/robot/cmd_vel_controller/set_control_mode"),
        "mode_service_type": "origin_msgs/srv/SetControlMode",
        "mode_reset_service": _text(env, "REAL_MODE_RESET_SERVICE",
                                    "/robot/cmd_vel_controller/reset_control_mode"),
        "mode_reset_type": "origin_msgs/srv/ReturnControlMode",
        "control_mode": _whole(env, "REAL_CONTROL_MODE", 30),
        "odom_topic": _text(env, "REAL_ODOM_TOPIC", "/robot/odom"),
        "clock_topic": None,          # the real robot has no Gazebo /clock
        "pose_topic": _text(env, "REAL_POSE_TOPIC", "/autopilot/estimated_pose"),
        "nav_kind": "behavior",       # autopilot behavior service, not Nav2
        "nav_name": _text(env, "REAL_BEHAVIOR_SERVICE",
                          "/autopilot/information_manager/set_and_execute_behaviors"),
        "nav_type": "knowledge_base_msgs/srv/SetAndExecuteBehaviors",
        "nav_frame": _text(env, "REAL_NAV_FRAME", "map"),
        "arrival_radius": 0.6,
        "arm_host": _text(env, "REAL_XARM_HOST", ""),
        "arm_label": "real xArm 6 control box",
        "scene_file": _text(env, "CEREBRA_SCENE", "/tmp/cerebra/scene.json"),
    }


def config(name, env=None):
    return sim_config(env) if name == SIM else real_config(env)


def configs_for(target, env=None):
    """The endpoint configurations of a target ("both" = simulation first, so
    the simulation stays the one whose feedback is checked)."""
    if target == BOTH:
        return [sim_config(env), real_config(env)]
    if target == REAL:
        return [real_config(env)]
    return [sim_config(env)]


class NoRobot:
    """A neutral stand-in for the robot link of a target that has no mobile
    base (the real target until REAL_ROBOT_IP is set): reads stay neutral
    and every command is refused, so a drive or goto step fails with a clear
    message instead of waiting for a robot that is not there. It is never in
    the command fan-out - only the feedback reads and the dashboard snapshot
    go through it."""

    def snapshot(self):
        return {
            "target": "real",
            "state": "not configured",
            "error": "no mobile base for this target (REAL_ROBOT_IP is not set)",
            "connected": False, "in_control": False, "nav_ready": False,
            "x": 0.0, "y": 0.0, "yaw": 0.0, "speed": 0.0,
            "time_factor": 0.0, "sim_fresh": False,
            "waypoints": [], "trajectory": [], "map": "", "spawn": "",
            "cmd_vel_topic": "", "domain_id": None, "mode_service": None,
            "nav_kind": "",
        }

    # reads
    def sim_seconds(self):
        return 0.0

    def sim_fresh(self):
        return False

    def time_factor(self):
        return 0.0

    def odom(self):
        return {"x": 0.0, "y": 0.0, "yaw": 0.0, "speed": 0.0, "fresh": False}

    def odom_topic(self):
        return "(no mobile base)"

    def pose(self):
        return None

    def waypoints(self):
        return []

    def waypoint(self, name):
        return None

    def nav_ready(self, action=None):
        return False

    def nav_status(self, goal):
        return "", 0.0

    def nav_error(self):
        return "no mobile base for this target"

    # commands (a backstop: the fan-out never includes this stand-in)
    def take_control(self, topic=None):
        return False, "no mobile base for this target (set REAL_ROBOT_IP)"

    def release_control(self, topic=None):
        return True, ""

    def publish_twist(self, msg, topic=None):
        pass

    def publish_cmd(self, linear, angular, topic=None):
        pass

    def stop(self, topic=None):
        pass

    def nav_send(self, x, y, yaw, frame=None, action=None):
        return None

    def nav_cancel(self, goal):
        pass

    def close(self):
        pass


_NO_ROBOT = NoRobot()


class Runtime:
    """One target: its configuration plus its ROS link and its arm link."""

    def __init__(self, cfg, log=print):
        from arm_control import Arm
        from ros_robot import Robot
        self.cfg = cfg
        # The real target exists without its mobile base until REAL_ROBOT_IP
        # is set (arm-only testing): there is then nothing to drive, and
        # drive/goto steps fail clearly instead of talking to a dead link.
        has_robot = cfg["name"] == SIM or cfg.get("robot_ip")
        self.robot = Robot(cfg, log=log) if has_robot else None
        self.arm = Arm(host=cfg["arm_host"], log=log)


class Hardware:
    """The robot(s) and arm(s) of the current run, behind one interface.

    It implements both the robot interface and the arm interface the mission
    engine uses (see engine.py): commands go to every active target, while
    feedback comes from the first one.
    """

    def __init__(self, env=None, log=print):
        self.env = dict(env if env is not None else os.environ)
        self.log = log
        self._runtimes = {}
        self.target = SIM
        self.active = []
        self.use(SIM)

    # ------------------------------------------------------------- targets

    def use(self, target):
        """Select the target of the next run (links are created on demand)."""
        if target not in TARGETS:
            raise ValueError(f"unknown target {target!r}")
        self.target = target
        names = [SIM] if target == SIM else [REAL] if target == REAL else [SIM, REAL]
        self.active = [self.connect(name) for name in names]
        return self

    def connect(self, name):
        """Create the links of one target (called when it is first used)."""
        if name not in self._runtimes:
            rt = Runtime(config(name, self.env), log=self.log)
            self._runtimes[name] = rt
            if rt.robot is None:
                base = "no mobile base - REAL_ROBOT_IP is not set"
            else:
                base = (f"ROS domain {rt.cfg['domain_id']}, "
                        f"cmd_vel on {rt.cfg['cmd_vel_topic']}")
            self.log(f"[targets] {name}: {rt.cfg['label']} ({base})")
        return self._runtimes[name]

    def names(self):
        return [rt.cfg["name"] for rt in self.active]

    def label(self):
        return TARGET_LABEL.get(self.target, self.target)

    def domain_for(self, name):
        for rt in self.active:
            if rt.cfg["name"] == name:
                return rt.cfg["domain_id"]
        return 0

    def _primary(self):
        return self.active[0]

    def _robots(self):
        """The active targets that have a mobile base to drive. The real
        target has none until REAL_ROBOT_IP is set, so with the real arm
        connected a "both" mission drives the simulation and moves the real
        arm - the dress-rehearsal setup."""
        return [rt for rt in self.active if rt.robot is not None]

    def _primary_robot(self):
        """The robot link the engine reads feedback from; a neutral stand-in
        when the primary target has no mobile base."""
        robot = self._primary().robot
        return robot if robot is not None else _NO_ROBOT

    def snapshot(self):
        """Per-target state for the dashboard (only targets that exist)."""
        return {name: {
            "name": name,
            "label": rt.cfg["label"],
            "active": name in self.names(),
            "domain_id": rt.cfg["domain_id"],
            "cmd_vel_topic": rt.cfg["cmd_vel_topic"],
            "mode_service": rt.cfg["mode_service"],
            "nav_name": rt.cfg["nav_name"],
            "nav_kind": rt.cfg["nav_kind"],
            "arm_host": rt.cfg["arm_host"] or None,
            "robot": rt.robot.snapshot() if rt.robot else _NO_ROBOT.snapshot(),
            "arm": rt.arm.snapshot(),
        } for name, rt in self._runtimes.items()}

    def robot_snapshot(self):
        """The robot state of the current run (for the dashboard panels)."""
        return self._primary_robot().snapshot()

    def arm_snapshot(self):
        return self._primary().arm.snapshot()

    def close(self):
        for rt in self._runtimes.values():
            if rt.robot is None:
                continue
            try:
                rt.robot.close()
            except Exception:
                pass

    # --------------------------------------------------- robot interface

    @staticmethod
    def wall_now():
        return time.time()

    def sim_seconds(self):
        return self._primary_robot().sim_seconds()

    def sim_fresh(self):
        return self._primary_robot().sim_fresh()

    def time_factor(self):
        return self._primary_robot().time_factor()

    def odom(self):
        return self._primary_robot().odom()

    def odom_topic(self):
        return self._primary_robot().odom_topic()

    def pose(self):
        return self._primary_robot().pose()

    def publish_twist(self, msg, topic=None):
        for rt in self._robots():
            rt.robot.publish_twist(msg, topic=topic)

    def publish_cmd(self, linear, angular, topic=None):
        for rt in self._robots():
            rt.robot.publish_cmd(linear, angular, topic=topic)

    def stop(self, topic=None):
        for rt in self._robots():
            rt.robot.stop(topic=topic)

    def take_control(self, topic=None):
        """[(target, ok, message)] - take over every mobile base that exists
        (the real Origin One only drives in USER mode)."""
        robots = self._robots()
        if not robots:
            return [(self._primary().cfg["name"], False,
                     "no mobile base for this target (set REAL_ROBOT_IP) - "
                     "nothing to drive")]
        return [(rt.cfg["name"],) + rt.robot.take_control(topic=topic)
                for rt in robots]

    def release_control(self, topic=None):
        return [(rt.cfg["name"],) + rt.robot.release_control(topic=topic)
                for rt in self._robots()]

    def nav_ready(self, action=None):
        return self._primary_robot().nav_ready(action=action)

    def nav_send(self, x, y, yaw, frame=None, action=None):
        """One goal per mobile base; None when any of them refused it."""
        goals = {}
        for rt in self._robots():
            goal = rt.robot.nav_send(x, y, yaw, frame=frame, action=action)
            if goal is None:
                return None
            goals[rt.cfg["name"]] = goal
        return goals or None

    def nav_status(self, goals):
        return self._primary_robot().nav_status(goals.get(self._primary().cfg["name"]))

    def nav_cancel(self, goals):
        for rt in self._robots():
            rt.robot.nav_cancel(goals.get(rt.cfg["name"]))

    def nav_error(self):
        return self._primary_robot().nav_error()

    def waypoint(self, name):
        """Waypoint lookup always uses the shared Cerebra scene (the map the
        simulation and the real robot are both localised in)."""
        rt = self._runtimes.get(SIM) or self._primary()
        robot = rt.robot if rt.robot is not None else _NO_ROBOT
        return robot.waypoint(name)

    def waypoints(self):
        rt = self._runtimes.get(SIM) or self._primary()
        robot = rt.robot if rt.robot is not None else _NO_ROBOT
        return robot.waypoints()

    # ----------------------------------------------------- arm interface

    def angles(self):
        return self._primary().arm.angles()

    def angles_all(self):
        return [(rt.cfg["name"], rt.arm.angles()) for rt in self.active]

    def arm_faults(self):
        """{target: controller error} of every active arm that has one."""
        return {rt.cfg["name"]: rt.arm.fault() for rt in self.active if rt.arm.fault()}

    def command_joints(self, angles_deg, speed, acc=None, wait=False):
        """{target: SDK code} - the same joint move on every active arm.

        A joint left as None keeps the angle of *that* arm: filling it in
        from the simulated arm would send the simulation's joints to the
        real arm whenever the two stand differently."""
        codes = {}
        for rt in self.active:
            own = rt.arm.angles()
            if any(a is None for a in angles_deg) and own is None:
                codes[rt.cfg["name"]] = -1
                continue
            angles = [own[j] if a is None else a for j, a in enumerate(angles_deg)]
            codes[rt.cfg["name"]] = rt.arm.command_joints(angles, speed, acc=acc, wait=wait)
        return codes
