# Step-by-step: ROS 2 on both simulations + the mission dashboard

This guide walks you through the COFF-E simulation stack in the order you
will actually use it:

| Part | What you learn |
|---|---|
| [1. Start the stack](#1-start-the-stack) | bring up both simulations + the orchestrator |
| [2. ROS 2 on the Origin One (driving robot)](#2-ros-2-on-the-origin-one-the-driving-robot) | topics, teleop, odometry, sim time |
| [3. ROS 2 on the xArm 6 (arm)](#3-ros-2-on-the-xarm-6-the-arm) | why the arm uses the SDK instead, Studio UI, SDK snippets |
| [4. The mission dashboard](#4-the-mission-dashboard) | one page that shows robot + arm + mission progress |
| [5. Run your first mission](#5-run-your-first-mission) | drive 60 s → arm 90° → drive further, with checks |
| [6. Edit missions in the step editor](#6-edit-missions-in-the-step-editor) | click a step, choose Avular or xArm, change it, even while it runs; live simulator view; sample missions |
| [7. See the raw ROS commands (and edit them)](#7-see-the-raw-ros-commands-and-edit-them) | the RAW button: the exact messages per step, the raw overrides, raw command steps |
| [8. Run the same mission on the real robots](#8-run-the-same-mission-on-the-real-robots) | the Run on switch, the zenoh bridge, what is different on hardware |
| [9. Write missions by hand](#9-write-missions-by-hand) | the step types and all options |
| [10. How the checking works](#10-how-the-checking-works) | what "PASS" actually means |
| [11. Troubleshooting](#11-troubleshooting) | |

## 1. Start the stack

```bash
cd Simulation-Docker
docker compose up -d --build
```

First build of the Origin One image takes 10-20 minutes; afterwards it is
cached. Wait ~30 s after `up`, then check that all four containers run:

```bash
docker compose ps
```

You should see `avular-origin-sim`, `origin-nav`, `ufactory-studio` (the
simulated xArm) and `coffe-orchestrator`. Then open:

| What | URL |
|---|---|
| **Mission dashboard** (start here) | <http://localhost:8080> |
| Gazebo GUI (watch the Origin One drive) | <http://localhost:6080/vnc.html?autoconnect=true&resize=scale> |
| RViz + Nav2 (path planning) | <http://localhost:6081/vnc.html?autoconnect=true&resize=scale> |
| UFACTORY Studio (watch the arm move) | <http://localhost:18333> |

All URLs are localhost-only, so nothing is exposed to the network.

> The simulation runs at roughly **0.4-0.5x real time** once Nav2 uses the
> lidar. The dashboard shows the live factor in the header (`sim 0.47x`),
> so you know what to expect: a "60 seconds" drive takes about two minutes
> to watch. Mission times are **Gazebo simulation time** by default, so a
> step is exactly 60 s *of robot time*.

## 2. ROS 2 on the Origin One (the driving robot)

ROS 2 runs **inside the `avular-origin-sim` container** - nothing is
installed on your Windows/macOS/Linux host. Open a shell in it:

```bash
docker exec -it avular-origin-sim bash
```

You now have a normal Ubuntu 22.04 with ROS 2 Humble and the Avular Origin
One packages. Everything below happens inside that shell.

### Step 2.1 - see what the robot publishes

```bash
ros2 topic list
```

The important ones (bridged between Gazebo and ROS 2):

| Topic | Type | What it is |
|---|---|---|
| `/robot/cmd_vel` | `geometry_msgs/msg/Twist` | **write**: drive the robot |
| `/robot/odom` | `nav_msgs/msg/Odometry` | **read**: where the robot actually is |
| `/robot/lidar/points` | `sensor_msgs/msg/PointCloud2` | the Ouster lidar |
| `/robot/camera/color/image_raw` | `sensor_msgs/msg/Image` | front camera |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo time (sim time) |

Watch the odometry live (Ctrl+C to stop):

```bash
ros2 topic echo /robot/odom
```

### Step 2.2 - drive it by hand

One message = one velocity command. Publish 0.5 m/s forward, 10x per
second, for a few seconds:

```bash
timeout 5 ros2 topic pub -r 10 /robot/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
```

Note: the robot keeps the **last** command - always send a zero afterwards
(`linear: {x: 0.0}`) or it drives on. (The orchestrator does this
automatically after every drive step.)

Or use the keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot/cmd_vel
```

### Step 2.3 - check that it really moved

The odometry topic is the ground truth for "did the robot do what I asked":

```bash
ros2 topic echo /robot/odom --once | grep -A2 "position:"
```

This is exactly the check the orchestrator uses after every drive step.

### Step 2.4 - sim time vs wall time

Gazebo publishes its own clock on `/clock`. Because the sim runs at ~0.5x
real time, "60 seconds" of driving is 60 seconds of *simulation* time.
Look at it with:

```bash
ros2 topic echo /clock
```

### Step 2.5 - drive to a waypoint with Nav2

The `origin-nav` container runs the navigation stack (AMCL localisation on
the Cerebra map, planner, controller) and exposes the waypoints of the
loaded map. From the host (no shell inside needed):

```bash
docker exec -it origin-nav goto list      # waypoints, paths, zones
docker exec -it origin-nav goto Home      # drive there, exit 0 = reached
```

The orchestrator's `goto` step uses the same Nav2 action under the hood.

## 3. ROS 2 on the xArm 6 (the arm)

The arm simulation is different on purpose: **UFACTORY does not use ROS 2**.
A real xArm control box speaks the UFACTORY TCP protocol, so the
simulation (and this guide, and the orchestrator) use the official
**xArm Python SDK** - the same SDK, the same code, as on real hardware.
Only the IP address changes between sim and lab.

### Step 3.1 - see the arm move in UFACTORY Studio

Open <http://localhost:18333> (if it says *"Unable to get robot SN"*, click
**Close** - the simulated arm still works). Jog a joint with the arrows and
watch it move. This is the "Gazebo equivalent" for the arm.

### Step 3.2 - read the arm state with Python

From the host (once: `pip install xarm-python-sdk`):

```python
from xarm.wrapper import XArmAPI

arm = XArmAPI('127.0.0.1', baud_checkset=False, check_joint_limit=False)
print("joints (deg):", arm.angles)   # [J1, J2, J3, J4, J5, J6]
arm.disconnect()
```

From inside a container on the compose network, use `192.168.100.200`
instead of `127.0.0.1` - that is the fixed address of the simulated
control box, exactly like a real xArm on the lab network.

### Step 3.3 - move the arm with Python

```python
from xarm.wrapper import XArmAPI
import time

arm = XArmAPI('127.0.0.1', baud_checkset=False, check_joint_limit=False)
arm.clean_error()
arm.motion_enable(enable=True)
arm.set_mode(0)     # 0 = position mode (like Studio)
arm.set_state(0)    # 0 = ready

# all-zero = "home", arm straight up. J1 = base rotation, so this is the
# "swing the arm 90 degrees" move:
code = arm.set_servo_angle(angle=[90, 0, 0, 0, 0, 0], speed=30, wait=True)
print("move result code:", code)          # 0 = accepted
print("joints now (deg):", arm.angles)    # the check: is J1 actually ~90?
```

That last line is the check the orchestrator performs after every arm step:
command sent (`code 0`), then **verify the reported joint angles** are
within a tolerance of the target.

So "ROS 2 on both simulations" in practice means:

- **Origin One** → ROS 2 topics (`/robot/cmd_vel` to write, `/robot/odom` to check)
- **xArm 6** → the SDK (`set_servo_angle` to write, `arm.angles` to check)

The orchestrator is the piece that speaks both, and the dashboard shows both.

## 4. The mission dashboard

Open <http://localhost:8080>:

| Panel | What it shows |
|---|---|
| **Header** | mission dropdown, Start/Abort, the two clean-up buttons (below), overall state, whether robot + arm are connected, the sim-time factor, links to Gazebo/RViz/Studio |
| **Step editor** | every step with its device (Avular / xArm), live progress and check result; click a step to edit it (see part 6) |
| **Origin One** | live position/heading/speed, the driven trajectory on a map (waypoints of the Cerebra map included), button to embed the Gazebo view |
| **xArm 6** | live joint angles with target markers, a stylized arm sketch that moves with J2/J3, button to embed UFACTORY Studio |
| **Log** | what the orchestrator did, step by step |

The robot panel plots **odometry relative to the spawn point**, which is
where the robot spawned (the `Home` waypoint by default).

### Clean up: Park arms and Avular home

Two header buttons put the robots back in their starting state, on the
target the **Run on** switch selects. Each one runs a small built-in mission,
so the step list shows its progress, every move is checked, and Abort stops
it:

| Button | What it does |
|---|---|
| **Park arms** | every arm back to the standard position [0, -90, 0, 0, 90, 0] at 10 deg/s: J2 goes back first on its own, which lifts the arm up and away from a table it reached towards, then the other joints follow. Each arm keeps its own other joints during that first move. |
| **Avular home** | the Avular navigates back to the home waypoint (`CEREBRA_SPAWN`). The arms must be parked first: the driving rule checks them against the standard position. While `REAL_ROBOT_IP` is not set, only the simulated Avular drives. |

After a clean-up run, pick the mission in the dropdown again to get back to it.

## 5. Run your first mission

On the dashboard:

1. Pick **Quick check: robot + arm end to end** (`quick_check.yaml`) - it is
   the short version of the real flow.
2. Press **Start mission**.
3. Watch the three panels: the trajectory grows while the robot drives,
   then the arm bars move to their target, then the robot drives back.
4. Each step turns green only when its **check** passed. If one fails, the
   mission stops there and shows why.

Or from a terminal, if you prefer curl:

```bash
curl -X POST http://localhost:8080/api/start -H "Content-Type: application/json" -d "{\"mission\": \"quick_check.yaml\"}"
curl -s http://localhost:8080/api/status | python -m json.tool | less   # watch
curl -X POST http://localhost:8080/api/abort                            # stop
```

The full-size version is **coffee_demo.yaml** - exactly the flow from the
project description:

1. drive forward for 60 s → check: odometry moved ≥ 2 m
2. arm joint 1 to 90° → check: joint within 3° of the target
3. arm back to home
4. drive further 30 s → check: moved ≥ 1 m
5. Nav2 drives the robot back to the `Home` waypoint

Each step only starts when the previous one **passed** - that is the
"something that checks it and connects them both".

## 6. Edit missions in the step editor

The left panel of the dashboard is a step-by-step editor. It shows the
mission as a list of numbered steps. Each step has a badge for the device
that does it (**AVULAR** for the Origin One, **XARM** for the arm,
**WAIT**) and a one-line summary of what it does.

1. Pick a mission in the dropdown. It loads into the editor.
2. **Click a step** to open it. Choose who does the step under **Done by**:
   *Avular Origin One*, *xArm 6* or *Wait*. Then pick the action and fill in
   the values:
   - Avular: *Drive for a time* (duration, speed, turn rate, minimum
     distance check) or *Navigate* (Cerebra waypoint or map x/y/yaw; Nav2 on
     the simulation, the autopilot on the real robot).
   - xArm: *Move joints* (one joint, or several joints where an empty box
     keeps that joint where it is) or *Home position*, with speed and tolerance.
3. Use the buttons under a step to move it up or down, duplicate or delete it,
   or insert a new Avular or xArm step right after it. **Add step** at the
   bottom adds one at the end.
4. The editor checks the mission as you type. A problem is shown in red,
   and the step it belongs to gets a red border.
5. **Start mission** runs the mission exactly as it is in the editor, even
   unsaved. **Save** writes it back to its file in `orchestrator/missions/`,
   **Save as…** writes a new file, and **New** starts an empty mission.

While a mission runs, the same list shows the live status: the running step
is highlighted and shows progress, passed steps turn green with their check
result, and steps that already ran or are running get a lock icon. You can
still open and change any **upcoming** step, or add, delete or reorder
steps after the current one. The change goes to the running mission within
a second ("Live: the running mission now uses the edited upcoming steps").
Locked steps cannot be changed.

### Watch it happen live

The **Live simulator view** at the top of the dashboard shows the
simulators themselves, inside the page:

| Button | Shows |
|---|---|
| **Gazebo (robot)** | the Origin One driving through the Cerebra map. The Gazebo camera follows the robot automatically (`GAZEBO_FOLLOW_ROBOT` in `docker-compose.yml`) |
| **RViz (Nav2 plan)** | the map, the planned path, waypoints and zones; best for `goto` steps |
| **UFACTORY Studio (arm)** | the xArm 6 in 3D |
| **Robot + arm side by side** | Gazebo and Studio next to each other |

With **follow the running step** on, the view switches to the simulator of
the step that is running: Gazebo for `drive`, RViz for `goto`, Studio for
arm steps. Every step in the editor also has a **👁 watch** button, and
the **▶ Live view** link in the header jumps to the panel. Each view has an
**open in a new tab ↗** link if you want it full screen.
### Sample missions: robot and arm together

The arm will later be mounted **on** the Origin One. These samples show
ideas for what the two can do together. They are in `orchestrator/missions/`
and appear in the dropdown. Open one in the step editor and change it.

| File | Idea |
|---|---|
| `sample_1_coffee_delivery.yaml` | fold arm in → drive to the table → hand the cup out to the side → wait → fold in → drive back |
| `sample_2_pick_and_place.yaml` | drive to an object, look down with the wrist camera, pick it up with the mission's own poses, turn, put it down on the left |
| `sample_3_patrol_inspect.yaml` | drive a square, inspect the floor at each corner with the arm |
| `sample_4_greeting.yaml` | wave at a visitor (J1 left/right), bow, then turn and drive off |
| `sample_5_safety_fail.yaml` | **fails on purpose**: tries to drive with the arm reaching out, and the safety rule refuses. Fix it by inserting an *Arm pose: transport* step |
| `sample_6_nav_serve.yaml` | Nav2 to a serving spot, serve to the left, Nav2 back to the Home waypoint (watch it in RViz) |
| `sample_7_raw_commands.yaml` | the raw side of the orchestrator: a raw command step, a drive with the raw Twist fields and publish rate spelled out, a drive on the USER topic (how the real robot is driven), a per-target command, and a hand-published zero Twist |
| `sample_8_real_coffee_delivery.yaml` | sample 1's delivery on the **real robots**: same steps and speeds, but the arm parks and drives in its standard position (the real arm has a front gripper, so all-zero is refused) |
| `sample_9_rehearsal.yaml` | **dress rehearsal**: the simulated Avular does the driving while the real arm does the arm steps (target: both with only the arm real) |

They all use two features made for the arm on the robot:

- **Named arm poses** (step type `arm_pose`): `home`, `standard` (the real
  xArm's parked position), `transport` (folded
  back over the robot, for driving), `reach_front`, `serve_left`,
  `serve_right`, `look_down`, `wave_up`. A mission can add its own with
  `arm_poses:` (see `sample_2_pick_and_place.yaml`). All built-in poses
  except `standard` are checked on the simulated xArm; `standard` is the
  real arm's position, where it parks and drives. Check the rest again on
  the real robot once the arm is mounted, because the robot body and its
  cargo are not in the arm simulation.
- **"Only drive with the arm tucked in"** (`require_arm_tucked: true`, the
  checkbox at the top of the step editor): before every `drive` or `goto`
  step the engine checks that the arm is within 5° of the `transport` pose.
  If not, the step fails and the robot does not move.

## 7. See the raw ROS commands (and edit them)

Every step in the editor has a **RAW** button. It opens the messages the
orchestrator sends for that step, as copy-ready command lines, for **both**
targets:

| Block | What it shows |
|---|---|
| Simulation (ROS domain 0) | what runs against the two simulations |
| Real robots (ROS domain 42) | what runs against the real Origin One and the real xArm 6 |

The block of the target the mission runs on is highlighted. A step that sends
nothing (a `wait`) says so; an arm step also shows the equivalent
**xarm_ros2** service call in radians next to the Python SDK call in degrees.
**copy** puts the text on the clipboard, so a mission can be run by hand on
the real robots.

The same view is available from a terminal:

```bash
curl -s -X POST http://localhost:8080/api/commands \
  -H "Content-Type: application/json" \
  -d '{"doc": {"name": "x", "steps": [{"type": "drive", "duration": 5, "linear": 0.25}]}}'
```

### Editing the raw message

Open a step and scroll to **Raw ROS message (override)** (an arm step has
**Raw xArm SDK options** instead). Empty fields keep the value from the
structured fields above, so a step stays readable and only what you type
changes:

| Step | Overridable | Effect |
|---|---|---|
| `drive` | `ros.topic`, `ros.rate`, `ros.linear.y/z`, `ros.angular.x/y` | publish a different message, on another topic, at another rate (`linear.x` and `angular.z` are the Speed and Turn rate fields above) |
| `goto` | `ros.action`, `ros.frame`, `ros.x/y/z/yaw` | send the goal to another action/service, in another frame, or to different coordinates |
| `arm_*` | `arm.acc`, `arm.wait` | pass `mvacc` to the SDK and/or block until the arm got there |

The message **type** is fixed - the orchestrator publishes typed messages.
For anything else, use a raw command step.

Pointing a `drive` step at `/robot/cmd_vel_user` also makes the orchestrator
do the control-mode handshake (set mode USER, publish, give control back),
exactly like on the real robot. That is a good way to test the real command
path without hardware; with `SIM_DRIVE_MODE=user` in `.env`, *every* drive
step does it.

### A raw command step

**+ Raw command** adds a step whose body is a shell command. It runs in the
orchestrator container with ROS 2 sourced (`bash -lc`), on the ROS domain of
the target, and passes when its exit code is the expected one (`expect`,
default 0). **Simulation only** / **Real robots only** give a per-target
variant; leave them empty to run the same command everywhere:

```yaml
  - name: "Raw: publish a zero Twist by hand"
    type: command
    run: 'ros2 topic pub --once /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"'
    expect: 0
    timeout: 20
```

The command runs in its own process group: **Abort**, a timeout or the end of
the step kills it, so a `ros2 topic pub -r 10` cannot keep driving the robot
after the mission stopped. `sample_7_raw_commands.yaml` uses all of this.

## 8. Run the same mission on the real robots

A mission says where it runs with **Run on: Sim | Real | Both** in the header
(saved in the mission file as `target`). `Sim` is the default, so a mission
file stays portable.

| Target | The robot | The arm | Checked with |
|---|---|---|---|
| `sim` | Gazebo Origin One, `/robot/cmd_vel` | UFACTORY Studio arm at 192.168.100.200 | the simulation's odometry and joint angles |
| `real` | real Origin One, over the zenoh bridge (none until `REAL_ROBOT_IP` is set) | real xArm at `REAL_XARM_HOST` | the real robot's odometry/global pose and the real arm's angles |
| `both` | every robot that exists (the simulation while `REAL_ROBOT_IP` is empty) | both at once | the simulation (the real robots get the same commands) |

Switching to Real or Both asks for confirmation and connects to the real robot
straight away, so the header chips show whether it is reachable *before* you
start. Starting a mission on the real robots asks again.

### Dress rehearsal: simulated Avular + real arm

The real target exists with only its arm: the arm speaks SDK-over-TCP
directly (no zenoh bridge needed), while its mobile base only exists once
`REAL_ROBOT_IP` is set and the bridge runs. So with the real arm connected
but no Avular yet:

- a `target: real` mission runs its arm steps on the real arm; a drive step
  in it fails immediately with "no mobile base for this target" instead of
  waiting for a robot that is not there;
- a `target: both` mission becomes a **dress rehearsal**: every drive step
  goes to the simulated Avular in Gazebo, every arm step also goes to the
  real arm and is checked on both arms.

`sample_9_rehearsal.yaml` is exactly that: sample 1's delivery sequence,
with the simulated Avular doing the driving and the real arm folding out to
serve. When the Avular is connected later (`REAL_ROBOT_IP` + the bridge),
the same mission automatically drives both robots.

The real arm can also be used from UFACTORY Studio itself: open
<http://localhost:18333> and add the arm with its IP (`192.168.1.229` when it
is on this machine's Ethernet) to see and jog it there.

### What is different on the real robot

The real Origin One is **not** driven like the simulation:

1. Take control of the mobile base: `ros2 service call
   /robot/cmd_vel_controller/set_control_mode origin_msgs/srv/SetControlMode
   "{mode: {mode: 30}}"` (30 = USER).
2. Only then publish `geometry_msgs/msg/Twist` on
   **`/robot/cmd_vel_user`**, 10x per second, for the duration of the step.
3. When the step is over, send a zero Twist and give control back with
   `/robot/cmd_vel_controller/reset_control_mode`.

Navigation is different too: the real robot plans and drives routes itself
with its **autopilot**, so a `goto` step becomes a `MOVE_THROUGH_POSES`
behavior:

```bash
ros2 service call /autopilot/information_manager/set_and_execute_behaviors \
  knowledge_base_msgs/srv/SetAndExecuteBehaviors \
  "{behaviors: [{type: 10, goals: [{header: {frame_id: map}, pose: {...}}]}]}"
```

The orchestrator then waits until the robot's global pose
(`/autopilot/estimated_pose`) is within 0.6 m of the goal. That service does
not take a cancel: if a `goto` step is aborted or times out, the log says so -
stop the robot from Cerebra Studio or with its remote. (The simulation keeps
using Nav2, which does support cancel.)

The arm is the same code path on both sides: the xArm Python SDK
(`set_servo_angle`), with `REAL_XARM_HOST` instead of the Studio container.
The **poses** are not the same, though: the real xArm has a gripper mounted
at its front, so it cannot take the all-zero home position. Its standard
("initial") position is

| J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|
| 0 | -90 | 0 | 0 | 90 | 0 |

and that is where a real mission parks and drives. The mission model refuses
`arm_home` (and all-zero `arm_joint` angles) on `target: real` or `both` for
that reason. `sample_8_real_coffee_delivery.yaml` is sample 1's workflow on
the real robots: it makes the standard position the `transport` pose with

```yaml
arm_poses:
  transport: [0, -90, 0, 0, 90, 0]
```

so the "arm tucked in" driving rule checks the real arm against the position
it actually drives in. The other built-in poses (`serve_right`, ...) come
from the simulation: check them with the RAW button and, if the gripper
changes what the arm can reach, put different angles in the mission's
`arm_poses`.

To confirm the arm **positions** on the real arm first - before connecting
the arm to the rest with ROS - there is a small script for the Python IDE of
UFACTORY Studio, one function per position:

```bash
py -3 tools/xarm_positions.py               # every position, one at a time
py -3 tools/xarm_positions.py serve_right   # or just one
```

Each move is checked with the same 3 deg tolerance a mission uses, and the
script stops at the first position that fails, so a problem is seen
immediately.

A note about the simulation: the simulated arm starts straight up
(all-zero) and folds into the standard position without a problem, so put
it there before a `both` run to match the real arm. An earlier standard
position, [0, -90, 0, 180, 180, 0], tripped the arm's self-collision
protection (controller error 22) on the way. A move the controller stops
halfway can leave the simulated arm stuck in that error; `docker restart
uf_software` resets it to straight up.

### Connecting to the real robot (zenoh bridge)

The robot's ROS 2 network is a local network *on* the robot. A computer joins
it through a zenoh bridge (the robot runs `zenoh-bridge-ros2dds` 1.0.3):

```bash
# .env
#   REAL_ROBOT_IP=192.168.100.1      the robot's IP (or its access point)
#   REAL_XARM_HOST=192.168.100.50    the real xArm control box (empty = no arm)
docker compose --profile real up -d   # starts coffe-zenoh-bridge
```

The bridge runs in its own ROS 2 domain (`REAL_ROS_DOMAIN_ID`, default 42), so
it cannot mix with the simulation's domain 0. The orchestrator joins that
domain with a second ROS 2 context and uses it only for `target: real|both`.

| Environment variable | Default | Meaning |
|---|---|---|
| `SIM_DRIVE_MODE` | `direct` | `user` = drive the *simulation* like the real robot (USER mode + `/robot/cmd_vel_user`) |
| `REAL_ROBOT_IP` | `192.168.100.1` | the Origin One, for the zenoh bridge |
| `REAL_ROS_DOMAIN_ID` | `42` | ROS 2 domain between the orchestrator and the bridge |
| `REAL_XARM_HOST` | empty | real xArm 6 control box; empty = this target has no arm |
| `REAL_CMD_VEL_TOPIC` | `/robot/cmd_vel_user` | where the drive messages go |
| `REAL_MODE_SERVICE` / `REAL_MODE_RESET_SERVICE` | `.../set_control_mode` / `.../reset_control_mode` | the control-mode handshake |
| `REAL_CONTROL_MODE` | `30` | 30 = USER |
| `REAL_ODOM_TOPIC` | `/robot/odom` | the robot's own odometry |
| `REAL_POSE_TOPIC` | `/autopilot/estimated_pose` | global pose, for the arrival check |
| `REAL_BEHAVIOR_SERVICE` | `/autopilot/information_manager/set_and_execute_behaviors` | navigation |

All of it lives in `orchestrator/targets.py`, so a different lab network is a
change in `.env`, not in the code.

### What is verified here, and what is not

- Verified live in the simulation: the control-mode handshake
  (`SIM_DRIVE_MODE=user`), driving over `/robot/cmd_vel_user`, the raw
  overrides, the raw command steps (including kill on abort), and every
  command line the RAW view shows.
- Verified against the real interfaces: the Avular `origin_msgs` and
  `knowledge_base_msgs` definitions are built into the orchestrator image from
  the Avular example workspace (pinned commit), and the xarm_ros2 service call
  is checked against `xarm_msgs/srv/MoveJoint`.
- **Not** verified (there is no hardware here): the TCP connection to the
  robot's zenoh bridge, and the arrival tracking of a real autopilot behavior.
  Try it with the robot powered up and the area clear, starting with a mission
  that only drives a short distance (`min_distance` small), before running a
  full mission.

## 9. Write missions by hand

Missions are plain YAML files, so you can also write or edit them in a text
editor. They live in `Simulation-Docker/orchestrator/missions/`. Create a
new `.yaml` file there and it appears in the dashboard dropdown within a
second, with no rebuild and no restart:

```yaml
name: "My first mission"
steps:
  - name: "Drive forward for 60 s"
    type: drive
    duration: 60            # seconds (simulation time; see clock below)
    linear: 0.25            # m/s, negative = backwards
    angular: 0.05           # rad/s, small value = drive a wide arc
    timeout: 300            # optional: wall-clock seconds before FAIL
    check:
      min_distance: 2.0     # the odometry must move at least 2 m

  - name: "Arm: joint 1 to 90 degrees"
    type: arm_joint
    joint: 1                # 1..6, or use angles: [90, 0, 0, 0, 0, 0]
    angle_deg: 90
    speed: 30               # degrees/s, like in Studio
    check:
      tolerance_deg: 3      # pass when every commanded joint is this close

  - name: "Drive to the coffee corner"
    type: goto
    waypoint: Home          # from the Cerebra map (goto list shows them all)
    # ...or map coordinates:  x: 2.06  y: -1.55  yaw: 0.0

  - name: "Arm back to home"
    type: arm_home          # all joints to 0

  - name: "Give the planner a second"
    type: wait
    duration: 2

  - name: "Drive 5 s with the raw Twist fields spelled out"
    type: drive
    duration: 5
    linear: 0.25
    ros:
      rate: 5                     # publish at 5 Hz instead of 10
      linear: {y: 0.0, z: 0.0}    # the raw message fields
    check: {min_distance: 0.4}

  - name: "Raw: publish a zero Twist by hand"
    type: command
    run: 'ros2 topic pub --once /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"'
```

Top-level options:

| Option | Values | Meaning |
|---|---|---|
| `name` | string | shown in the dashboard |
| `target` | `sim` (default) / `real` / `both` | where the mission runs (see part 8) |
| `require_arm_tucked` | `true` / `false` (default) | drive/goto steps fail unless the arm is in the `transport` pose |
| `arm_poses` | `{name: [6 angles]}` | extra or changed named poses for `arm_pose` steps (a real mission changes `transport` to the real arm's standard position) |
| `clock` | `sim` (default) / `wall` | `sim` = step durations count Gazebo time; `wall` = real time |

Step types and their checks:

| Type | Options | Passes when |
|---|---|---|
| `drive` | `duration`, `linear`, `angular`, `check.min_distance` | duration elapsed **and** odometry moved ≥ `min_distance` (if set) |
| `goto` | `waypoint` or `x`/`y`/`yaw` | Nav2 reports the goal reached |
| `arm_joint` | `joint`+`angle_deg` or `angles` (6 values), `speed`, `check.tolerance_deg` | every commanded joint is within `tolerance_deg` of its target |
| `arm_home` | `speed` | all six joints within `tolerance_deg` of 0 - **simulation only**: refused on `target: real`/`both`, the real arm's gripper cannot take all-zero (use `arm_pose`, e.g. `standard`) |
| `arm_pose` | `pose` (a name), `speed`, `check.tolerance_deg` | all six joints within `tolerance_deg` of the pose |
| `wait` | `duration` | the time passed |
| `command` | `run` (or `sim_run`/`real_run`), `expect` (0), `timeout` (30) | the command's exit code is `expect` |

Every step also takes the raw overrides of part 7: `ros: {...}` on `drive` and
`goto`, `arm: {acc, wait}` on the arm steps. They change the fields of the
message that goes out (topic, rate, extra Twist fields, goal action/frame/pose,
SDK acceleration); the message type is fixed.

Notes:

- Unspecified joints keep their current angle during an `arm_joint` step.
- A `command` step is free text and runs as a shell command with ROS 2
  sourced, so it can do anything the step types cannot - but it is only
  checked by its exit code.
- `goto` needs the `origin-nav` container (Nav2) and, for waypoints by
  name, a loaded Cerebra map (`CEREBRA_MAP` in `.env`).
- Blind `drive` steps can hit a wall - that is what `min_distance` is for:
  the mission fails honestly instead of pretending.

## 10. How the checking works

The orchestrator (`orchestrator/engine.py`) runs one step at a time:

1. **Run** the step (publish `cmd_vel`, send the Nav2 goal, or command the
   arm over the SDK).
2. **Verify** it with the ground truth of the *other* end of the wire:
   the drive check reads `/robot/odom` (does not trust "command sent"),
   the arm check reads `arm.angles` (does not trust the SDK return code),
   the goto check waits for Nav2's action result.
3. Only a **PASS** moves to the next step. A **FAIL** stops the mission,
   skips everything after it and shows the reason on the dashboard and in
   the log. A per-step `timeout` (wall clock) is the safety net if a
   simulation dies mid-step.
4. The robot is always stopped (zero `cmd_vel`) when a step ends, when the
   mission ends, and when you press **Abort**.

Because of this, "the mission passed" means: **the robot really drove and
the arm really moved, in the order you asked, verified by measurement.**

## 11. Troubleshooting

- **Dashboard says "NO SERVER"** - `docker compose ps`: is `coffe-orchestrator`
  running? `docker compose logs coffe-orchestrator` for the reason.
- **Robot dot is red ("not connected")** - the sim is not publishing
  `/robot/odom` (still starting up?), or you restarted it: give it a moment.
  The odometry also stops when Gazebo is paused.
- **Arm dot is red** - `ufactory-studio` not running yet, or still starting
  (the orchestrator retries every ~8 s). Check <http://localhost:18333>.
- **A drive step fails with "only moved X m"** - the robot hit something
  (wall? keep-out?) or you set `linear` too low for your `min_distance`.
  Watch the Gazebo view while the mission runs.
- **A `goto` step fails "Nav2 action server not reachable"** - the
  `origin-nav` container is down: `docker compose up -d origin-nav`.
- **"no route" on a goto step** - the goal is inside a `no_go_area` zone;
  that is Nav2's keep-out filter doing its job (see the main README).
- **An arm step fails "timed out, still X deg from the target"** - the arm
  stopped mid-move (collision in Studio? error state?). Jog it in Studio,
  then re-run the mission.
- **Everything is slow** - the sim runs at ~0.5x real time; the factor is
  in the dashboard header. Lower the lidar load with `HEADLESS: "true"`
  in `docker-compose.yml` if you only need the dashboard.
- **A raw command step fails with "exit N, expected 0"** - the command itself
  is in the log (`[sim] $ ...`), with its last output line in the step detail.
  Run it by hand in the container to see the full output:
  `docker exec -it coffe-orchestrator bash -lc 'source /opt/ros/humble/setup.bash; <command>'`.
- **The real robots stay "not reachable"** - the zenoh bridge is not running
  (`docker compose --profile real up -d`), `REAL_ROBOT_IP` is wrong, or the
  robot is not on the network. `docker logs coffe-zenoh-bridge` says which.
- **The real robot does not move** - look for `took control` in the dashboard
  log. Without it the control-mode handshake failed (the robot was in another
  mode, e.g. manual). You can also do it by hand:
  `ros2 service call /robot/cmd_vel_controller/set_control_mode origin_msgs/srv/SetControlMode "{mode: {mode: 30}}"`
  from inside `docker exec -it coffe-orchestrator bash`.
- **A real `goto` step times out** - the autopilot keeps driving the behavior
  it was given (it cannot be cancelled through the same service). Stop the
  robot from Cerebra Studio or with its remote, then check the map is loaded
  and the robot is localised.
- **Units reminder** - `linear`/`angular` are m/s and rad/s; arm angles are
  **degrees** (the xarm_ros2 command shown in the RAW view is in radians);
  mission `duration` is seconds (sim time by default).
