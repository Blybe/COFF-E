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
| [6. Write your own mission](#6-write-your-own-mission) | the step types and all options |
| [7. How the checking works](#7-how-the-checking-works) | what "PASS" actually means |
| [8. Troubleshooting](#8-troubleshooting) | |

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
| **Header** | mission dropdown, Start/Abort, overall state, whether robot + arm are connected, the sim-time factor, links to Gazebo/RViz/Studio |
| **Mission steps** | every step with live progress and its check result (green = verified, red = failed, rest skipped) |
| **Origin One** | live position/heading/speed, the driven trajectory on a map (waypoints of the Cerebra map included), button to embed the Gazebo view |
| **xArm 6** | live joint angles with target markers, a stylized arm sketch that moves with J2/J3, button to embed UFACTORY Studio |
| **Log** | what the orchestrator did, step by step |

The robot panel plots **odometry relative to the spawn point**, which is
where the robot spawned (the `Home` waypoint by default).

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

## 6. Write your own mission

Mission files live in `Simulation-Docker/orchestrator/missions/`. Create a
new `.yaml` file there and it appears in the dashboard dropdown within a
second - no rebuild, no restart:

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
```

Top-level options:

| Option | Values | Meaning |
|---|---|---|
| `name` | string | shown in the dashboard |
| `clock` | `sim` (default) / `wall` | `sim` = step durations count Gazebo time; `wall` = real time |

Step types and their checks:

| Type | Options | Passes when |
|---|---|---|
| `drive` | `duration`, `linear`, `angular`, `check.min_distance` | duration elapsed **and** odometry moved ≥ `min_distance` (if set) |
| `goto` | `waypoint` or `x`/`y`/`yaw` | Nav2 reports the goal reached |
| `arm_joint` | `joint`+`angle_deg` or `angles` (6 values), `speed`, `check.tolerance_deg` | every commanded joint is within `tolerance_deg` of its target |
| `arm_home` | `speed` | all six joints within `tolerance_deg` of 0 |
| `wait` | `duration` | the time passed |

Notes:

- Unspecified joints keep their current angle during an `arm_joint` step.
- `goto` needs the `origin-nav` container (Nav2) and, for waypoints by
  name, a loaded Cerebra map (`CEREBRA_MAP` in `.env`).
- Blind `drive` steps can hit a wall - that is what `min_distance` is for:
  the mission fails honestly instead of pretending.

## 7. How the checking works

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

## 8. Troubleshooting

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
- **Units reminder** - `linear`/`angular` are m/s and rad/s; arm angles are
  **degrees**; mission `duration` is seconds (sim time by default).
