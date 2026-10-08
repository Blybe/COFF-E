# COFF-E Simulation Stack (Docker)

Docker simulation of the COFF-E robot, so the project can be developed and
tested before the real hardware is available:

| Component | Simulation | Access |
|---|---|---|
| **Avular Origin One** (mobile base) | ROS 2 Humble + Gazebo Fortress, built from the official Avular sources | Gazebo GUI in the browser: <http://localhost:6080> |
| **Nav2 + RViz** (path planning) | Navigation 2 stack on the Cerebra map (same image, second container) | RViz in the browser: <http://localhost:6081> |
| **UFACTORY xArm 6** (robot arm) | UFACTORY Studio + simulated xArm 6 firmware (official UFACTORY Docker image) | UFACTORY Studio web UI: <http://localhost:18333> |

Everything runs from one `docker compose up -d --build`, on Windows, macOS or
Linux. Nothing ROS-related needs to be installed on the host.

## Folder contents

```text
Simulation-Docker/
├── docker-compose.yml      # starts the three services
├── .env                    # selects the Cerebra map + spawn waypoint
├── avular-origin/
│   ├── Dockerfile          # ROS 2 Humble + Gazebo Fortress + Avular sim + Nav2
│   └── entrypoint.sh       # display/VNC setup + launches the simulation
├── nav/                    # Nav2 + RViz for the Origin One (mounted into origin-nav)
│   ├── origin_nav.launch.py      # Nav2 launch: costmaps, AMCL, keepout, RViz
│   ├── origin_nav_params.yaml    # Nav2 tuning (merged over the stock defaults)
│   ├── origin_nav.rviz           # RViz view (map, robot, plan, waypoints, zones)
│   ├── cerebra_markers.py        # waypoints/paths/zones as RViz markers
│   └── goto.py                   # the `goto` command (see "Path planning")
├── tools/
│   ├── cerebra_import.py   # Cerebra data -> sim world + Nav2 map (+ --list)
│   ├── cerebra_map_to_world.py  # Cerebra map -> Gazebo world (stand-alone usable)
│   └── sync_cerebra.ps1    # Cerebra Studio <-> ../cerebra-data (Git sharing)
├── tests/
│   ├── test_cerebra_import.py   # importer unit tests (py -m unittest ...)
│   ├── origin_drive_test.sh     # drives the Origin One, checks odometry
│   └── xarm_smoke_test.py       # moves the simulated xArm 6 via the SDK
├── .gitignore              # ignores build/cache scratch (tmp/, __pycache__)
└── .gitattributes          # keeps shell scripts LF-only (Windows-safe)

../cerebra-data/            # shared maps, waypoints, paths, zones (see its README)
```

## Shared Cerebra Studio data (maps, waypoints, paths, zones)

The simulation runs on the same data as the real Origin One. The
`cerebra-data/` folder next to `Simulation-Docker/` (inside the repository)
mirrors Cerebra Studio's Mission Planner data: maps, waypoints, paths and
zones (`go_area`, `no_go_area`, `cover_area`, `map_layout`), one JSON file per
item. Everyone on the team gets the same data through Git, without needing
Cerebra Studio installed.

- **Publish data from Cerebra Studio** (laptop that has Studio):

  ```powershell
  powershell -ExecutionPolicy Bypass -File Simulation-Docker\tools\sync_cerebra.ps1
  git add cerebra-data
  git commit -m "Update Cerebra data"
  git push
  ```

- **Receive data into your own Cerebra Studio** (close Studio first):

  ```powershell
  git pull
  powershell -ExecutionPolicy Bypass -File Simulation-Docker\tools\sync_cerebra.ps1 -Direction Import
  ```

The script only overwrites files whose content differs, and never deletes
anything unless you pass `-Prune`. See `cerebra-data/README.md` for details.

### Selecting the map and spawn point

`Simulation-Docker/.env` picks the map (by name or id) and the waypoint where
the robot spawns:

```ini
CEREBRA_MAP=JMH - 3rd floor
CEREBRA_SPAWN=Home
```

List the available maps with `py tools\cerebra_import.py --list` (run in the
`Simulation-Docker` folder). With `CEREBRA_MAP` empty, the stack falls back
to Avular's own `TY_test_area` world and the `SIM_WORLD` / `ROBOT_POSE_*`
variables.

At startup, both services run `tools/cerebra_import.py` for the selected map:
the simulation gets a Gazebo world (walls from the occupancy grid, markers on
the waypoints), and the nav service gets the matching Nav2 map, an AMCL
initial pose at the spawn waypoint, and keep-out masks for the `no_go_area`
zones. The world uses the same coordinates as Cerebra Studio, so positions
match between the real robot and the simulation. After syncing new data,
`docker compose restart avular-origin-sim origin-nav` reloads it.

Limits: walls are only as good as the scan. Glass, doors that were open during
mapping, and unscanned (grey) areas have no walls, so the robot could drive
out through gaps. Draw `no_go_area` zones in Cerebra Studio (or clean up the
map) if that matters; the zones are keep-out areas for the planner.

## Path planning with Nav2 on the Cerebra map

The `origin-nav` service (second container, same image) runs the full Nav2
stack against the simulated Origin One: AMCL localisation on the Cerebra map,
global and local costmaps fed by the Ouster lidar (converted to a 2D laser
scan), the DWB controller, and a keep-out filter for `no_go_area` zones.
Cerebra waypoints, paths and zones are shown as RViz markers.

Open RViz (with the Nav2 toolbar) in the browser:
<http://localhost:6081/vnc.html?autoconnect=true&resize=scale> and set goals
with the **2D Goal Pose** tool, or use `goto` from any shell:

```bash
docker exec -it origin-nav goto list              # waypoints, paths, zones
docker exec -it origin-nav goto Home              # drive to a Cerebra waypoint
docker exec -it origin-nav goto path "Corridor"    # follow a Cerebra path (any recorded path)
docker exec -it origin-nav goto 3.5 -1.2 3.14     # drive to map coordinates x y [yaw]
```

`goto` exits 0 when the goal was reached, 1 when it failed. Driving to a
waypoint plans a route (like Cerebra's MOVE_THROUGH_POSES); `goto path`
drives along the recorded poses in order (like MOVE_ALONG_PATH), so the robot
follows the exact line that was recorded. A goal inside a `no_go_area` zone
is refused with "no route" — that is the keep-out filter doing its job.

Notes:

- The sim runs at roughly 0.4-0.5x real time once the lidar is active, so
  driving takes about twice as long as on the real robot. The progress
  checker is tuned for that (see Troubleshooting).
- Cerebra Studio cannot drive the simulation — no Cerebra software runs in
  the containers. Use RViz or `goto`.
- Nav2 tuning lives in `nav/origin_nav_params.yaml`, merged over the stock
  Nav2 defaults. Edit it and run `docker compose restart origin-nav`.

## Smoke tests

Run these after `docker compose up -d` (wait about 30 s for startup):

```bash
# Importer (host, no Docker): Cerebra data -> Gazebo world + Nav2 map
py -m unittest tests/test_cerebra_import.py

# Origin One: drives 5 s forward, expects PASS
docker cp tests/origin_drive_test.sh avular-origin-sim:/tmp/origin_drive_test.sh
docker exec avular-origin-sim bash /tmp/origin_drive_test.sh

# xArm 6: connects with the SDK, moves joint 1 to 20 deg and back
docker run --rm --network coffe-simulation_coffe-net -v "${PWD}/tests:/tests" python:3.11-slim \
  bash -c "pip install -q xarm-python-sdk && python /tests/xarm_smoke_test.py 192.168.100.200"
```

## Prerequisites

- Docker Desktop (WSL2 backend on Windows) with the Compose plugin.
- About 10 GB free disk space for the two images.
- First build of the Avular image takes roughly 10-20 minutes (ROS 2 +
  Gazebo Fortress download and workspace build); after that it is cached.

## Quick start

```bash
cd Simulation-Docker
docker compose up -d --build
```

Then open:

- **Gazebo GUI (Origin One):** <http://localhost:6080/vnc.html?autoconnect=true&resize=scale>
  (click inside the window to control the camera, drag with the left mouse
  button to look around; the robot spawns on the `Home` waypoint of the
  selected Cerebra map, or in Avular's test area when `CEREBRA_MAP` is empty)
- **RViz (Nav2, path planning):** <http://localhost:6081/vnc.html?autoconnect=true&resize=scale>
  — drive with the **2D Goal Pose** tool or
  `docker exec -it origin-nav goto Home`
- **UFACTORY Studio (xArm 6):** <http://localhost:18333>
  If the web page shows *"Unable to get robot SN"*, click **Close** and the
  simulated xArm 6 is still fully usable.

Stop everything with `docker compose down` (add `-v` to also remove the
volumes).

## The Avular Origin One simulation

### What runs inside

The container builds the same setup as the Avular
[install instructions](https://github.com/avular-robotics/avular_origin_simulation):
ROS 2 Humble on Ubuntu 22.04, Gazebo Fortress, the `origin_one_gazebo` and
`origin_one_description` packages, the Git LFS meshes, and the closed-source
`cmd_vel_controller` deb packages so the simulation prioritises control
inputs exactly like the real Origin One.

**Pinned description version.** `avular_origin_description` is pinned to
commit `051deab` (Feb 2026, v1.2.1). The current `main` (v2.0.0, Jul 2026)
removed all Gazebo plugins (drive, lidar, camera, GNSS) from the URDF. With
that version, the robot spawns but has no drive and no sensors. The
Dockerfile also replaces that commit's `CMakeLists.txt` with a plain
`ament_cmake` one, because the original needs Avular's internal
`cmake_avular` package.

This is Gazebo **Fortress**, so the command-line tool is `ign` (for example
`ign topic -l`), not `gz`.

The Gazebo GUI is rendered on a virtual display inside the container and
streamed to the browser (Xvfb -> x11vnc -> noVNC). If you run Docker from
inside a WSL2 or Linux desktop, you can instead show the GUI natively, see
[Options](#options).

### Driving the robot (teleop)

Open a shell inside the running container and use the keyboard teleop from
the Avular README:

```bash
docker exec -it avular-origin-sim bash
# inside the container:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot/cmd_vel
```

Or as a one-liner (`bash -ic` loads the ROS environment from `.bashrc`):

```bash
docker exec -it avular-origin-sim bash -ic "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot/cmd_vel"
```

Teleop publishes on `/cmd_vel`; Nav2 (`origin-nav`) also publishes there via
a relay to `/robot/cmd_vel`. Stop one before using the other.

### Important ROS 2 topics

Bridged between Gazebo and ROS 2 inside the container:

| Topic | Type | Direction |
|---|---|---|
| `/robot/cmd_vel` | `geometry_msgs/msg/Twist` | ROS -> Gazebo (drive it) |
| `/robot/odom` | `nav_msgs/msg/Odometry` | Gazebo -> ROS |
| `/robot/lidar/points` | `sensor_msgs/msg/PointCloud2` | Gazebo -> ROS |
| `/robot/camera/color/image_raw` | `sensor_msgs/msg/Image` | Gazebo -> ROS |
| `/robot/camera/depth/points` | `sensor_msgs/msg/PointCloud2` | Gazebo -> ROS |
| `/robot/joint_states` | `sensor_msgs/msg/JointState` | Gazebo -> ROS |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo -> ROS |

Inspect them with:

```bash
docker exec -it avular-origin-sim bash -ic "ros2 topic list"
docker exec -it avular-origin-sim bash -ic "ros2 topic echo /robot/odom"
```

### RViz

RViz is installed in the container, with the Avular configuration. To look
at the robot model and sensor data through the same browser window:

```bash
docker exec -d avular-origin-sim bash -c "source /home/origin_ws/install/setup.bash && ros2 launch origin_one_description origin_one_rviz.launch.py"
```

(For path planning with RViz + Nav2, use the `origin-nav` service on port
6081 instead, see [Path planning](#path-planning-with-nav2-on-the-cerebra-map).)

### Saving a screenshot of the simulation

```bash
docker exec avular-origin-sim bash -c "DISPLAY=:1 import -window root /tmp/sim.png"
docker cp avular-origin-sim:/tmp/sim.png .
```

## The UFACTORY xArm 6 simulation

The `ufactory-studio` service uses the image from the official UFACTORY
article
["How to install UFACTORY Studio in Docker"](https://docs.ufactory.cc/support_articles/software/how-to-install-ufactory-studio-in-docker)
and starts a simulated xArm 6 firmware plus UFACTORY Studio automatically.

Use the web UI at <http://localhost:18333> to move the simulated arm,
jog joints and program with Blockly.

### Using the xArm Python SDK from the host

The published ports let code running on the host (or in another container)
control the simulated arm. Point the SDK at `127.0.0.1`:

```python
from xarm.wrapper import XArmAPI

arm = XArmAPI('127.0.0.1', baud_checkset=False, check_joint_limit=False)
arm.motion_enable(enable=True)
arm.set_state(0)
arm.set_position(x=300, y=0, z=200, roll=-180, pitch=0, yaw=0, speed=100)
```

`check_joint_limit=False` is required for Blockly-generated code, per the
UFACTORY docs. Install the SDK on the host with
`pip install xarm-python-sdk`.

From another container on the compose network (`coffe-simulation_coffe-net`),
use the fixed address `192.168.100.200`, the same way you would use the IP
of a real xArm control box:

```python
arm = XArmAPI('192.168.100.200', baud_checkset=False, check_joint_limit=False)
```

Use an IP address, not a hostname. The SDK treats any value that is not an
IP address as a serial port name.

### Simulating a different robot

UFACTORY ships several models in the same image; change the environment
variable in `docker-compose.yml` and restart:

```yaml
environment:
  XARM_CONFIG: "6 6"   # "5 5"=xArm 5, "6 6"=xArm 6, "7 7"=xArm 7, "6 9"=Lite 6, "6 12"=xArm 850
```

```bash
docker compose up -d ufactory-studio
```

## Ports

All ports are bound to `127.0.0.1` (localhost only).

| Port | Used by | Purpose |
|---|---|---|
| 6080 | avular-origin-sim | noVNC: Gazebo GUI in the browser |
| 6081 | origin-nav | noVNC: RViz + Nav2 in the browser |
| 18333 | ufactory-studio | UFACTORY Studio web UI + xArm SDK API |
| 502-504 | ufactory-studio | Modbus TCP |
| 30000-30003 | ufactory-studio | xArm status report ports |

### Fixed IP addresses

The containers sit on the network `coffe-simulation_coffe-net`
(subnet `192.168.100.0/24`):

| Container | IP |
|---|---|
| `avular-origin-sim` | `192.168.100.10` |
| `origin-nav` | `192.168.100.11` |
| `uf_software` (xArm 6) | `192.168.100.200` |

These addresses work from other containers on that network. From the
Windows host itself, keep using `127.0.0.1` (Docker Desktop does not route
container IPs to the host). If `192.168.100.x` clashes with a network you
use, change the subnet and the addresses in `docker-compose.yml`.

## Options

The map and spawn waypoint are set in `.env` (see
[Shared Cerebra Studio data](#shared-cerebra-studio-data-maps-waypoints-paths-zones));
the rest are set in `docker-compose.yml` under `avular-origin-sim.environment`
(fallback values when `CEREBRA_MAP` is empty):

| Variable | Default | Meaning |
|---|---|---|
| `CEREBRA_MAP` | `JMH - 3rd floor` (.env) | name or id of a map in `../cerebra-data/maps`; empty = Avular's test area |
| `CEREBRA_SPAWN` | `Home` | waypoint on that map where the robot spawns |
| `USE_CMD_VEL_CONTROLLER` | `True` | `True` makes the sim use the cmd_vel_controller package, like the real robot |
| `DRIVE_CONFIGURATION` | `skid_steer_drive` | `mecanum_drive` for the mecanum-wheel variant |
| `HEADLESS` | `false` | `true` runs the sim without GUI/VNC (sensors still work) |
| `SIM_WORLD` | `TY_test_area.world` | world file, only used when `CEREBRA_MAP` is empty |
| `ROBOT_POSE_X` / `_Y` / `_YAW` | `-18.75` / `-34.8` / `1.57` | robot spawn pose in the world (m, m, rad), only used without a Cerebra map |

**Native GUI instead of the browser:** uncomment `USE_HOST_DISPLAY: "1"`
and `DISPLAY: ":0"` in `docker-compose.yml` and run `docker compose up -d`
from inside your WSL2 distro (Windows 11 WSLg) or a Linux desktop session.
The Gazebo window then opens on your own desktop, and the VNC/noVNC stack
is skipped.

**Changing the robot spawn pose or world:** set `CEREBRA_SPAWN` in `.env`,
or empty `CEREBRA_MAP` and set `SIM_WORLD` / `ROBOT_POSE_*`.

## Troubleshooting

- **Simulation runs at ~0.5x real time**: expected once Nav2 subscribes to
  the lidar. The GPU lidar renders in software (llvmpipe), and the image
  pins the sensor's render engine to Ogre, which is roughly 7x faster than
  the default Ogre2 in this setup. Driving simply takes about twice as long
  as on the real robot; the Nav2 progress checker is tuned for it
  (`nav/origin_nav_params.yaml`).
- **`goto` or a Nav2 goal fails with "no route"**: the goal lies inside a
  `no_go_area` zone (the keep-out filter refusing it, as intended) or is
  unreachable on the map. Pick another goal, or update the zone in Cerebra
  Studio and re-sync.
- **First `docker compose up` does nothing / build fails**: run
  `docker compose build avular-origin-sim` without `-d` to see the full
  build log. The most fragile step is installing the Avular deb packages; if
  upstream publishes new filenames, update them in `avular-origin/Dockerfile`.
- **Gazebo window is black or slow**: the GUI renders with software OpenGL
  (llvmpipe) by default, which is slower than a GPU but fine for this world.
  Rendering is much lighter with `HEADLESS: "true"` if you only need topics.
- **Port already in use**: something else on your machine uses one of the
  published ports. Change the left-hand side of the mapping in
  `docker-compose.yml` (e.g. `"127.0.0.1:6082:6081"`).
- **Container keeps restarting**: check `docker compose logs <service>`.
- **`Unable to get robot SN` in UFACTORY Studio**: click **Close**; the web
  simulation still works (this is expected with the simulated firmware).
- **Robot spawns but does not drive / no lidar or camera topics**: the
  description repo was not pinned (see above). Check with
  `docker exec avular-origin-sim bash -ic "ign topic -l"`; `/robot/odom`
  and `/robot/lidar` must be listed.
- **Small "xmessage" popup in the VNC view**: comes from the fluxbox window
  manager menu and is harmless. Close it.
- **Reset everything**:
  `docker compose down` then `docker compose build --no-cache avular-origin-sim`.

## Relation to the COFF-E project

This stack covers the simulation part of building blocks A and B from
`Project-COFF-E/PROJECT_KICKOFF.md`:

- **xArm 6** can be controlled from Python via the SDK exactly like the real
  arm (same API, IP `127.0.0.1` instead of the control box).
- **Origin One** exposes the same ROS 2 topics as documented by Avular
  (`/robot/cmd_vel`, `/robot/odom`, LiDAR, camera), so navigation and
  alignment logic can be developed before the platform is in the lab.
- **Maps, waypoints, paths and zones** are shared through `cerebra-data/`
  in this repository (see `cerebra-data/README.md`), so the simulation plans
  on the exact map the real Origin One drives on, and the whole team sees
  the same data with or without Cerebra Studio.

Later, a COFF-E orchestrator container can be added to the same
`docker-compose.yml` and talk to all services over the compose network
(Origin One sim at `192.168.100.10`, Nav2 at `192.168.100.11`, xArm at
`192.168.100.200`).

## Sources

- Avular Origin simulation: <https://github.com/avular-robotics/avular_origin_simulation>
- Avular Origin description (URDF/meshes): <https://github.com/avular-robotics/avular_origin_description>
- Navigation 2: <https://docs.nav2.org/>
- UFACTORY Studio in Docker: <https://docs.ufactory.cc/support_articles/software/how-to-install-ufactory-studio-in-docker>
- xArm Python SDK: <https://github.com/xArm-Developer/xArm-Python-SDK>
