# COFF-E · Real Avular (ROS 2 only)

A separate, lightweight stack for the **physical Avular Origin One**. The
dashboard keeps the simulation dashboard's dark panel layout, but has no Gazebo,
Avular simulator, simulated xArm, UFACTORY Studio, arm SDK, RViz desktop, or
dependency on `Simulation-Docker/`. Camera/computer vision and arm control are
deliberately deferred.

## What runs where

```text
Browser at localhost:8082
          |
avular container: dashboard + ROS 2 Humble node + safety watchdog
          | local DDS domain 42, Cyclone DDS
zenoh-bridge container: zenoh-bridge-ros2dds 1.0.3
          | TCP 7447
Physical Avular: sensors + map storage + AMCL + onboard ROS 2 autopilot
```

**The physical robot must be switched on, connected and have its ROS 2 autopilot
running.** These two containers are a client of that autopilot, not a replacement
for it. Navigation uses its `MOVE_THROUGH_POSES` ROS 2 service. We deliberately
do not run another AMCL/Nav2 instance that would compete with the robot's TF,
localization and velocity control.

The browser supports:

- hold-to-drive forward/reverse/turn, using USER mode 30 and
  `/robot/cmd_vel_user`, not `/robot/cmd_vel`;
- list and load maps **already stored on the robot**, fetched over ROS 2;
- display the map, `/autopilot/estimated_pose`, filtered laser returns and
  the onboard planned path;
- **Load simulation map** (`SIM_MAP=JMH - 3rd floor`), matching by map ID,
  or show a clearly labelled preview if it is not stored onboard;
- show estimated distance driven and straight-line distance remaining,
  keeping the last navigation result after stopping;
- drag an initial position/heading, then explicitly confirm localization;
- drag a destination/heading or enter coordinates and send a real navigation
  request;
- stop, heartbeat loss handling, manual-command expiry and feedback checks.
- automatically detected robot identity, battery percent/voltage, ROS feedback freshness,
  and separate laptop and robot Wi-Fi readings.

### Driving to a place on Chud's map

Chud can only navigate on a map stored on Chud. The simulation map
("JMH - 3rd floor", 97.6 x 44.8 m, origin -50.4/-28.0) is **not** on Chud and
none of its 42 onboard maps has the same size or origin, so coordinates from the
simulation do not carry over. Use Chud's own third-floor map instead:

1. **Load Chud map** loads the detected robot's map shortcut
   (`JMH-03-D07-10-26-PT-1`, 59.2 x 30.4 m). Other maps are under "Other maps".
2. Localize: initial pose tool, drag, **Send initial pose**, wait until the pink
   laser dots match the walls, **Autopilot position** says accurate and
   **Laser on walls** is at least 40%, confirm.
3. Check that the header says **control: NONE** or **AUTOPILOT**, not
   **MANUAL (remote)**. In MANUAL the remote holds control and Chud refuses the
   dashboard's USER mode (`Requested control mode with id 30 is not one of the
   accepted control modes for mode 10`). Release control on the remote first.
4. **Enable real base**, click a saved place or drag a goal, tick the
   destination box, **Drive to destination**. The **Pre-flight** box lists
   only the conditions that are still missing, each with a button that jumps
   to the step that fixes it.

The left column is a four-step wizard (**Map → Localize → Enable → Drive**).
Only one step is shown at a time; the bar at the top shows each step's state
and can be clicked to jump to any step. When the shown step completes, the
wizard moves on to the next one. Dragging on the map opens the matching step
(initial pose → Localize, goal → Drive). The safety checks themselves are
unchanged and enforced by the server, not by the wizard.

**Display controls:** the dashboard uses the full window width. **Compact**
reduces spacing; **Focus map** hides the side columns (click **Show panels**
to restore them). Connection, feedback, manual drive and the activity log
can each be collapsed by clicking their heading. These choices and the
**Theme** selection are remembered in this browser. Themes include Midnight,
Forest, Violet and Light. **Robot (automatic)** uses a red-to-green gradient
for Ronaldo and a brown palette for Chud, using fresh Wi-Fi detection and a neutral accent if the
robot cannot be detected. Colour is not proof of
robot identity. STOP remains visible in every layout; hiding the manual
controls ends any held manual command without enabling or starting motion.

**Saved places** are stored per onboard map ID in `places/places.json`
(mounted into the container, kept across rebuilds). In step 4 they are a
searchable list with each place's coordinates, heading and distance from the
robot; ✎ renames a place and × deletes it. With **Pick on map** active,
clicking a purple marker on the map selects that place. Save the current blue
goal, or the robot's position once localization is confirmed, under
"Save a new place"; saving under an existing name updates that place.
Selecting a place only selects it; driving still needs the checkbox and the
drive button.

### Chud and Ronaldo

**Chud** has an access-point SSID ending in **RK5** (`Origin PB4U9RK5`).
**Ronaldo** has SSID `Origin PB4U9CR7`. Both hotspots give the robot
192.168.192.1, so connect only one
adapter to a robot at a time.

The running `Watch-Wifi.ps1` monitor automatically finds the adapter connected
to either exact SSID, even with another adapter on eduroam. Within the next
monitor/dashboard updates, the name, automatic theme and map shortcut follow
that robot, without editing `.env`:

Click **Detect robot** under **Robot & connection** to refresh immediately.
The result names the robot, SSID and adapter, or explains missing/stale/ambiguous
monitoring. It only reads the latest host snapshot, never changes Wi-Fi, loads a
map, enables motion or sends a goal. Keep the Windows monitor running.

| Robot | `ROBOT_NAME` | `ROBOT_WIFI_SUFFIX` | `ROBOT_MAP_ID` / `ROBOT_MAP_NAME` |
|---|---|---|---|
| Chud | Chud | RK5 | `b3704c57-6e5b-49dc-b121-ab55ecae9857` / JMH-03-D07-10-26-PT-1 |
| Ronaldo | Ronaldo | CR7 | `da573728-35da-4fd2-9784-a211f9850d1b` / JMH - 3rd floor |

Stop and disable motion **before** switching hotspots. After switching, reconnect
the bridge with `docker compose restart zenoh-bridge` and reload the dashboard
and onboard map before checking localization again. Detection alone does not
reconnect a stopped bridge, clear a previously loaded map, or verify ROS feedback
belongs to that robot. Never switch during driving.
When the chosen map is already active onboard, **Load map** fetches it
without reloading it, so a robot that is already localized keeps its pose:
check the laser dots and confirm without sending an initial pose.

`ROBOT_NAME` remains configuration metadata. `ROBOT_MAP_ID` / `ROBOT_MAP_NAME`
override the shortcut only when the detected robot matches `ROBOT_WIFI_SUFFIX`.
Stale/missing monitoring, an unknown/disconnected SSID or two robot adapters
produce **Unknown robot**, a neutral automatic theme and no map shortcut,
not a fallback to the last configured robot. Other maps can still be selected.
Wi-Fi detection is not a verified identity handshake and does not change routes.
Verify the target IP, Wi-Fi and physical robot before enabling motion.

Battery comes from `/robot/battery/info` (`origin_msgs/msg/BatteryInfo`).
The robot's `/robot/origin_network_telemetry/network` reports its own Wi-Fi
**client**, which may say “Not connected” while your laptop is connected to
the robot's access point. It is not the laptop signal strength.

For live **laptop** SSID and signal, leave this read-only Windows monitor running
in another PowerShell window:

```powershell
.\Watch-Wifi.ps1                    # automatically find the robot adapter
# Optional: restrict detection to a named adapter
.\Watch-Wifi.ps1 -Adapter 'WiFi'
```

It reads `netsh wlan show interfaces` every 3 seconds and writes only adapter,
SSID, connection state, signal percent, ambiguity flag and timestamp to gitignored
`status/wifi.json`, mounted read-only into Docker. It never requests keys.
Ctrl+C stops the monitor; after 15 seconds the dashboard marks it unavailable.
This parser expects the English Windows field names. Without the monitor,
ROS battery/feedback and robot network telemetry still work. Automatic identity
and map-shortcut selection never load a map, enable motion or send a goal.
This is not a safety-rated identity interlock.

**Local PGM/YAML or Cerebra JSON file upload is not implemented.** If your map
only exists in `../cerebra-data`, first import it into the robot's knowledge base
using Avular's supported workflow. Cerebra is not required while operating this
dashboard, but creating/importing the first onboard map is an existing robot
setup task. Loading an onboard map here calls `load_localization_map`.
The simulation-map button only does that if the same ID exists onboard.
Otherwise it draws the read-only local JSON as **PREVIEW ONLY**, blocks
localization/navigation on it and hides live overlays to avoid implying
an unverified coordinate match. It does not upload or alter an onboard map.
The selected map is configured independently in this stack's `.env`; update
`SIM_MAP` if you change `Simulation-Docker/.env`.

On Chud, neither the simulation's `JMH - 3rd floor` ID nor the other local
third-floor map ID appeared in the onboard map list during inspection.
Use Cerebra Studio's supported import workflow before navigating on that map.

## First start (Windows PowerShell)

Start Docker Desktop with **Linux containers**, connect your laptop to the
Avular network, and open PowerShell:

```powershell
cd C:\Users\robin\MinorAutoSys\Robotic-Projects\Project-COFF-E\Real-Avular-Docker
Copy-Item .env.example .env
notepad .env
```

Set `REAL_ROBOT_IP` to the reachable robot IP, without `tcp/` or a port.
Leave `LOCAL_ROS_DOMAIN_ID=42` only if that differs from the domain **inside
the robot**. The bridge and dashboard must share the local domain.
Confirm `NAV_FRAME` using the robot's estimated pose/map frames. The value
`map` is a default to verify, not a discovered hardware fact.

```powershell
docker compose config --quiet
docker compose up -d --build
docker compose logs -f
```

Open **http://localhost:8082**. Starting the stack does **not** enable motion,
take control, change maps, seed a pose, or send a goal.

The first build downloads ROS packages and compiles the same pinned Avular
message definitions used by the existing stack. Internet access is needed.
No xArm connection is made.

If you also run the simulation, keep distinct ROS domains and dashboard ports.
Stop any other laptop/WSL Zenoh bridge connected to this same robot, and do not
operate two controllers against it. Do not copy the simulation's fixed Docker
subnet: it may overlap the real robot's network.

## Operator workflow

First tests require supervision, an empty marked area and a tested physical
emergency stop. Secure the mounted arm in its approved transport pose. There is
no electronic arm interlock in this base-only stack.

1. **Inspect feedback.** Odometry and emergency-brake feedback must be fresh
   (under 2 seconds), and the brake must be clear. A topic being listed alone
   does not establish this.
2. **Disable onboard jobs/scheduling and other controllers** using the approved
   Avular procedure. Never assume that taking USER control deletes queued jobs.
3. **Refresh maps**, select an onboard map and click **Load**. This requests
   USER zero hold, required onboard `stop_execution` and optional legacy
   `abort`, then loads the map.
   It refuses to continue if stop is unconfirmed. Changing maps invalidates
   the previous localization confirmation.
4. Select **Initial pose tool**. Drag from the robot's actual map position
   toward the direction it faces (yellow arrow) and click **Send initial
   pose**. If you dragged with the goal tool by mistake, **Use blue goal marker
   as initial pose** sends the blue arrow instead. Sending works while enabled
   or stopped, as long as Chud is held at zero speed and standing still. A click
   without a drag selects yaw zero. The note below the buttons says whether the
   green estimate jumped to the pose you sent.
5. Wait for new estimated-pose feedback. Inspect the position/heading against
   the real surroundings, check the localization checkbox and click
   **Confirm localization**. Compare the **pink laser points** to map walls.
   The **cyan line** shows the onboard plan when published. If you cannot
   establish the robot is correctly localized, do not confirm or navigate.
   Freshness, frame identity and the laser match are additional gates, not
   proof that localization converged. Chud localizes with its onboard
   gridslam (AMCL is unconfigured), whose reported covariance stays high even
   when the pose is right. Confirmation and every goal need the autopilot's
   own `position_accuracy_status` (`/autopilot/information_manager/system_status`)
   to be `accurate`: before that, goals are queued onboard and start later
   by themselves. Confirmation also needs at least 40% of the fresh laser
   points within 0.15 m of an occupied map cell (measured: right pose
   48-83%, wrong pose 12-17%). The covariance limits (0.25 m², 0.30 rad²)
   only apply when no laser overlay is available. During navigation the run
   stops when fewer than 25% of the points match for more than 3 seconds.
   The initial pose goes to `/initialpose` (pose filter) and
   `/autopilot/amcl/initial_pose` (AMCL, transient-local), five times 0.3 s
   apart, stamped with Chud's odometry clock. Its covariance is 0.005 while
   the autopilot is not accurate (its start-up step only finishes below 0.01)
   and 0.05 / 0.03 rad² when re-localising. AMCL only corrects after about
   10 cm or 0.1 rad of motion and never searches the whole map, so the click
   must be on the right side of doors with the right heading.
   **Auto-fit yellow arrow to the laser** searches ±1.5 m / ±60° around your
   yellow arrow for the best laser fit. It never searches the whole map:
   rooms have look-alike spots (on Chud two poses 3.5 m apart both fit
   ~73%), and it warns when another nearby pose fits almost as well. It only
   moves the yellow arrow; the yellow dots show the live scan as it would lie
   there. Check them, then send.
   The laser overlay needs `base_link -> ouster_base`. The bridge delivers
   only part of Chud's `/tf_static`, so the dashboard reads the fixed joints
   from `/robot/robot_description` into its own TF buffer (nothing is
   published to the robot).
6. Complete the safety checklist and click **Enable real base**. Enabling clears
   autonomous work before taking USER zero hold. Manual drive is available
   without loading/localizing a map: hold a direction button to move.
7. For autonomous driving, select **Goal tool**, drag a destination/heading
   inside free map space, check the destination confirmation and click
   **Send navigation goal**. Coordinates are in the selected map frame; yaw
   is radians, counterclockwise from +X. Sending hands USER control back to
   NONE and asks the onboard autopilot to navigate.
8. Stay with the robot and keep the tab visible. **STOP / HOLD** (or Escape)
   disables further dashboard motion, takes USER zero hold and requests both
   onboard stop services. Re-enable explicitly for another movement.
9. After a verified stop, **Release USER control after stop** resets USER to
   NONE. It does not restore a previous control mode. If needed for normal
   robot operation, restore that mode with the approved Avular controls.

Manual speed is set with the **Forward** and **Turn** sliders (remembered in
the browser). Their top is `MANUAL_MAX_LINEAR` / `MANUAL_MAX_ANGULAR` in
`.env` (default 0.6 m/s and 1.0 rad/s, hard ceiling 1.0 m/s and 1.5 rad/s);
the server rejects anything above the configured values. Reverse is capped at
0.20 m/s. Drive by holding a pad button or the **arrow keys** (Up+Left curves);
releasing every key, switching window or hiding the tab stops, and Escape is
STOP / HOLD. Arrow keys are ignored while typing in a text or number field.
**Autonomous speed and obstacle avoidance are configured on the robot**, not
by these manual limits. Verify conservative onboard settings before testing.
Occupied/unknown destination cells are rejected, but this check is not a
footprint/collision planner: the onboard autopilot must find a safe path.
The dashboard also refuses a start less than **0.30 m from an occupied map
cell** and shows **Room to map wall**. This is a preliminary map check, not the
onboard costmap or a guarantee of clearance. If it reports too close, first
check that the green pose and laser match reality. A temporary object recorded
while mapping can leave a **ghost wall** even when the real floor is clear.
The nearby-open-floor hint is relative to the robot's heading and uses only
the stored map. It is **not a checked route**: inspect the real surroundings
before moving manually, or correct the map in Cerebra Studio. Do not change
the initial pose to a false position just to bypass the wall.
If the autopilot reports **Behavior did not complete**, the dashboard ends
the goal immediately, takes zero hold and logs the last robot warning.

### What “stop” does and does not guarantee

- Twist publishing runs at 10 Hz on an independent ROS timer. Nonzero commands
  expire after 0.25 seconds unless renewed by the controller.
- A held manual button must be renewed within 0.35 seconds. Browser release or
  blur sends zero. Missed release requests still expire server-side.
- The controlling browser must heartbeat within 1.5 seconds during manual
  **and autonomous** operation. Closing/hiding the tab requests a stop;
  heartbeat expiry is the fallback.
- Stale odometry, active/stale emergency-brake feedback, stale/invalid/high
  uncertainty localization during navigation, and a 180-second navigation
  deadline latch stop.
- The behavior submission service has **no ROS action cancellation/result**.
  Stop separately calls `/autopilot/behaviortree_executor/stop_execution`
  (`autonomy_msgs/srv/Trigger`) and `/autopilot/behaviortree_executor/abort`
  (`std_srvs/srv/Trigger`), and holds zero USER velocity.
  USER control and `stop_execution` must acknowledge success. Failure of either
  remains **STOP UNCONFIRMED**. Chud advertises `abort` but does not respond;
  it is attempted with a short timeout and logged as an optional legacy
  endpoint, not mistaken for an active physical safety interlock. Brake status
  and stop-service acknowledgment are separate things. Do not bypass failures
  of the required stop or USER control.
- Arrival means a fresh estimated pose within 0.25 m and 0.20 rad of the goal,
  stationary for one second. It is labelled **arrival observed**, not an
  authoritative autopilot success result. Arrival also stops/aborts and holds
  zero rather than leaving a behavior running.

**This is not a certified emergency-stop system.** If the network/bridge or
container dies, it cannot send stop/abort. A service timeout can mean the robot
accepted a command but the response was lost. Onboard velocity timeouts and
the physical e-stop are essential. Verify actual stopping distance, command
timeout behavior, USER priority, scheduler behavior and both stop services
on hardware with the supervisor before relying on navigation.
There is no automatic recovery/resume after a fault.

## Read-only connection checks

Open a shell (this sources the Avular definitions as well as ROS):

```powershell
docker compose exec avular bash -c "source /opt/ros/humble/setup.bash; source /opt/avular_msgs/install/setup.bash; exec bash"
```

Inside that Linux shell:

```bash
ros2 node list
ros2 topic info /autopilot/estimated_pose -v
ros2 topic echo /autopilot/estimated_pose --once
ros2 topic info /robot/safety_supervisor/emergency_brake_active -v
ros2 topic echo /robot/safety_supervisor/emergency_brake_active --once
ros2 topic info /initialpose -v
ros2 service list -t
ros2 service call /autopilot/information_manager/list_maps knowledge_base_msgs/srv/ListMaps '{}'
```

Expected contracts:

| Endpoint | Type |
|---|---|
| `/robot/odom` | `nav_msgs/msg/Odometry` |
| `/autopilot/estimated_pose` | `nav_msgs/msg/Odometry` |
| `/autopilot/lidar/scan_filtered` | `sensor_msgs/msg/LaserScan` |
| `/autopilot/plan_twist` | `autonomy_msgs/msg/Path` |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` |
| `/robot/safety_supervisor/emergency_brake_active` | `std_msgs/msg/Bool` |
| `INITIAL_POSE_TOPIC` | `geometry_msgs/msg/PoseWithCovarianceStamped` |
| `/robot/cmd_vel_user` | `geometry_msgs/msg/Twist` |
| `/robot/cmd_vel_controller/set_control_mode` | `origin_msgs/srv/SetControlMode` |
| `/robot/cmd_vel_controller/reset_control_mode` | `origin_msgs/srv/ReturnControlMode` |
| `/autopilot/information_manager/list_maps` | `knowledge_base_msgs/srv/ListMaps` |
| `/autopilot/information_manager/get_map` | `knowledge_base_msgs/srv/GetMap` |
| `/autopilot/information_manager/resource/map/get_active_id` | `knowledge_base_msgs/srv/GetActiveMapID` |
| `/autopilot/information_manager/load_localization_map` | `knowledge_base_msgs/srv/PublishLocalizationMap` |
| `/autopilot/information_manager/set_and_execute_behaviors` | `knowledge_base_msgs/srv/SetAndExecuteBehaviors` |

The provided discovery output confirms the service names and the estimated
pose type. It **does not confirm** the brake message type, initial-pose message
type, map/pose frames, fresh messages, localization health or stop semantics.
Verify these on the actual robot. If its firmware exposes different contracts,
adapt `app/ros_link.py` against the installed interfaces, not guessed message
fields. Do not change the Origin One's internal computers.

### Interpreting your partner's output

Only seeing `/zenoh_bridge_ros2dds` in `ros2 node list` is consistent with
bridged endpoints: the remote node identities need not appear locally.
`ros2 topic list` also lists subscriptions. Their
`/autopilot/mapping/map` has **zero publishers and one bridge subscription**,
so it is not a usable live map source at that moment.
This dashboard instead requests the selected stored map with `get_map`, then
loads it onboard with `load_localization_map`.
The initial-pose topic must have a route/subscriber of the right type;
the existence of the dashboard's own publisher is insufficient.

`ros2 service list -t` is valid; `-5` is not.
There is no `/navigate_to_pose` action in the supplied output, so this client
does not assume one exists. The listed `ComputePathThroughPoses`/`FollowPath`
actions are lower-level interfaces and are not used as a replacement for the
onboard behavior/control-mode workflow.

## Troubleshooting and shutdown

- **No topics/services:** check `REAL_ROBOT_IP`, TCP 7447 reachability, the two
  containers' matching local domain, the local-vs-robot domain difference and
  bridge logs. Do not enable motion to debug connectivity.
- **Endpoints listed but no data:** both bridge and node must use Humble
  (`ROS_DISTRO=humble` on the bridge). The 1.0.3 image otherwise defaults to
  Iron, whose DDS GID format differs. This image uses the positional `client`
  mode, not `-m client`. Queries allow 30 seconds for large map responses.
- **Feedback goes stale seconds after a restart:** the bridge tracks nodes by
  name, so an expiring older node with the same name removes the live node's
  routes. The dashboard node therefore gets a random suffix per process.
  `docker kill -s USR1 coffe-real-avular-avular-1` dumps all Python thread
  stacks to the logs for diagnosis.
- **No laser/path overlay:** inspect the topics above and TF from `ouster_base`
  to `map`. Scan points are downsampled and transformed using onboard TF at
  their timestamp; no pose-derived fallback is invented. Scan data older than
  2 seconds and plans older than 5 seconds are hidden. A stationary robot may
  have no plan. Preview maps deliberately show no live overlays.
- **No brake data / initial pose rejected:** check the types above. The stack
  fails closed if required feedback/routes are absent.
- **Stop service returns false while idle:** record the response and confirm the
  firmware's semantics with Avular. This version deliberately treats false as
  unconfirmed, even if the message seems benign.
- **Frame mismatch:** inspect the stored occupancy-grid frame and
  `estimated_pose.header.frame_id`. They must match `NAV_FRAME`. Change `.env`
  only after verifying the frame. No odom-to-map fallback or invented TF is used.
- **Pose does not change after seeding:** the bridge route is not an AMCL
  acknowledgment. Check onboard localization/lifecycle state and its correct
  initial-pose endpoint. Do not click confirm as a workaround.
- **Request accepted but no motion:** inspect onboard mode, active map,
  localization and safety state. Do not send repeated goals or bypass stops.
  **Starting point in lethal space** means the onboard planner considers the
  starting footprint blocked. Check the map-wall clearance and localization;
  a ghost wall or a real obstacle can cause this. Clearing a costmap does not
  remove a wall stored in the map.
- **Dashboard stopped after tab change:** expected heartbeat/visibility safety
  behavior. Stop, inspect the robot and re-enable explicitly.

Before shutting down, click **STOP / HOLD**, verify the physical robot is
stationary, release USER only after a confirmed stop, and keep the physical
stop available. Then:

```powershell
docker compose down
```

Graceful shutdown attempts stop if this process controlled the base. Forced
termination, laptop sleep, power loss and broken networks cannot guarantee
delivery. No automatic container restart is configured.

## Validation

Host-only tests need Python 3 and the standard library, not ROS or Docker:

```powershell
py -m unittest discover -s tests -v
docker compose config --quiet
```

For the presentation checks, open the disabled dashboard in the browser
session, then run (no robot commands are sent):

```powershell
Get-Content -Raw tests/display_browser.js | agent-browser --cdp "$env:AGENT_BROWSER_CDP" eval --stdin
```

This checks themes, panel controls and responsive layouts at 420–3840 px.

Tests cover read-only startup, manual expiry and limits, heartbeat/feedback
loss, ownership, rotated map coordinates, localization gates, goal submission
failure, arrival checks, stop/release behavior and HTTP command protections.
They use fake hardware only and never command the robot.

The host tests use fake hardware; they do not validate stopping distance,
localization accuracy or actual navigation. Docker build/runtime and live
read-only feedback are checked separately. No real-motion validation has
been performed by the agent.

The dashboard is published on loopback only. A per-process page token guards
command POSTs; this is a local operator tool, **not** an internet-facing
authenticated service. Do not expose its port through public interfaces.

Files: `docker-compose.yml` (two services), `Dockerfile` (ROS + messages),
`app/control.py` (safety/state), `app/ros_link.py` (hardware interfaces),
`app/server.py` (stdlib HTTP API), `app/index.html` (dashboard),
`app/sim_map.py` (read-only Cerebra preview),
`tests/test_stack.py` (hardware-free controller/HTTP tests) and
`tests/test_ros_link.py` (adapter methods with ROS import stand-ins).
