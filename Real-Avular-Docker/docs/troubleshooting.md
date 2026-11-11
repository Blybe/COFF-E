# Troubleshooting

When the dashboard shows an error, click it for the technical details, and use
**Copy log** under *Activity log* to share what happened.

## Robot not detected / “Laptop Wi-Fi monitor stale”

- Keep `.\Watch-Wifi.ps1` running in its own PowerShell window, started from
  this folder (the one with `docker-compose.yml`). A copy in another folder
  writes to a `status/` folder Docker does not see.
- It must say nothing and keep running. “A COFF-E Wi-Fi monitor is already
  running” means another window already runs it.
- The parser expects English Windows field names (`netsh wlan show interfaces`).
- Connect only one adapter to a robot hotspot at a time.
- Since v1.2 a Docker clock that differs from Windows no longer causes
  “stale”. If it still says stale, compare
  `[DateTimeOffset]::UtcNow.ToUnixTimeSeconds()` with
  `docker compose exec -T avular date +%s`, and check that
  `status\wifi.json` updates every 3 seconds.

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
  After a hotspot switch or robot reboot the robot clock restarts; the
  dashboard notices odometry jumping back and clears its TF buffer (logs
  `TF_OLD_DATA` until then). If the pink dots still do not return, run
  `docker compose restart avular`.
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

### Interpreting `ros2` output

Only seeing `/zenoh_bridge_ros2dds` in `ros2 node list` is consistent with
bridged endpoints: the remote node identities need not appear locally.
`ros2 topic list` also lists subscriptions. If
`/autopilot/mapping/map` has **zero publishers and one bridge subscription**,
so it is not a usable live map source at that moment.
This dashboard instead requests the selected stored map with `get_map`, then
loads it onboard with `load_localization_map`.
The initial-pose topic must have a route/subscriber of the right type;
the existence of the dashboard's own publisher is insufficient.

`ros2 service list -t` is valid; `-5` is not.
There is no `/navigate_to_pose` action on these robots, so this client
does not assume one exists. The listed `ComputePathThroughPoses`/`FollowPath`
actions are lower-level interfaces and are not used as a replacement for the
onboard behavior/control-mode workflow.
