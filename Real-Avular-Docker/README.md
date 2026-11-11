# COFF-E · Real Avular dashboard

A browser dashboard for driving and navigating a **physical Avular Origin One**
over ROS 2. It runs in two Docker containers on your laptop and talks to the
robot's own onboard autopilot. You can see the robot on its map, with live
laser dots, localize it, drive it by hand and send it to saved places.

Built for the Origin One robots **Chud** (`Origin PB4U9RK5`) and **Ronaldo**
(`Origin PB4U9CR7`). The dashboard detects which one your laptop is connected
to and picks that robot's map and colours automatically.

> **Safety first.** This is a student tool, not a certified safety system.
> Always keep the physical emergency stop within reach, test in a clear area,
> and stay with the robot. See [What “stop” does and does not guarantee](docs/operator-guide.md#what-stop-does-and-does-not-guarantee).

## What runs where

```text
Browser at localhost:8082
          |
avular container: dashboard + ROS 2 Humble node + safety watchdog
          | local DDS domain 42, Cyclone DDS
zenoh-bridge container: zenoh-bridge-ros2dds 1.0.3
          | TCP 7447
Physical Avular: sensors + map storage + localization + onboard ROS 2 autopilot
```

The robot must be switched on with its autopilot running. These containers
are a client of that autopilot, not a replacement for it.

## Requirements

- Windows 10/11 with **Docker Desktop** (Linux containers). Linux and macOS
  work for the containers, but the Wi-Fi robot detection script is Windows-only.
- Internet access for the first build (ROS packages and Avular message
  definitions are downloaded).
- A laptop Wi-Fi connection to the robot's hotspot.

## Quick start

```powershell
cd Real-Avular-Docker
Copy-Item .env.example .env
notepad .env        # set REAL_ROBOT_IP (Chud and Ronaldo: 192.168.192.1)
docker compose up -d --build
```

In a second PowerShell window, start the read-only Wi-Fi monitor and leave it
running. It lets the dashboard see which robot you are connected to:

```powershell
cd Real-Avular-Docker
.\Watch-Wifi.ps1
```

Open **http://localhost:8082**. Starting the stack does **not** enable motion,
take control, change maps, set a pose or send a goal.

Then follow the four steps on the left of the dashboard:
**Map → Localize → Enable → Drive**. The [operator guide](docs/operator-guide.md)
explains each step.

Check the version in the top-right corner of the dashboard (for example
**v1.2**). After pulling a new version, rebuild with
`docker compose up -d --build`, otherwise you keep running the old one.

## Documentation

| Document | For |
|---|---|
| [docs/operator-guide.md](docs/operator-guide.md) | Driving, localizing, saved places, Chud vs Ronaldo, what STOP guarantees |
| [docs/troubleshooting.md](docs/troubleshooting.md) | Common problems, read-only ROS checks, shutdown |
| [docs/development.md](docs/development.md) | Running the tests, project files, making a release |
| [CHANGELOG.md](CHANGELOG.md) | What changed in each version |

When something goes wrong, the dashboard shows a short explanation. Click it
for the technical details. **Copy log** under *Activity log* copies the log
(with the dashboard version and robot name) so you can share it.

## Project layout

```text
app/            dashboard server (Python, no extra packages) and index.html
docs/           operator guide, troubleshooting, development notes
maps/           optional Cerebra map exports for the read-only map preview
places/         saved places per onboard map (kept across rebuilds)
tests/          hardware-free tests; they never command the robot
Watch-Wifi.ps1  read-only Windows Wi-Fi monitor for robot detection
```

## License

[MIT](LICENSE) © 2026 Robin van den Dungen & Ian Kolsteeg.
