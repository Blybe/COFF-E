# Development and tests

The dashboard server uses only the Python standard library plus ROS 2 inside
the container. Dashboard code in `app/` is copied into the image, so rebuild
after every change: `docker compose up -d --build`.

## Tests

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

## Files

| File | Role |
|---|---|
| `docker-compose.yml` | the two services |
| `Dockerfile` | ROS 2 Humble image with the pinned Avular messages |
| `app/server.py` | stdlib HTTP API and page token |
| `app/control.py` | safety state machine and watchdog |
| `app/ros_link.py` | ROS 2 interfaces to the robot |
| `app/connection.py` | robot detection from the laptop Wi-Fi snapshot |
| `app/places.py` | saved places per map |
| `app/sim_map.py` | read-only Cerebra map preview |
| `app/index.html` | the whole dashboard (HTML, CSS, JS) |
| `Watch-Wifi.ps1` | read-only Windows Wi-Fi monitor |
| `tests/` | hardware-free tests and browser checks |

## Making a release

1. Change the version in `app/index.html` (`<span … id="version">`).
2. Add an entry to `CHANGELOG.md`.
3. Run the tests above, then `docker compose up -d --build` and check the
   version in the dashboard header.
