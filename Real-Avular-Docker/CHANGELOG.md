# Changelog

The version is shown in the top-right corner of the dashboard. Bump it in
`app/index.html` (`id="version"`) and add an entry here for every release.

## 1.2 · 2026-11-11

- Version number in the dashboard header.
- Error messages are written for non-experts; click one to see the technical
  details.
- Activity log: stays where you scrolled while new lines arrive (**Jump to
  latest** returns), can be resized, keeps the last 500 lines, and has a
  **Copy log** button for sharing.
- Robot detection works when the Docker clock drifts from the Windows clock
  (common after laptop sleep), which made the Wi-Fi monitor look stale.
- Pink laser dots come back by themselves after switching between Chud and
  Ronaldo or rebooting the robot, instead of needing a container restart.
- Self-contained folder for publishing: MIT license, `docs/`, `maps/` and
  `SIM_MAPS_DIR`.

## Earlier

- Automatic Chud/Ronaldo detection from laptop Wi-Fi, robot themes, saved
  places, four-step wizard, laser overlay and auto-fit, manual drive and
  autopilot navigation.
