# Map previews (optional)

Put Cerebra Studio map exports (`*.json`) here to use **Load simulation map**.
The dashboard finds the map named by `SIM_MAP` in `.env`, by name or ID.

These files are only drawn as a read-only preview. The robot can only
localize and navigate on maps stored on the robot itself; import a map with
Avular's Cerebra Studio first. To use maps from another folder, set
`SIM_MAPS_DIR` in `.env` (for example `SIM_MAPS_DIR=../cerebra-data/maps`).
