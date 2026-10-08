# Cerebra Studio data

Shared copy of Avular Cerebra Studio's Mission Planner data, so every team
member (with or without Cerebra Studio) has the same maps, waypoints, paths
and zones. The layout is the same as on disk in Cerebra Studio
(`%APPDATA%\Avular\Cerebra Studio\Mission Planner\`), one JSON file per item:

| Folder           | Content                                              |
|------------------|------------------------------------------------------|
| `maps/`          | occupancy-grid maps                                  |
| `waypoints/`     | named poses, linked to a map                         |
| `paths/`         | recorded routes (list of poses)                      |
| `typedpolygons/` | zones: `go_area`, `no_go_area`, `cover_area`, `map_layout` |
| `locations/`     | groups of maps                                       |
| `jobs/`          | missions (not used by the simulation yet)            |

## Updating

After changing something in Cerebra Studio (from the repository root):

```powershell
powershell -ExecutionPolicy Bypass -File Simulation-Docker\tools\sync_cerebra.ps1
git add cerebra-data
git commit -m "Update Cerebra data"
git push
```

To load the shared data into your own Cerebra Studio (close it first):

```powershell
git pull
powershell -ExecutionPolicy Bypass -File Simulation-Docker\tools\sync_cerebra.ps1 -Direction Import
```

The simulation (`Simulation-Docker/`) reads this folder directly; select the
map with `CEREBRA_MAP` in `Simulation-Docker/.env`.
