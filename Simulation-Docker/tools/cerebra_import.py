"""Import Cerebra Studio data (cerebra-data/) into the Origin One simulation.

cerebra-data/ mirrors Cerebra Studio's Mission Planner folder (see
tools/sync_cerebra.ps1): maps/, waypoints/, paths/, typedpolygons/, ...
This script selects one map and converts it plus everything that belongs to
it into the files the containers need:

  --mode sim   <slug>.world/.obj/.mtl  Gazebo world (walls + waypoint markers)
  --mode nav   <slug>.pgm/.yaml        Nav2 map
               keepout_mask.pgm/.yaml  Nav2 keep-out mask (no_go_area zones)
               scene.json              waypoints, paths and zones (map frame)
  both modes   selected.env            settings read by the container command

Only the Python standard library is needed, so it also runs on the host:
  py tools/cerebra_import.py --data ../cerebra-data --list
"""

import argparse
import json
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cerebra_map_to_world import (  # noqa: E402
    free_near, load_map, merge_walls, world_sdf, write_obj, write_pgm, write_yaml)

# Order of the TypedPolygon type constants, used if a file stores the type as a number.
POLYGON_TYPES = ["go_area", "no_go_area", "cover_area", "map_layout"]


def slug(text):
    return re.sub(r"[^a-z0-9]+", "_", text.lower()).strip("_") or "map"


def norm_id(value):
    return str(value or "").strip("{}").lower()


def read_dir(data_dir, kind):
    items = []
    folder = os.path.join(data_dir, kind)
    if not os.path.isdir(folder):
        return items
    for name in sorted(os.listdir(folder)):
        if not name.endswith(".json"):
            continue
        path = os.path.join(folder, name)
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            print(f"WARNING: skipping {path}: {e}")
            continue
        # Knowledge-base exports wrap the item as {"type": ..., "resource": {...}}
        if isinstance(data, dict) and isinstance(data.get("resource"), dict):
            data = data["resource"]
        data["_file"] = path
        items.append(data)
    return items


def select_map(maps, query):
    if not maps:
        sys.exit("No maps found in cerebra-data/maps. Run tools/sync_cerebra.ps1 on a laptop with Cerebra Studio.")
    if not query:
        if len(maps) == 1:
            return maps[0]
        sys.exit("Several maps found; set CEREBRA_MAP to one of: " + ", ".join(f'"{m["name"]}"' for m in maps))
    q = query.strip()
    for test in (lambda m: norm_id(m.get("id")) == norm_id(q),
                 lambda m: m.get("name") == q,
                 lambda m: m.get("name", "").lower() == q.lower(),
                 lambda m: slug(m.get("name", "")) == slug(q)):
        hits = [m for m in maps if test(m)]
        if hits:
            return hits[0]
    sys.exit(f'Map "{query}" not found. Available: ' + ", ".join(f'"{m["name"]}"' for m in maps))


def yaw_of(q):
    return math.atan2(2 * (q.get("w", 1) * q.get("z", 0) + q.get("x", 0) * q.get("y", 0)),
                      1 - 2 * (q.get("y", 0) ** 2 + q.get("z", 0) ** 2))


class Frame:
    """Converts item coordinates into the selected map's frame."""

    def __init__(self, m):
        self.map_id = norm_id(m["id"])
        gt = m.get("global_transformation") or {}
        p = gt.get("position") or {}
        self.tx, self.ty = p.get("x", 0.0), p.get("y", 0.0)
        self.tyaw = yaw_of(gt.get("orientation") or {})

    def belongs(self, item):
        """True if the item is defined relative to this map or globally."""
        info = item.get("coordinate_system_info") or item.get("coord_info") or {}
        rel = info.get("local_relative")
        return not rel or norm_id(rel) == self.map_id

    def is_global(self, item):
        info = item.get("coordinate_system_info") or item.get("coord_info") or {}
        return not info.get("local_relative")

    def to_map(self, item, x, y, yaw=0.0):
        if not self.is_global(item):
            return x, y, yaw
        # global_transformation is the map's pose in the global frame
        dx, dy = x - self.tx, y - self.ty
        c, s = math.cos(-self.tyaw), math.sin(-self.tyaw)
        return c * dx - s * dy, s * dx + c * dy, yaw - self.tyaw


def xy(p):
    if isinstance(p, dict):
        if "position" in p:
            return xy(p["position"])
        if "pose" in p:
            return xy(p["pose"])
        if "x" in p and "y" in p:
            return float(p["x"]), float(p["y"])
    if isinstance(p, (list, tuple)) and len(p) >= 2 and all(isinstance(v, (int, float)) for v in p[:2]):
        return float(p[0]), float(p[1])
    return None


def find_points(obj, keys):
    """Returns the first list of points found under one of `keys` (searched recursively)."""
    if isinstance(obj, dict):
        for k in keys:
            if k in obj:
                v = obj[k]
                if isinstance(v, list) and v and all(xy(p) for p in v):
                    return [xy(p) for p in v]
                found = find_points(v, keys)
                if found:
                    return found
        for v in obj.values():
            if isinstance(v, (dict, list)):
                found = find_points(v, keys)
                if found:
                    return found
    elif isinstance(obj, list):
        for v in obj:
            found = find_points(v, keys)
            if found:
                return found
    return None


def load_scene(data_dir, m):
    frame = Frame(m)
    waypoints, paths, zones = [], [], []

    for wp in read_dir(data_dir, "waypoints"):
        if not frame.belongs(wp) or "pose" not in wp:
            continue
        p = wp["pose"].get("position", {})
        x, y, yaw = frame.to_map(wp, p.get("x", 0.0), p.get("y", 0.0), yaw_of(wp["pose"].get("orientation", {})))
        waypoints.append({"name": wp.get("name", "?"), "x": x, "y": y, "yaw": yaw})

    for path in read_dir(data_dir, "paths"):
        if not frame.belongs(path):
            continue
        pts = find_points(path, ["poses", "points", "path", "waypoints"])
        if not pts or len(pts) < 2:
            print(f"WARNING: path '{path.get('name')}' has no readable poses ({path['_file']})")
            continue
        pts = [frame.to_map(path, x, y)[:2] for x, y in pts]
        paths.append({"name": path.get("name", "?"), "points": pts})

    for poly in read_dir(data_dir, "typedpolygons"):
        if not frame.belongs(poly):
            continue
        ptype = poly.get("polygon_type", poly.get("type", ""))
        if isinstance(ptype, int):
            ptype = POLYGON_TYPES[ptype] if 0 <= ptype < len(POLYGON_TYPES) else str(ptype)
        ptype = str(ptype).lower()
        if ptype == "map_layout" and norm_id(poly.get("map_id")) not in ("", frame.map_id):
            continue
        pts = find_points(poly, ["polygon", "points", "vertices", "shape"])
        if not pts or len(pts) < 3:
            print(f"WARNING: zone '{poly.get('name')}' has no readable polygon ({poly['_file']})")
            continue
        pts = [frame.to_map(poly, x, y)[:2] for x, y in pts]
        zones.append({"name": poly.get("name", "?"), "type": ptype, "points": pts})

    return waypoints, paths, zones


def inside(px, py, pts):
    hit = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > py) != (yj > py) and px < (xj - xi) * (py - yi) / (yj - yi) + xi:
            hit = not hit
        j = i
    return hit


def keepout_grid(zones, w, h, res, origin):
    """Occupancy-style grid: 100 inside no_go_area zones, 0 elsewhere."""
    grid = bytearray(w * h)
    for z in zones:
        if z["type"] != "no_go_area":
            continue
        xs = [p[0] for p in z["points"]]
        ys = [p[1] for p in z["points"]]
        c0 = max(0, int((min(xs) - origin["x"]) / res))
        c1 = min(w - 1, int((max(xs) - origin["x"]) / res))
        r0 = max(0, int((min(ys) - origin["y"]) / res))
        r1 = min(h - 1, int((max(ys) - origin["y"]) / res))
        for r in range(r0, r1 + 1):
            cy = origin["y"] + (r + 0.5) * res
            for c in range(c0, c1 + 1):
                if inside(origin["x"] + (c + 0.5) * res, cy, z["points"]):
                    grid[r * w + c] = 100
    return grid


def print_listing(data_dir, maps):
    for m in maps:
        wps, paths, zones = load_scene(data_dir, m)
        print(f'Map "{m["name"]}"  ({norm_id(m["id"])})')
        print("  waypoints: " + (", ".join(w["name"] for w in wps) or "-"))
        print("  paths:     " + (", ".join(p["name"] for p in paths) or "-"))
        print("  zones:     " + (", ".join(f'{z["name"]} [{z["type"]}]' for z in zones) or "-"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default=os.path.join(os.path.dirname(__file__), "..", "..", "cerebra-data"))
    ap.add_argument("--map", default="", help="map name or id (may be empty if there is only one map)")
    ap.add_argument("--spawn", default="Home", help="waypoint name to spawn the robot at")
    ap.add_argument("--mode", choices=["sim", "nav", "all"], default="all")
    ap.add_argument("--out", help="output folder")
    ap.add_argument("--sim-world-prefix", default="cerebra",
                    help="SIM_WORLD is written as <prefix>/<slug>.world (relative to the worlds folder)")
    ap.add_argument("--wall-height", type=float, default=2.0)
    ap.add_argument("--occupied-threshold", type=int, default=65)
    ap.add_argument("--list", action="store_true", help="only list maps and their items")
    args = ap.parse_args()

    maps = read_dir(args.data, "maps")
    if args.list:
        print_listing(args.data, maps)
        return
    if not args.out:
        sys.exit("--out is required")

    m = select_map(maps, args.map)
    name = slug(m["name"])
    _, grid, w, h, res = load_map(m["_file"])
    waypoints, paths, zones = load_scene(args.data, m)

    spawn = next((wp for wp in waypoints if wp["name"] == args.spawn), None)
    if spawn is None and args.spawn:
        spawn = next((wp for wp in waypoints if wp["name"].lower() == args.spawn.lower()), None)
    if spawn is None:
        spawn = waypoints[0] if waypoints else {"name": "map origin", "x": 0.0, "y": 0.0, "yaw": 0.0}
        print(f"Waypoint '{args.spawn}' not found on this map; spawning at '{spawn['name']}'.")
    if not free_near(grid, w, h, res, m["origin"], spawn["x"], spawn["y"]):
        print(f"WARNING: spawn point ({spawn['x']:.2f}, {spawn['y']:.2f}) is within 0.4 m of a wall.")

    os.makedirs(args.out, exist_ok=True)
    out = lambda fname: os.path.join(args.out, fname)
    env = {
        "CEREBRA_MAP_NAME": m["name"],
        "ROBOT_POSE_X": f"{spawn['x']:.3f}",
        "ROBOT_POSE_Y": f"{spawn['y']:.3f}",
        "ROBOT_POSE_YAW": f"{spawn['yaw']:.4f}",
    }

    if args.mode in ("sim", "all"):
        rects = merge_walls(grid, w, h, args.occupied_threshold)
        write_obj(out(name + ".obj"), name + ".mtl", rects, res, m["origin"], args.wall_height)
        mesh_uri = "file://" + os.path.abspath(out(name + ".obj")).replace("\\", "/")
        with open(out(name + ".world"), "w", encoding="utf-8", newline="\n") as f:
            f.write(world_sdf(m, len(rects), res, waypoints, spawn, args.wall_height, mesh_uri))
        env["SIM_WORLD"] = f"{args.sim_world_prefix}/{name}.world"

    if args.mode in ("nav", "all"):
        write_pgm(out(name + ".pgm"), grid, w, h)
        write_yaml(out(name + ".yaml"), name + ".pgm", res, m["origin"])
        env["NAV_MAP"] = os.path.abspath(out(name + ".yaml"))
        if any(z["type"] == "no_go_area" for z in zones):
            write_pgm(out("keepout_mask.pgm"), keepout_grid(zones, w, h, res, m["origin"]), w, h)
            write_yaml(out("keepout_mask.yaml"), "keepout_mask.pgm", res, m["origin"])
            env["KEEPOUT_MASK"] = os.path.abspath(out("keepout_mask.yaml"))
        with open(out("scene.json"), "w", encoding="utf-8") as f:
            json.dump({"map": m["name"], "spawn": spawn["name"], "waypoints": waypoints,
                       "paths": paths, "zones": zones}, f, indent=1)

    with open(out("selected.env"), "w", encoding="utf-8", newline="\n") as f:
        # Shell-quoted, the container command sources this file
        f.writelines("{}='{}'\n".format(k, str(v).replace("'", "'\\''")) for k, v in env.items())

    print(f'[cerebra] map "{m["name"]}": {w}x{h} cells at {res:.3f} m')
    print(f"[cerebra] waypoints: {', '.join(wp['name'] for wp in waypoints) or '-'}")
    print(f"[cerebra] paths: {', '.join(p['name'] for p in paths) or '-'}")
    print(f"[cerebra] zones: {', '.join(z['name'] + ' [' + z['type'] + ']' for z in zones) or '-'}")
    print(f"[cerebra] spawn at '{spawn['name']}': x={spawn['x']:.3f} y={spawn['y']:.3f} yaw={spawn['yaw']:.4f}")


if __name__ == "__main__":
    main()
