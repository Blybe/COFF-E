"""Drive the simulated Origin One with Nav2, using the Cerebra data of the loaded map.

Usage (from the host):
  docker exec -it origin-nav goto list               show waypoints, paths and zones
  docker exec -it origin-nav goto Home               drive to a Cerebra waypoint
  docker exec -it origin-nav goto path "Corridor"    follow a Cerebra path
  docker exec -it origin-nav goto 3.5 -1.2 [yaw]     drive to map coordinates (yaw in rad)

Exit code 0 = goal reached, 1 = failed. Driving to a waypoint plans a route
(like Cerebra's MOVE_THROUGH_POSES); following a path drives along the
recorded poses (like MOVE_ALONG_PATH).
"""

import json
import math
import os
import sys
import time

SCENE_FILE = os.environ.get("CEREBRA_SCENE", "/tmp/cerebra/scene.json")


def load_scene():
    try:
        with open(SCENE_FILE, encoding="utf-8") as f:
            return json.load(f)
    except OSError:
        return {"map": "(no Cerebra map loaded)", "waypoints": [], "paths": [], "zones": []}


def find(items, name, kind):
    for test in (lambda i: i["name"] == name, lambda i: i["name"].lower() == name.lower()):
        hits = [i for i in items if test(i)]
        if hits:
            return hits[0]
    names = ", ".join(f'"{i["name"]}"' for i in items) or "none"
    sys.exit(f'{kind} "{name}" not found. Available: {names}')


def pose(nav, x, y, yaw):
    from geometry_msgs.msg import PoseStamped
    p = PoseStamped()
    p.header.frame_id = "map"
    # Keep the stamp at zero: tf2 then uses the latest available transform.
    # AMCL only publishes map->odom while the robot is moving, so a pose
    # stamped with the current time can be rejected as "Transform data too
    # old" when a goal is sent from standstill.
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def wait(nav, what, timeout=600):
    from nav2_simple_commander.robot_navigator import TaskResult
    start, last = time.time(), -1
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        elapsed = int(time.time() - start)
        if fb is not None and elapsed // 5 != last:
            last = elapsed // 5
            dist = getattr(fb, "distance_remaining", getattr(fb, "distance_to_goal", None))
            speed = getattr(fb, "speed", None)
            extra = f", {speed:.2f} m/s" if speed is not None else ""
            if dist is not None:
                print(f"  {what}: {dist:.1f} m to go{extra}")
        if time.time() - start > timeout:
            nav.cancelTask()
            print(f"FAIL: {what} timed out after {timeout} s")
            return False
        time.sleep(0.5)
    result = nav.getResult()
    print(f"{what}: {result.name} after {time.time() - start:.0f} s")
    return result == TaskResult.SUCCEEDED


def main():
    args = sys.argv[1:]
    scene = load_scene()
    if not args or args[0] in ("-h", "--help"):
        print(__doc__)
        return 0
    if args[0] == "list":
        print(f'Map: {scene["map"]}')
        for wp in scene["waypoints"]:
            print(f'  waypoint  {wp["name"]:<24} x={wp["x"]:.2f} y={wp["y"]:.2f} yaw={wp["yaw"]:.2f}')
        for p in scene["paths"]:
            print(f'  path      {p["name"]:<24} {len(p["points"])} poses')
        for z in scene["zones"]:
            print(f'  zone      {z["name"]:<24} {z["type"]}')
        return 0

    import rclpy
    from nav2_simple_commander.robot_navigator import BasicNavigator
    from nav_msgs.msg import Path

    rclpy.init()
    nav = BasicNavigator()
    # Not using waitUntilNav2Active(): it publishes an initial pose of (0, 0),
    # which would overwrite the AMCL pose set from the spawn waypoint.
    nav._waitForNodeToActivate("bt_navigator")

    if args[0] == "path":
        if len(args) < 2:
            sys.exit('usage: goto path "<name>"')
        p = find(scene["paths"], " ".join(args[1:]), "Path")
        pts = p["points"]
        poses = []
        for i, (x, y) in enumerate(pts):
            nx, ny = pts[min(i + 1, len(pts) - 1)]
            px, py = pts[max(i - 1, 0)]
            poses.append(pose(nav, x, y, math.atan2(ny - py, nx - px)))
        # Drive to the start first; the path controller expects to begin on the path.
        print(f'Driving to the start of path "{p["name"]}"')
        nav.goToPose(poses[0])
        if not wait(nav, "to path start"):
            return 1
        path = Path()
        path.header.frame_id = "map"
        path.poses = poses
        print(f'Following path "{p["name"]}" ({len(poses)} poses)')
        nav.followPath(path)
        ok = wait(nav, "path")
    else:
        try:
            x, y = float(args[0]), float(args[1])
            yaw = float(args[2]) if len(args) > 2 else 0.0
            name = f"({x:.2f}, {y:.2f})"
        except (ValueError, IndexError):
            wp = find(scene["waypoints"], " ".join(args), "Waypoint")
            x, y, yaw, name = wp["x"], wp["y"], wp["yaw"], f'waypoint "{wp["name"]}"'
        goal = pose(nav, x, y, yaw)
        route = nav.getPath(pose(nav, 0, 0, 0), goal, use_start=False)
        if route is None or not route.poses:
            print(f"FAIL: no route to {name}")
            return 1
        length = sum(math.dist((a.pose.position.x, a.pose.position.y), (b.pose.position.x, b.pose.position.y))
                     for a, b in zip(route.poses, route.poses[1:]))
        print(f"Driving to {name}, planned route {length:.1f} m")
        nav.goToPose(goal)
        ok = wait(nav, name)

    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
