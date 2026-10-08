"""Unit tests for tools/cerebra_import.py (standard library only).

Run from the Simulation-Docker folder:
  py -m unittest tests/test_cerebra_import.py
"""

import base64
import json
import os
import subprocess
import sys
import tempfile
import unittest

TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools")
sys.path.insert(0, TOOLS)
import cerebra_import  # noqa: E402

MAP_ID = "{11111111-2222-3333-4444-555555555555}"
W, H, RES = 40, 20, 0.5  # 20 m x 10 m map, origin (-10, -5)


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def write(folder, kind, name, data):
    os.makedirs(os.path.join(folder, kind), exist_ok=True)
    with open(os.path.join(folder, kind, name + ".json"), "w", encoding="utf-8") as f:
        json.dump(data, f)


def make_data(folder):
    grid = bytearray(W * H)
    for c in range(W):  # walls along the bottom and top rows
        grid[c] = 100
        grid[(H - 1) * W + c] = 100
    write(folder, "maps", MAP_ID, {
        "id": MAP_ID, "name": "Test Lab",
        "global_reference_frame": {"type": 1, "local_tangent_plane": {"latitude": 51.0, "longitude": 5.0, "altitude": 10.0}},
        "global_transformation": {"orientation": {"w": 1, "x": 0, "y": 0, "z": 0}, "position": {"x": 0, "y": 0, "z": 0}},
        "origin": {"x": -10.0, "y": -5.0, "z": 0},
        "size": {"width_in_pixels": W, "height_in_pixels": H, "width_in_meters": W * RES, "height_in_meters": H * RES},
        "occupancy_grid": base64.b64encode(bytes(grid)).decode(),
    })
    local = {"type": 0, "local_relative": MAP_ID}
    write(folder, "waypoints", "wp1", {"id": "wp1", "name": "Home", "coordinate_system_info": local,
                                        "pose": {"position": {"x": 1.0, "y": 2.0, "z": 0}, "orientation": {"w": 0.7071068, "x": 0, "y": 0, "z": 0.7071068}}})
    write(folder, "waypoints", "wp2", {"id": "wp2", "name": "Elsewhere",
                                        "coordinate_system_info": {"type": 0, "local_relative": "{other-map}"},
                                        "pose": {"position": {"x": 0, "y": 0, "z": 0}, "orientation": {"w": 1, "x": 0, "y": 0, "z": 0}}})
    # Path in the layout of Avular's example_origin_ros (global frame, list of poses)
    write(folder, "paths", "p1", {"type": "path", "resource": {
        "name": "Corridor", "coordinate_system_info": {"type": 1, "local_tangent_plane": {"latitude": 51.0, "longitude": 5.0, "altitude": 10.0}},
        "poses": [{"position": {"x": i * 0.1, "y": 0.0, "z": 0.0}, "orientation": {"w": 1, "x": 0, "y": 0, "z": 0}} for i in range(30)]}})
    # Zone in the layout of knowledge_base_msgs/TypedPolygon
    write(folder, "typedpolygons", "z1", {"name": "Lab door", "type": "no_go_area", "coordinate_system_info": local,
                                           "polygon": {"points": [{"x": 4, "y": -1, "z": 0}, {"x": 6, "y": -1, "z": 0},
                                                                  {"x": 6, "y": 1, "z": 0}, {"x": 4, "y": 1, "z": 0}]}})
    write(folder, "typedpolygons", "z2", {"name": "Work area", "type": 2, "coordinate_system_info": local,
                                           "points": [[-8, -3], [-6, -3], [-6, -1]]})


class CerebraImportTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.data = os.path.join(self.tmp.name, "cerebra-data")
        self.out = os.path.join(self.tmp.name, "out")
        make_data(self.data)

    def tearDown(self):
        self.tmp.cleanup()

    def run_import(self, *extra):
        return subprocess.run([sys.executable, os.path.join(TOOLS, "cerebra_import.py"), "--data", self.data,
                               "--out", self.out, *extra], capture_output=True, text=True, check=True).stdout

    def test_scene_contents(self):
        m = cerebra_import.select_map(cerebra_import.read_dir(self.data, "maps"), "test lab")
        waypoints, paths, zones = cerebra_import.load_scene(self.data, m)
        self.assertEqual([w["name"] for w in waypoints], ["Home"])
        self.assertAlmostEqual(waypoints[0]["yaw"], 1.5708, places=3)
        self.assertEqual(len(paths), 1)
        self.assertEqual(len(paths[0]["points"]), 30)
        self.assertEqual({z["name"]: z["type"] for z in zones}, {"Lab door": "no_go_area", "Work area": "cover_area"})

    def test_map_selection_by_id_and_slug(self):
        maps = cerebra_import.read_dir(self.data, "maps")
        self.assertEqual(cerebra_import.select_map(maps, MAP_ID.strip("{}"))["name"], "Test Lab")
        self.assertEqual(cerebra_import.select_map(maps, "test_lab")["name"], "Test Lab")
        with self.assertRaises(SystemExit):
            cerebra_import.select_map(maps, "Unknown")

    def test_nav_outputs_and_keepout_mask(self):
        self.run_import("--mode", "nav")
        env = read(os.path.join(self.out, "selected.env"))
        self.assertIn("ROBOT_POSE_X='1.000'", env)
        self.assertIn("KEEPOUT_MASK=", env)
        with open(os.path.join(self.out, "keepout_mask.pgm"), "rb") as f:
            pixels = f.read()[-W * H:]
        # PGM rows are flipped (top row = max y); cell at (5, 0) is inside the no-go zone
        col, row = int((5 - -10) / RES), int((0 - -5) / RES)
        self.assertEqual(pixels[(H - 1 - row) * W + col], 0)
        self.assertEqual(pixels[(H - 1 - row) * W + 2], 254)

    def test_sim_outputs(self):
        self.run_import("--mode", "sim", "--spawn", "missing")
        env = read(os.path.join(self.out, "selected.env"))
        self.assertIn("SIM_WORLD='cerebra/test_lab.world'", env)
        self.assertIn("ROBOT_POSE_Y='2.000'", env)  # falls back to the first waypoint
        world = read(os.path.join(self.out, "test_lab.world"))
        self.assertIn("waypoint_Home", world)


if __name__ == "__main__":
    unittest.main()
