"""Reads the simulation stack's Cerebra map so the real dashboard can show it.

The robot can only localize on maps stored in its own knowledge base, so this
file is a display source; the controller decides whether the robot has it.
"""

import base64
import json
import os

MAX_CELLS = 4_000_000


def norm_id(value):
    return str(value or "").strip("{}").lower()


class Source:
    def __init__(self, folder, query):
        self.folder, self.query = folder, (query or "").strip()
        self._data = None

    def _find(self):
        if self._data is not None:
            return self._data
        if not self.query:
            raise ValueError("SIM_MAP is not set")
        if not self.folder or not os.path.isdir(self.folder):
            raise ValueError("Simulation map folder is not mounted: " + str(self.folder))
        q = self.query.lower()
        for name in sorted(os.listdir(self.folder)):
            if not name.endswith(".json"):
                continue
            with open(os.path.join(self.folder, name), encoding="utf-8") as f:
                data = json.load(f)
            # Knowledge-base exports wrap the map as {"type": ..., "resource": {...}}
            if isinstance(data.get("resource"), dict):
                data = data["resource"]
            if norm_id(data.get("id")) == norm_id(q) or str(data.get("name", "")).lower() == q:
                self._data = data
                return data
        raise ValueError(f'Simulation map "{self.query}" not found in {self.folder}')

    def info(self):
        data = self._find()
        return {"id": norm_id(data["id"]), "name": data["name"]}

    def grid(self, frame):
        data = self._find()
        size = data["size"]
        width, height = int(size["width_in_pixels"]), int(size["height_in_pixels"])
        if not width or not height or width * height > MAX_CELLS:
            raise ValueError("Simulation map is empty or too large")
        raw = base64.b64decode(data["occupancy_grid"])
        if len(raw) != width * height:
            raise ValueError("Simulation map grid size does not match its header")
        origin = data.get("origin") or {}
        # Cerebra stores unknown as 255; the dashboard uses the ROS -1 convention.
        return {"width": width, "height": height,
                "resolution": float(size["width_in_meters"]) / width,
                "origin": [float(origin.get("x", 0.0)), float(origin.get("y", 0.0)), 0.0],
                "frame": frame, "name": data["name"],
                "data": [v if v <= 100 else -1 for v in raw]}
