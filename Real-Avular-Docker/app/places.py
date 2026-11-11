"""Named destinations per onboard map, stored as JSON on the host."""

import json
import math
import os
import threading


class Store:
    def __init__(self, path):
        self.path, self.lock = path, threading.Lock()

    def _read(self):
        try:
            with open(self.path, encoding="utf-8") as f:
                data = json.load(f)
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def _write(self, data):
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, sort_keys=True)
        os.replace(tmp, self.path)

    def list(self, map_id):
        with self.lock:
            return list(self._read().get(str(map_id), []))

    def save(self, map_id, name, x, y, yaw):
        name = str(name).strip()
        if not 1 <= len(name) <= 40:
            raise ValueError("Place name must be 1-40 characters")
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError("Place coordinates must be finite numbers")
        with self.lock:
            data = self._read()
            places = [p for p in data.get(str(map_id), []) if p.get("name") != name]
            places.append({"name": name, "x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3)})
            data[str(map_id)] = sorted(places, key=lambda p: p["name"].lower())
            self._write(data)
            return data[str(map_id)]

    def delete(self, map_id, name):
        with self.lock:
            data = self._read()
            data[str(map_id)] = [p for p in data.get(str(map_id), []) if p.get("name") != name]
            self._write(data)
            return data[str(map_id)]

    def rename(self, map_id, old, new):
        new = str(new).strip()
        if not 1 <= len(new) <= 40:
            raise ValueError("Place name must be 1-40 characters")
        with self.lock:
            data = self._read()
            places = data.get(str(map_id), [])
            if not any(p.get("name") == old for p in places):
                raise ValueError(f'No saved place "{old}"')
            if new != old and any(p.get("name") == new for p in places):
                raise ValueError(f'A place called "{new}" already exists')
            for p in places:
                if p.get("name") == old:
                    p["name"] = new
            data[str(map_id)] = sorted(places, key=lambda p: p["name"].lower())
            self._write(data)
            return data[str(map_id)]
