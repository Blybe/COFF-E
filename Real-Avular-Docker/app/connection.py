"""Read-only laptop Wi-Fi snapshot; never reads wireless credentials."""

import json
import threading
import time
from pathlib import Path


ROBOTS = {
    "Origin PB4U9RK5": {"name": "Chud", "suffix": "RK5",
                       "map": {"id": "b3704c57-6e5b-49dc-b121-ab55ecae9857",
                               "name": "JMH-03-D07-10-26-PT-1"}},
    "Origin PB4U9CR7": {"name": "Ronaldo", "suffix": "CR7",
                       "map": {"id": "da573728-35da-4fd2-9784-a211f9850d1b",
                               "name": "JMH - 3rd floor"}},
}


class Source:
    def __init__(self, path, name, suffix, ip, clock=time.time, default_map=None,
                 monotonic=time.monotonic):
        self.path, self.clock, self.monotonic = Path(path), clock, monotonic
        self.name, self.suffix, self.ip = name, suffix, ip
        self.default_map = default_map
        self._lock = threading.Lock()
        self._last_updated = None
        self._changed_at = None

    def _fresh(self, updated):
        # The Docker Desktop VM clock can drift from Windows (e.g. after
        # sleep), so a timestamp that keeps advancing also proves the monitor
        # is alive, measured on this process's own monotonic clock.
        with self._lock:
            now = self.monotonic()
            if self._last_updated is not None and updated != self._last_updated:
                self._changed_at = now if updated > self._last_updated else None
            self._last_updated = updated
            if self._changed_at is not None and now - self._changed_at < 15:
                return True
        return 0 <= self.clock() - updated < 15

    def snapshot(self):
        result = {"name": "Unknown robot", "wifi_suffix": None, "ip": self.ip,
                  "detected": False, "configured_name": self.name, "default_map": None,
                  "host_wifi": None, "host_status": "Laptop Wi-Fi monitor unavailable"}
        try:
            data = json.loads(self.path.read_text(encoding="utf-8-sig"))
            if not isinstance(data, dict):
                return result
            if not self._fresh(float(data["updated"])):
                result["host_status"] = "Laptop Wi-Fi monitor stale"
                return result
            wifi = {k: data.get(k) for k in ("adapter", "ssid", "signal", "state")}
            ssid = str(wifi["ssid"] or "")
            # Exact known SSIDs, not arbitrary networks with the same suffix.
            robot = next((r for name, r in ROBOTS.items() if name.casefold() == ssid.casefold()), None)
            if str(wifi["state"]).casefold() != "connected" or data.get("ambiguous"):
                robot = None
            wifi["matches"] = robot is not None
            wifi["known_robot"] = robot["name"] if robot else None
            if robot:
                result.update(name=robot["name"], wifi_suffix=robot["suffix"], detected=True,
                              default_map=(self.default_map if self.default_map and
                                           self.suffix.upper() == robot["suffix"] else robot["map"]))
            result.update(host_wifi=wifi, host_status="Laptop Wi-Fi monitor live")
            if data.get("ambiguous"):
                result["host_status"] = "Multiple robot Wi-Fi adapters connected; disconnect one"
        except (OSError, ValueError, TypeError, KeyError):
            pass
        return result
