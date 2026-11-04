"""Read-only laptop Wi-Fi snapshot; never reads wireless credentials."""

import json
import time
from pathlib import Path


class Source:
    def __init__(self, path, name, suffix, ip, clock=time.time):
        self.path, self.clock = Path(path), clock
        self.name, self.suffix, self.ip = name, suffix, ip

    def snapshot(self):
        result = {"name": self.name, "wifi_suffix": self.suffix, "ip": self.ip,
                  "host_wifi": None, "host_status": "Laptop Wi-Fi monitor unavailable"}
        try:
            data = json.loads(self.path.read_text(encoding="utf-8-sig"))
            age = self.clock() - float(data["updated"])
            if not 0 <= age < 15:
                result["host_status"] = "Laptop Wi-Fi monitor stale"
                return result
            wifi = {k: data.get(k) for k in ("adapter", "ssid", "signal", "state")}
            ssid = str(wifi["ssid"] or "")
            wifi["matches"] = bool(self.suffix and ssid.upper().endswith(self.suffix.upper()))
            wifi["known_robot"] = next((name for suffix, name in
                                       (("RK5", "Chud"), ("CR7", "Ronaldo"))
                                       if ssid.upper().endswith(suffix)), None)
            result.update(host_wifi=wifi, host_status="Laptop Wi-Fi monitor live")
        except (OSError, ValueError, TypeError, KeyError):
            pass
        return result
