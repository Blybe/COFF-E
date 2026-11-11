import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "app"))
from connection import Source, ROBOTS


class ConnectionTests(unittest.TestCase):
    def test_live_host_wifi_matches_chud_and_does_not_echo_extra_data(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "wifi.json"
            path.write_text(json.dumps({"updated": 100, "adapter": "WiFi 2",
                                        "ssid": "Origin PB4U9RK5", "signal": 100,
                                        "state": "connected", "unrelated": "omit"}))
            source = Source(path, "Chud", "RK5", "192.168.192.1", clock=lambda: 101)
            value = source.snapshot()
            self.assertEqual(value["name"], "Chud")
            self.assertTrue(value["host_wifi"]["matches"])
            self.assertEqual(value["host_wifi"]["known_robot"], "Chud")
            self.assertNotIn("unrelated", value["host_wifi"])
            data = json.loads(path.read_text())
            data["ssid"] = "Origin PB4U9CR7"
            path.write_text(json.dumps(data))
            value = source.snapshot()
            self.assertTrue(value["host_wifi"]["matches"])
            self.assertEqual(value["name"], "Ronaldo")
            self.assertTrue(value["detected"])
            self.assertEqual(value["host_wifi"]["known_robot"], "Ronaldo")

    def test_switches_in_both_directions_without_changing_configuration(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "wifi.json"
            configured_map = {"id": "custom", "name": "Custom"}
            source = Source(path, "Ronaldo", "CR7", "192.168.192.1",
                            clock=lambda: 101, default_map=configured_map)
            for ssid in ("Origin PB4U9CR7", "Origin PB4U9RK5", "Origin PB4U9CR7"):
                path.write_text(json.dumps({"updated": 100, "ssid": ssid, "state": "connected"}))
                value = source.snapshot()
                self.assertEqual(value["name"], ROBOTS[ssid]["name"])
                self.assertEqual(value["configured_name"], "Ronaldo")
                self.assertEqual(value["default_map"], configured_map if ssid.endswith("CR7")
                                 else ROBOTS[ssid]["map"])
                self.assertEqual(value["ip"], "192.168.192.1")

    def test_unknown_disconnected_ambiguous_or_stale_never_uses_configured_identity(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "wifi.json"
            source = Source(path, "Ronaldo", "CR7", "", clock=lambda: 100)
            for changes in ({"ssid": "eduroam"}, {"ssid": "OtherCR7"},
                            {"ssid": "Origin somethingCR7"}, {"state": "disconnected"},
                            {"ambiguous": True}, {"updated": 85}, {"updated": 101}):
                value = {"updated": 100, "ssid": "Origin PB4U9CR7", "state": "connected"}
                value.update(changes)
                path.write_text(json.dumps(value))
                result = source.snapshot()
                self.assertFalse(result["detected"], changes)
                self.assertEqual(result["name"], "Unknown robot")
                self.assertIsNone(result["default_map"])

    def test_case_insensitive_exact_ssid(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "wifi.json"
            path.write_text(json.dumps({"updated": 100, "ssid": "origin pb4u9rk5",
                                        "state": "Connected"}))
            self.assertEqual(Source(path, "", "", "", clock=lambda: 100).snapshot()["name"], "Chud")

    def test_missing_stale_partial_and_future_snapshot_not_reported_live(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "wifi.json"
            source = Source(path, "Chud", "RK5", "", clock=lambda: 100)
            self.assertIsNone(source.snapshot()["host_wifi"])
            for value in ('{', 'null', '[]', '{"updated": 1}', '{"updated": 101}', '{"updated": "bad"}'):
                path.write_text(value)
                self.assertIsNone(source.snapshot()["host_wifi"])
