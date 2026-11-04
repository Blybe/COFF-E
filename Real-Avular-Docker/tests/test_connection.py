import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "app"))
from connection import Source


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
            data["ssid"] = "Origin somethingCR7"
            path.write_text(json.dumps(data))
            value = source.snapshot()
            self.assertFalse(value["host_wifi"]["matches"])
            self.assertEqual(value["host_wifi"]["known_robot"], "Ronaldo")

    def test_missing_stale_partial_and_future_snapshot_not_reported_live(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "wifi.json"
            source = Source(path, "Chud", "RK5", "", clock=lambda: 100)
            self.assertIsNone(source.snapshot()["host_wifi"])
            for value in ('{', '{"updated": 1}', '{"updated": 101}', '{"updated": "bad"}'):
                path.write_text(value)
                self.assertIsNone(source.snapshot()["host_wifi"])
