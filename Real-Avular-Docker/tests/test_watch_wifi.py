"""Exercise the actual PowerShell monitor without accessing WLAN or credentials."""

import json
import os
import subprocess
import tempfile
import unittest
import uuid
from pathlib import Path


@unittest.skipUnless(os.name == "nt", "Windows WLAN monitor")
class WifiMonitorTests(unittest.TestCase):
    def monitor(self, interfaces, adapter=None):
        script = Path(__file__).resolve().parents[1] / "Watch-Wifi.ps1"
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder) / script.name
            # Isolate the single-instance mutex as well as the output folder,
            # so tests can run while the operator's real monitor stays active.
            target.write_text(script.read_text(encoding="utf-8").replace(
                "Local\\COFFE-Real-Avular-Wifi",
                "Local\\COFFE-Wifi-Test-" + uuid.uuid4().hex), encoding="utf-8-sig")
            lines = "\n".join(
                f"    Name : {name}\n    State : {state}\n    SSID : {ssid}\n    Signal : 84%"
                for name, state, ssid in interfaces)
            # Mock only netsh. The parser, selection, mutex and atomic writer
            # are the production script, copied beside an isolated status folder.
            command = "function netsh { @'\n" + lines + "\n'@ -split \"`n\" }\n"
            command += f"& '{str(target).replace(chr(39), chr(39) * 2)}' -Once"
            if adapter:
                command += " -Adapter '" + adapter + "'"
            wrapper = Path(folder) / "run.ps1"
            wrapper.write_text(command, encoding="utf-8-sig")
            run = subprocess.run(["powershell.exe", "-NoProfile", "-NonInteractive",
                                  "-File", str(wrapper)], capture_output=True, text=True, timeout=15)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            return json.loads((Path(folder) / "status" / "wifi.json").read_text())

    def test_finds_robot_not_internet_adapter_in_either_order(self):
        for name, ssid in (("WiFi", "Origin PB4U9RK5"), ("WiFi 2", "Origin PB4U9CR7")):
            for reverse in (False, True):
                interfaces = [("WiFi 3", "connected", "eduroam"), (name, "connected", ssid)]
                value = self.monitor(interfaces[::-1] if reverse else interfaces)
                self.assertEqual((value["adapter"], value["ssid"], value["signal"]), (name, ssid, 84))
                self.assertFalse(value["ambiguous"])

    def test_unknown_disconnected_and_explicit_adapter(self):
        value = self.monitor([("WiFi", "disconnected", "Origin PB4U9RK5"),
                              ("WiFi 3", "connected", "eduroam")])
        self.assertIsNone(value["ssid"])
        value = self.monitor([("WiFi", "connected", "Origin PB4U9RK5"),
                              ("WiFi 3", "connected", "eduroam")], adapter="WiFi 3")
        self.assertEqual(value["ssid"], "eduroam")

    def test_two_robot_adapters_are_ambiguous_even_with_explicit_adapter(self):
        interfaces = [("WiFi", "connected", "Origin PB4U9RK5"),
                      ("WiFi 2", "connected", "Origin PB4U9CR7")]
        for adapter in (None, "WiFi"):
            value = self.monitor(interfaces, adapter)
            self.assertTrue(value["ambiguous"])
            self.assertIsNone(value["ssid"])
