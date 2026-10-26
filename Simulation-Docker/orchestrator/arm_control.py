"""xArm side of the orchestrator: the simulated UFACTORY xArm 6.

The arm simulation (ufactory-studio container) does not speak ROS 2 - like a
real xArm control box it speaks the UFACTORY TCP protocol, so this module uses
the official xArm Python SDK, the same one used against real hardware.

A background thread (re)connects to the simulated control box, enables
motion, and polls the joint angles a few times per second for the dashboard.
The engine uses command_joints() and angles(); every SDK call goes through
one lock because the SDK shares a single TCP connection and report thread.
"""

import threading
import time

POLL_PERIOD = 0.25      # s between angle polls while connected
RECONNECT_PERIOD = 8.0  # s between connect attempts while disconnected


class Arm:
    def __init__(self, host="192.168.100.200", log=print, sleep=time.sleep):
        self.host = host
        self._log = log
        self._sleep = sleep
        self._lock = threading.Lock()
        self._api = None
        self._angles = None   # list of 6 joint angles in degrees
        self._state = "disconnected"  # disconnected / connecting / ready / error
        self._error = ""
        self._thread = threading.Thread(target=self._monitor, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------ monitor

    def _monitor(self):
        while True:
            try:
                if self._api is None:
                    self._connect()
                else:
                    self._poll()
                    self._sleep(POLL_PERIOD)
            except Exception as exc:
                self._set_state("error", f"connection lost: {exc}")
                self._close()

    def _set_state(self, state, error=""):
        with self._lock:
            self._state = state
            self._error = error
        if error:
            self._log(f"[arm] {state}: {error}")
        else:
            self._log(f"[arm] {state}")

    def _close(self):
        with self._lock:
            api, self._api, self._angles = self._api, None, None
        if api is not None:
            try:
                api.disconnect()
            except Exception:
                pass
        self._sleep(RECONNECT_PERIOD)

    def _connect(self):
        self._set_state("connecting")
        from xarm.wrapper import XArmAPI  # lazy: keeps unit tests SDK-free
        api = XArmAPI(self.host, baud_checkset=False, check_joint_limit=False)
        deadline = time.time() + 8.0
        while not getattr(api, "connected", False) and time.time() < deadline:
            self._sleep(0.2)
        if not getattr(api, "connected", False):
            raise RuntimeError(f"cannot reach the xArm at {self.host}")
        api.clean_error()
        api.motion_enable(enable=True)
        api.set_mode(0)   # 0 = position mode, like the Studio UI
        api.set_state(0)  # 0 = ready
        with self._lock:
            self._api = api
        self._set_state("ready", f"connected to the simulated xArm at {self.host}")

    def _poll(self):
        with self._lock:
            angles = self._api.angles if self._api else None
        # The SDK reports 7 values for an xArm 6 (6 joints + a gripper slot);
        # the orchestrator only cares about the 6 joints.
        if angles:
            self._angles = [float(a) for a in list(angles)[:6]]

    # ------------------------------------------------------------ engine

    def angles(self):
        """List of joint angles in degrees, or None when disconnected."""
        with self._lock:
            return None if self._api is None else self._angles

    def command_joints(self, angles_deg, speed):
        """Start a joint move (non-blocking); returns the SDK code (0 = ok)."""
        with self._lock:
            if self._api is None:
                return -1
            return self._api.set_servo_angle(
                angle=[float(a) for a in angles_deg], speed=float(speed), wait=False)

    # ------------------------------------------------------------ dashboard

    def snapshot(self):
        with self._lock:
            return {
                "connected": self._api is not None and self._state == "ready",
                "state": self._state,
                "error": self._error,
                "host": self.host,
                "angles": self._angles,
            }
