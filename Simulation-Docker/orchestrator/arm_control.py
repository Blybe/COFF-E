"""xArm side of the orchestrator: the simulated UFACTORY xArm 6.

The arm simulation (ufactory-studio container) does not speak ROS 2 - like a
real xArm control box it speaks the UFACTORY TCP protocol, so this module uses
the official xArm Python SDK, the same one used against real hardware.

A background thread (re)connects to the simulated control box, enables
motion, and polls the joint angles a few times per second for the dashboard.
The engine uses command_joints() and angles(); every SDK call goes through
one lock because the SDK shares a single TCP connection and report thread.

The same class drives the real xArm: only the host changes (REAL_XARM_HOST),
so the commands the dashboard shows for the simulation are the commands that
go to the real arm. An empty host means "this target has no arm": the monitor
thread is then not started at all, instead of logging connection errors.
"""

import threading
import time

POLL_PERIOD = 0.25      # s between angle polls while connected
RECONNECT_PERIOD = 8.0  # s between connect attempts while disconnected


def controller_error_text(code):
    """'controller error C2: The Emergency IO of the Control Box is triggered'
    - the SDK's own description of a controller error code."""
    try:
        from xarm.core.config.x_code import ControllerErrorCodeMap
        title = ControllerErrorCodeMap.get(code, {}).get("en", {}).get("title", "")
    except Exception:
        title = ""
    return f"controller error C{code}" + (f": {title}" if title else "")


class Arm:
    def __init__(self, host="192.168.100.200", log=print, sleep=time.sleep):
        self.host = (host or "").strip()
        self._log = log
        self._sleep = sleep
        self._lock = threading.Lock()
        self._api = None
        self._angles = None   # list of 6 joint angles in degrees
        self._state = "disconnected"  # disconnected / connecting / ready / error
        self._error = ""
        self._fault = ""      # the controller's error, e.g. an emergency stop
        self._thread = None
        if not self.host:
            self._state = "not configured"
            self._error = "no xArm host configured for this target"
        else:
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
        self._set_state("ready", f"connected to the xArm at {self.host}")

    def _poll(self):
        # Query the angles instead of reading api.angles: that cache is filled
        # by the SDK's report stream, which on the real xArm stayed at all-zero
        # on a long-lived connection while the arm stood at J2 -90 / J5 90.
        with self._lock:
            if self._api is None:
                return
            # A restarted control box (or arm simulation) closes the socket;
            # the SDK then keeps answering with its last values instead of
            # failing, so the monitor has to reconnect by itself.
            if not self._api.connected:
                raise RuntimeError("the control box closed the connection")
            code, angles = self._api.get_servo_angle()
            err_code, codes = self._api.get_err_warn_code()
        # The SDK reports 7 values for an xArm 6 (6 joints + a gripper slot);
        # the orchestrator only cares about the 6 joints.
        if code == 0 and angles:
            self._angles = [float(a) for a in list(angles)[:6]]
        if err_code == 0:
            fault = controller_error_text(codes[0]) if codes and codes[0] else ""
            if fault != self._fault:
                self._fault = fault
                if fault:
                    self._set_state("error", fault)
                else:
                    self._set_state("ready", f"connected to the xArm at {self.host}")

    # ------------------------------------------------------------ engine

    def fault(self):
        """The controller error that blocks motion ("" when there is none)."""
        return self._fault

    def angles(self):
        """List of joint angles in degrees, or None when disconnected."""
        with self._lock:
            return None if self._api is None else self._angles

    def command_joints(self, angles_deg, speed, acc=None, wait=False):
        """Move the joints; returns the SDK code (0 = ok).

        acc (deg/s^2) and wait are the raw overrides of the step: with
        acc=None the SDK picks its own acceleration, and wait=True makes the
        SDK call block until the arm has reached the target."""
        kwargs = {"angle": [float(a) for a in angles_deg], "speed": float(speed),
                  "wait": bool(wait)}
        if acc is not None:
            kwargs["mvacc"] = float(acc)
        with self._lock:
            if self._api is None:
                return -1
            return self._api.set_servo_angle(**kwargs)

    # ------------------------------------------------------------ dashboard

    def snapshot(self):
        with self._lock:
            return {
                "connected": self._api is not None and self._state == "ready",
                "state": self._state,
                "error": self._error,
                "host": self.host or None,
                "angles": self._angles,
            }
