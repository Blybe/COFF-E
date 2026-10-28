"""Runs a raw shell command for one mission step.

A step of type "command" is free text, entered in the dashboard's raw editor,
and that text is exactly what runs here: bash inside the orchestrator
container, with the ROS 2 environment sourced first. Output is collected line
by line, and the command runs in its own process group, so Abort (or a step
timeout) really stops it - a `ros2 topic pub` that keeps publishing after the
mission stopped would be a safety problem.
"""

import os
import signal
import subprocess
import threading
import time

ROS_SETUP = "/opt/ros/humble/setup.bash"
AVULAR_SETUP = "/opt/avular_msgs/install/setup.bash"
KILL_GRACE = 0.7        # s between the "stop" signal and SIGKILL


def _bootstrap():
    """Shell prelude: the ROS 2 environment (and the Avular messages) the
    user's command may rely on. Sourced once per command, so a `ros2 topic
    pub` typed by hand behaves the same as one the orchestrator runs."""
    return (f'if [ -f {ROS_SETUP} ]; then . {ROS_SETUP}; fi\n'
            f'if [ -f {AVULAR_SETUP} ]; then . {AVULAR_SETUP}; fi\n')


class RunningCommand:
    """A command running in its own process group, so it can be stopped as a
    whole (a `ros2 topic pub` spawns child processes)."""

    def __init__(self, command, popen):
        self.command = command
        self._proc = popen
        self._lines = []
        self._lock = threading.Lock()
        self.killed = False
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        try:
            for line in self._proc.stdout:
                with self._lock:
                    self._lines.append(line.rstrip("\r\n"))
        except (OSError, ValueError):
            pass  # the pipe was closed (process killed)

    def poll(self):
        """None while the command runs, else its exit code."""
        return self._proc.poll()

    def output(self, limit=2000):
        with self._lock:
            text = "\n".join(self._lines)
        return text[-limit:] if len(text) > limit else text

    def tail(self, limit=160):
        with self._lock:
            lines = list(self._lines)
        return lines[-1][:limit] if lines else ""

    def kill(self):
        """Stop the command (and everything it started)."""
        if self._proc.poll() is not None:
            return
        self.killed = True
        self._signal(signal.SIGINT)     # what a user would press
        deadline = time.time() + KILL_GRACE
        while time.time() < deadline and self._proc.poll() is None:
            time.sleep(0.05)
        if self._proc.poll() is None:
            self._signal(signal.SIGKILL)

    def _signal(self, sig):
        try:
            os.killpg(os.getpgid(self._proc.pid), sig)
        except OSError:
            try:
                self._proc.send_signal(sig)
            except OSError:
                pass


class Runner:
    """Starts raw commands; pure subprocess work, so it is unit tested with a
    fake in the engine tests and with real commands in the smoke test."""

    def __init__(self, env=None):
        self.env = env

    def start(self, command, domain_id=None):
        env = dict(self.env if self.env is not None else os.environ)
        env["PYTHONUNBUFFERED"] = "1"
        if domain_id is not None:
            # The real robot lives in its own ROS 2 domain (see targets.py).
            env["ROS_DOMAIN_ID"] = str(domain_id)
        proc = subprocess.Popen(
            ["bash", "-lc", _bootstrap() + command + "\n"],
            env=env, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1,
            start_new_session=True)
        return RunningCommand(command, proc)

    def run(self, command, domain_id=None, timeout=30.0):
        """Run a command to completion: returns (exit_code, output), with
        exit_code -1 when it had to be killed after `timeout` seconds."""
        cmd = self.start(command, domain_id=domain_id)
        deadline = time.time() + timeout
        while cmd.poll() is None:
            if time.time() > deadline:
                cmd.kill()
                return -1, cmd.output()
            time.sleep(0.05)
        return cmd.poll(), cmd.output()
