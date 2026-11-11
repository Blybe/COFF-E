"""Local-only dashboard. All motion POSTs require a same-origin page token."""

import faulthandler
import html
import json
import os
import secrets
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from control import Controller


def make_handler(controller, token, connection=None, robot_name="the robot"):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def state(self):
            value = controller.snapshot()
            if connection:
                value["default_map"] = connection.snapshot()["default_map"]
            return value

        def reply(self, code, body, content_type="application/json"):
            data = body.encode("utf-8") if isinstance(body, str) else json.dumps(body, allow_nan=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            try:
                if self.path == "/":
                    page = Path(__file__).with_name("index.html").read_text(encoding="utf-8")
                    return self.reply(200, page.replace("__TOKEN__", token).replace("__ROBOT__", html.escape("the robot" if connection else robot_name)), "text/html; charset=utf-8")
                if self.path == "/api/state":
                    return self.reply(200, self.state())
                if self.path == "/api/maps":
                    return self.reply(200, controller.list_maps())
                if self.path == "/api/map":
                    return self.reply(200, controller.grid)
                if self.path == "/api/overlay":
                    return self.reply(200, controller.robot.overlay())
                if self.path == "/api/connection":
                    return self.reply(200, connection.snapshot() if connection else {})
                if self.path == "/api/places":
                    return self.reply(200, controller.list_places())
                self.reply(404, {"error": "Not found"})
            except Exception as exc:
                self.reply(503, {"error": str(exc)})

        def do_POST(self):
            if not secrets.compare_digest(self.headers.get("X-COFFE-Token", ""), token):
                return self.reply(403, {"error": "Open the local dashboard to issue commands"})
            if self.headers.get("Sec-Fetch-Site") == "cross-site":
                return self.reply(403, {"error": "Cross-site commands are not allowed"})
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 4096:
                    raise ValueError("Invalid request size")
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict):
                    raise ValueError("Expected a JSON object")
                owner = body.get("owner")
                if not isinstance(owner, str) or not 1 <= len(owner) <= 100:
                    raise ValueError("Missing browser operator ID")
                if self.path == "/api/enable":
                    controller.enable(owner, body.get("confirmed"))
                elif self.path == "/api/heartbeat":
                    controller.ping(owner)
                elif self.path == "/api/stop":
                    if not controller.stop():
                        raise ValueError("STOP UNCONFIRMED. Use physical e-stop and inspect the log.")
                elif self.path == "/api/release":
                    controller.release()
                elif self.path == "/api/drive":
                    controller.drive(owner, body["linear"], body["angular"])
                elif self.path == "/api/load-map":
                    controller.load_map(body["id"])
                elif self.path == "/api/load-sim-map":
                    controller.load_sim_map()
                elif self.path == "/api/seed":
                    controller.seed(body["x"], body["y"], body["yaw"])
                elif self.path == "/api/confirm-localization":
                    controller.confirm_localization(body.get("confirmed"))
                elif self.path == "/api/navigate":
                    controller.navigate(owner, body["x"], body["y"], body["yaw"])
                elif self.path == "/api/fit-pose":
                    return self.reply(200, controller.fit_pose(body["around"]))
                elif self.path == "/api/places/save":
                    return self.reply(200, controller.save_place(body["name"], body["x"], body["y"], body["yaw"]))
                elif self.path == "/api/places/delete":
                    return self.reply(200, controller.delete_place(body["name"]))
                elif self.path == "/api/places/rename":
                    return self.reply(200, controller.rename_place(body["name"], body["new_name"]))
                else:
                    return self.reply(404, {"error": "Not found"})
                self.reply(200, self.state())
            except (ValueError, KeyError, TypeError) as exc:
                self.reply(400, {"error": str(exc)})
            except Exception as exc:
                controller.stop("Command exception")
                self.reply(503, {"error": str(exc)})

    return Handler


def main():
    # `docker kill -s USR1 <container>` prints every thread's stack to the logs.
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    from ros_link import Robot
    from sim_map import Source
    from connection import Source as Connection
    from places import Store
    robot = Robot()
    map_id = os.environ.get("ROBOT_MAP_ID", "")
    controller = Controller(robot, sim_map=Source(os.environ.get("SIM_MAP_DIR", "/sim_maps"),
                                                os.environ.get("SIM_MAP", "JMH - 3rd floor")),
                            places=Store("/places/places.json"),
                            default_map={"id": map_id, "name": os.environ.get("ROBOT_MAP_NAME", map_id)}
                            if map_id else None,
                            max_linear=float(os.environ.get("MANUAL_MAX_LINEAR", "0.6")),
                            max_angular=float(os.environ.get("MANUAL_MAX_ANGULAR", "1.0")))
    quit_event = threading.Event()
    connection = Connection("/host_status/wifi.json", os.environ.get("ROBOT_NAME", "Chud"),
                            os.environ.get("ROBOT_WIFI_SUFFIX", "RK5"),
                            os.environ.get("REAL_ROBOT_IP", ""), default_map=controller.default_map)
    server = ThreadingHTTPServer(("0.0.0.0", 8080),
                                make_handler(controller, secrets.token_hex(32), connection,
                                             os.environ.get("ROBOT_NAME", "the robot")))
    server.daemon_threads = True

    def watchdog():
        while not quit_event.wait(0.1):
            try:
                controller.tick()
            except Exception as exc:
                controller.log("Watchdog error: " + str(exc))
                controller.stop("Watchdog exception")

    def shutdown(*_):
        quit_event.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    thread = threading.Thread(target=watchdog, daemon=True)
    thread.start()
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        quit_event.set()
        thread.join(timeout=10)
        # Do not touch hardware on read-only startup/shutdown.
        if controller.holding or controller.enabled:
            controller.stop("Container shutting down; physical stop remains required")
        server.server_close()
        robot.close()


if __name__ == "__main__":
    main()
