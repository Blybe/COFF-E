"""COFF-E orchestrator server: ties the robot, the arm and the mission
engine together and serves the dashboard.

  GET  /             the dashboard (dashboard/index.html)
  GET  /api/status   everything the dashboard polls (robot, arm, mission,
                    step states, log, mission list)
  POST /api/start    {"mission": "quick_check.yaml"} - start a mission file,
                    or {"doc": {...}, "file": "x.yaml"} - start the mission
                    from the step editor without saving it first
  POST /api/abort    abort the running mission
  GET  /api/mission/<file>   the mission file as written, for the editor
  POST /api/validate {"doc": {...}} - check an edited mission
  POST /api/save     {"file": "x.yaml", "doc": {...}} - write a mission file
  POST /api/live_edit {"doc": {...}} - replace the steps of the running
                    mission that have not started yet

Environment variables:
  XARM_HOST       address of the (simulated) xArm control box
                  [192.168.100.200, the ufactory-studio container]
  CEREBRA_SCENE   scene.json with the waypoints of the loaded Cerebra map
                  [/tmp/cerebra/scene.json]
  MISSIONS_DIR    folder with the mission files [/missions]
  DASHBOARD_DIR   folder with index.html [dashboard, next to this file]
  DASHBOARD_PORT  port of the dashboard [8080]
"""

import json
import os
import sys
import threading
import traceback

from flask import Flask, jsonify, request, send_from_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mission_model  # noqa: E402
from arm_control import Arm  # noqa: E402
from engine import MissionEngine  # noqa: E402
from ros_robot import Robot  # noqa: E402

XARM_HOST = os.environ.get("XARM_HOST", "192.168.100.200")
CEREBRA_SCENE = os.environ.get("CEREBRA_SCENE", "/tmp/cerebra/scene.json")
MISSIONS_DIR = os.environ.get("MISSIONS_DIR", "/missions")
DASHBOARD_DIR = os.environ.get("DASHBOARD_DIR",
                               os.path.join(os.path.dirname(__file__), "dashboard"))
DASHBOARD_PORT = int(os.environ.get("DASHBOARD_PORT", "8080"))

_print_lock = threading.Lock()


def log(text):
    """stdout is shared by Flask and the background threads."""
    with _print_lock:
        print(text, flush=True)


# The three pieces of hardware (simulated):
robot = Robot(scene_file=CEREBRA_SCENE, log=log)
arm = Arm(host=XARM_HOST, log=log)
engine = MissionEngine(robot, arm)

app = Flask(__name__)


@app.get("/")
def index():
    return send_from_directory(DASHBOARD_DIR, "index.html")


@app.get("/api/status")
def status():
    return jsonify({
        "mission": engine.snapshot(),
        "robot": robot.snapshot(),
        "arm": arm.snapshot(),
        "missions": mission_model.list_missions(MISSIONS_DIR),
    })


def _doc_mission(data):
    """Normalized mission from an editor document, or raises MissionError."""
    doc = data.get("doc")
    if not isinstance(doc, dict):
        raise mission_model.MissionError("missing mission document")
    return mission_model.normalize_mission(mission_model.clean_doc(doc), origin="editor")


@app.get("/api/mission/<path:fname>")
def get_mission(fname):
    if fname not in os.listdir(MISSIONS_DIR):
        return jsonify(error=f'mission "{fname}" not found'), 404
    try:
        doc = mission_model.read_mission_doc(os.path.join(MISSIONS_DIR, fname))
    except mission_model.MissionError as exc:
        return jsonify(error=str(exc)), 400
    return jsonify(file=fname, doc=doc)


@app.post("/api/validate")
def validate():
    try:
        mission = _doc_mission(request.get_json(silent=True) or {})
    except mission_model.MissionError as exc:
        return jsonify(ok=False, error=str(exc))
    return jsonify(ok=True, steps=len(mission["steps"]))


@app.post("/api/save")
def save():
    data = request.get_json(silent=True) or {}
    try:
        fname = mission_model.safe_filename(data.get("file"))
        _doc_mission(data)  # never write a mission that cannot run
        text = mission_model.dump_mission(data["doc"], fname)
    except mission_model.MissionError as exc:
        return jsonify(error=str(exc)), 400
    try:
        with open(os.path.join(MISSIONS_DIR, fname), "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    except OSError as exc:
        return jsonify(error=f"cannot write {fname}: {exc}"), 500
    log(f"[dashboard] saved mission {fname}")
    return jsonify(ok=True, file=fname)


@app.post("/api/live_edit")
def live_edit():
    try:
        mission = _doc_mission(request.get_json(silent=True) or {})
        engine.update_pending(mission)
    except mission_model.MissionError as exc:
        return jsonify(error=str(exc)), 400
    except RuntimeError as exc:
        return jsonify(error=str(exc)), 409
    return jsonify(ok=True)


@app.post("/api/start")
def start():
    data = request.get_json(silent=True) or {}
    if data.get("doc") is not None:
        try:
            mission = _doc_mission(data)
            run_id = engine.start(mission, mission_file=str(data.get("file") or "(editor)"))
        except mission_model.MissionError as exc:
            return jsonify(error=str(exc)), 400
        except RuntimeError as exc:
            return jsonify(error=str(exc)), 409
        return jsonify(ok=True, mission=data.get("file"), run_id=run_id)
    missions = mission_model.list_missions(MISSIONS_DIR)
    if not missions:
        return jsonify(error=f"no mission files found in {MISSIONS_DIR}"), 400
    chosen = data.get("mission") or missions[0]["file"]
    entry = next((m for m in missions if m["file"] == chosen), None)
    if entry is None:
        return jsonify(error=f'mission "{chosen}" not found'), 400
    if entry["error"]:
        return jsonify(error=entry["error"]), 400
    try:
        mission = mission_model.load_mission(os.path.join(MISSIONS_DIR, chosen))
    except mission_model.MissionError as exc:
        return jsonify(error=str(exc)), 400
    try:
        run_id = engine.start(mission, mission_file=chosen)
    except RuntimeError as exc:
        return jsonify(error=str(exc)), 409
    return jsonify(ok=True, mission=chosen, run_id=run_id)


@app.post("/api/abort")
def abort():
    engine.abort()
    log("[dashboard] abort requested")
    return jsonify(ok=True)


def main():
    log(f"[server] COFF-E orchestrator dashboard: http://localhost:{DASHBOARD_PORT}")
    log(f"[server] missions in {MISSIONS_DIR}, arm at {XARM_HOST}")
    app.run(host="0.0.0.0", port=DASHBOARD_PORT, threaded=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
        raise
