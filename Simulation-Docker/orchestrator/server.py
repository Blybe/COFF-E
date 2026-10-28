"""COFF-E orchestrator server: ties the robot, the arm and the mission
engine together and serves the dashboard.

  GET  /             the dashboard (dashboard/index.html)
  GET  /api/status   everything the dashboard polls (robot, arm, mission,
                    step states, log, mission list, per-target state)
  POST /api/commands {"doc": {...}} - the raw ROS commands of every step, as
                    the dashboard's RAW view shows them (per target)
  POST /api/target   {"target": "real"} - create the links of a target now
                    (connect to the real robot) instead of at the first run
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
  plus the target variables of targets.py (SIM_DRIVE_MODE, REAL_ROS_DOMAIN_ID,
  REAL_ROBOT_IP, REAL_XARM_HOST, ...) for running on the real robots
"""

import json
import os
import sys
import threading
import traceback

from flask import Flask, jsonify, request, send_from_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import commands  # noqa: E402
import mission_model  # noqa: E402
import targets  # noqa: E402
from engine import MissionEngine  # noqa: E402
from runner import Runner  # noqa: E402
from targets import Hardware  # noqa: E402

MISSIONS_DIR = os.environ.get("MISSIONS_DIR", "/missions")
DASHBOARD_DIR = os.environ.get("DASHBOARD_DIR",
                               os.path.join(os.path.dirname(__file__), "dashboard"))
DASHBOARD_PORT = int(os.environ.get("DASHBOARD_PORT", "8080"))

_print_lock = threading.Lock()


def log(text):
    """stdout is shared by Flask and the background threads."""
    with _print_lock:
        print(text, flush=True)


# The hardware of the run: the simulation, the real robots, or both
# (targets.py). The Hardware object implements the robot interface *and* the
# arm interface the engine uses, so it is passed as both; the engine also gets
# it as `hw` for the target bookkeeping (which ROS domains are in play).
hardware = Hardware(env=os.environ, log=log)
engine = MissionEngine(hardware, hardware, hw=hardware, runner=Runner())

app = Flask(__name__)


@app.get("/")
def index():
    return send_from_directory(DASHBOARD_DIR, "index.html")


@app.get("/api/status")
def status():
    return jsonify({
        "mission": engine.snapshot(),
        # robot/arm = the target of the current run (the simulation while it
        # is one of them); targets = every target that exists, for the chips
        "robot": hardware.robot_snapshot(),
        "arm": hardware.arm_snapshot(),
        "targets": hardware.snapshot(),
        "target_labels": targets.TARGET_LABEL,
        "missions": mission_model.list_missions(MISSIONS_DIR),
        "arm_poses": mission_model.ARM_POSES,
        "arm_pose_info": mission_model.ARM_POSE_INFO,
        "tuck_pose": mission_model.TUCK_POSE,
    })


@app.post("/api/commands")
def api_commands():
    """The raw ROS commands of every step of a mission document.

    Both targets are always listed (simulation and real), so the dashboard can
    show what a step sends today and what it will send on the real robots.
    """
    data = request.get_json(silent=True) or {}
    try:
        mission = _doc_mission(data)
    except mission_model.MissionError as exc:
        return jsonify(error=str(exc)), 400
    cfgs = [targets.config(name, os.environ) for name in (targets.SIM, targets.REAL)]
    return jsonify(commands.mission_commands(mission, cfgs, hardware.waypoint))


@app.post("/api/target")
def api_target():
    """Create the links of a target now: the dashboard calls this when the
    "Run on" switch is set to real/both, so the real robot (and the real arm)
    can be checked before a mission is started."""
    data = request.get_json(silent=True) or {}
    target = data.get("target")
    if target not in targets.TARGETS:
        return jsonify(error=f"target must be one of {', '.join(targets.TARGETS)}"), 400
    names = [targets.SIM, targets.REAL] if target == targets.BOTH else [target]
    try:
        for name in names:
            hardware.connect(name)
    except Exception as exc:
        log(f"[dashboard] cannot connect {target}: {exc}")
        return jsonify(error=f"cannot connect {target}: {exc}"), 500
    return jsonify(ok=True, target=target, targets=hardware.snapshot())


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
    log(f"[server] missions in {MISSIONS_DIR}")
    for name, info in hardware.snapshot().items():
        log(f"[server] target {name}: {info['label']} - cmd_vel on "
            f"{info['cmd_vel_topic']} (ROS domain {info['domain_id']}), "
            f"arm at {info['arm_host'] or 'none'}")
    app.run(host="0.0.0.0", port=DASHBOARD_PORT, threaded=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
        raise
