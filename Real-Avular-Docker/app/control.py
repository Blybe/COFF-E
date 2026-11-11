"""Hardware-independent safety gate. No ROS, Docker or web dependencies."""

import math
import threading
import time
from collections import deque


def number(value):
    if isinstance(value, bool):
        raise ValueError("Expected a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("Expected a finite number")
    return value


def world_to_cell(grid, x, y):
    """OccupancyGrid origins can be translated AND rotated."""
    ox, oy, yaw = grid["origin"]
    dx, dy = x - ox, y - oy
    c, s = math.cos(yaw), math.sin(yaw)
    return (math.floor((c * dx + s * dy) / grid["resolution"]),
            math.floor((-s * dx + c * dy) / grid["resolution"]))


def validate_point(grid, x, y):
    col, row = world_to_cell(grid, x, y)
    if not (0 <= col < grid["width"] and 0 <= row < grid["height"]):
        raise ValueError("Point is outside the selected map")
    value = grid["data"][row * grid["width"] + col]
    if value < 0 or value >= 50:
        raise ValueError("Point is in unknown or occupied space")


def scan_match(grid, points, tolerance=0.15):
    """Fraction of laser points (map frame) within tolerance of an occupied cell."""
    k = max(1, math.ceil(tolerance / grid["resolution"]))
    width, height, data = grid["width"], grid["height"], grid["data"]
    hits = 0
    for x, y in points:
        col, row = world_to_cell(grid, x, y)
        if any(0 <= col + dc < width and 0 <= row + dr < height
               and data[(row + dr) * width + col + dc] >= 50
               for dr in range(-k, k + 1) for dc in range(-k, k + 1)):
            hits += 1
    return hits / len(points) if points else 0.0


def wall_mask(grid, tolerance):
    """bytearray: 1 where a cell lies within tolerance of an occupied cell."""
    width, height, data = grid["width"], grid["height"], grid["data"]
    k = max(1, math.ceil(tolerance / grid["resolution"]))
    mask = bytearray(width * height)
    for i, value in enumerate(data):
        if value >= 50:
            row, col = divmod(i, width)
            for r in range(max(0, row - k), min(height, row + k + 1)):
                mask[r * width + max(0, col - k): r * width + min(width, col + k + 1)] = \
                    b"\x01" * (min(width, col + k + 1) - max(0, col - k))
    return mask


def fit_pose(grid, masks, local, center, radius, yaw_span):
    """Candidates (match, x, y, yaw), best first, for robot-frame laser points near center.

    Coarse grid on a generously dilated mask, then a fine search around the
    best coarse candidates. Grid origin rotation is not supported here."""
    if grid["origin"][2]:
        raise ValueError("Auto-fit does not support rotated map origins")
    ox, oy = grid["origin"][:2]
    res, width, height = grid["resolution"], grid["width"], grid["height"]

    def search(mask, points, poses):
        scored = []
        for x, y, yaw in poses:
            c, s = math.cos(yaw), math.sin(yaw)
            hits = 0
            for px, py in points:
                col = math.floor((x + c * px - s * py - ox) / res)
                row = math.floor((y + s * px + c * py - oy) / res)
                if 0 <= col < width and 0 <= row < height and mask[row * width + col]:
                    hits += 1
            scored.append((hits / len(points), x, y, yaw))
        return sorted(scored, reverse=True)

    def steps(span, step):
        n = int(span / step)
        return [i * step for i in range(-n, n + 1)]

    cx, cy, cyaw = center
    sparse = local[::max(1, len(local) // 60)]
    coarse = search(masks["coarse"], sparse,
                    [(cx + dx, cy + dy, cyaw + da) for da in steps(yaw_span, math.radians(8))
                     for dx in steps(radius, 0.25) for dy in steps(radius, 0.25)])
    fine_points = local[::max(1, len(local) // 150)]
    best = []
    for _, x, y, yaw in coarse[:8]:
        best += search(masks["fine"], fine_points,
                       [(x + dx, y + dy, yaw + da) for da in steps(math.radians(8), math.radians(2))
                        for dx in steps(0.3, 0.05) for dy in steps(0.3, 0.05)])[:1]
    return [(m, x, y, math.atan2(math.sin(yaw), math.cos(yaw))) for m, x, y, yaw in sorted(best, reverse=True)]


def clearance(grid, x, y, limit=1.0):
    """Distance to occupied cell edges, capped at limit (not a footprint check)."""
    col, row = world_to_cell(grid, x, y)
    width, height, data, res = grid["width"], grid["height"], grid["data"], grid["resolution"]
    if not (0 <= col < width and 0 <= row < height) or data[row * width + col] < 0:
        return 0.0
    ox, oy, yaw = grid["origin"]
    dx, dy = x - ox, y - oy
    gx = math.cos(yaw) * dx + math.sin(yaw) * dy
    gy = -math.sin(yaw) * dx + math.cos(yaw) * dy
    k = math.ceil(limit / res) + 1
    best = limit
    for dr in range(-k, k + 1):
        for dc in range(-k, k + 1):
            c, r = col + dc, row + dr
            if 0 <= c < width and 0 <= r < height and data[r * width + c] >= 50:
                dx = max(c * res - gx, 0, gx - (c + 1) * res)
                dy = max(r * res - gy, 0, gy - (r + 1) * res)
                best = min(best, math.hypot(dx, dy))
    return best


def open_spot(grid, x, y, yaw, room=0.40, search=2.0):
    """Nearby free map position with 'room' metres of clearance, as
    (ahead, left) metres in the robot frame, or None."""
    res = grid["resolution"]
    for ring in range(1, math.ceil(search / res) + 1):
        options = []
        for dr in range(-ring, ring + 1):
            for dc in range(-ring, ring + 1):
                if max(abs(dr), abs(dc)) != ring:
                    continue
                if math.hypot(dc, dr) * res > search:
                    continue
                px, py = x + dc * res, y + dr * res
                try:
                    validate_point(grid, px, py)
                except ValueError:
                    continue
                if clearance(grid, px, py, room) >= room:
                    options.append((math.hypot(dc, dr), px, py))
        if options:
            _, px, py = min(options)
            dx, dy = px - x, py - y
            return (math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy)
    return None


def drive_hint(spot):
    if not spot:
        return "no open map floor within 2 m"
    ahead, left = spot
    parts = [f"{abs(ahead):.2f} m {'forward' if ahead > 0 else 'back'}" if abs(ahead) >= 0.05 else "",
             f"{abs(left):.2f} m to its {'left' if left > 0 else 'right'}" if abs(left) >= 0.05 else ""]
    return "open map floor is " + " and ".join(p for p in parts if p)


# origin_msgs/msg/ControlMode. From MANUAL (the handheld remote) and the other
# remote/docking modes the robot refuses a switch to USER outright.
MODE_NAMES = {0: "NONE", 10: "MANUAL (remote)", 11: "TELEOPS", 12: "MANUAL MAPPING",
              20: "AUTOPILOT", 21: "DOCKING", 30: "USER (dashboard)"}
REMOTE_MODES = {10, 11, 12, 21}


class Controller:
    FRESH = 2.0
    LEASE = 1.5
    DEADMAN = 0.35
    NAV_TIMEOUT = 180.0
    # the robot's localizer reports a covariance that stays far above any useful
    # limit even when it is right, so the laser-to-map match is the main
    # evidence. Covariance remains the fallback when no laser overlay exists.
    # Measured on Origin Ones in JMH: right pose 48-83 %, wrong pose 12-17 %
    # of points within 0.15 m of a wall; fast turns briefly dip to ~30 %.
    MATCH_MIN = 0.40
    MATCH_POINTS = 100
    NAV_MATCH_MIN = 0.25
    NAV_BAD_GRACE = 3.0
    # The onboard planner refuses with "Starting point in lethal space" when
    # a map wall lies inside the robot's footprint (seen at 0.20-0.25 m).
    START_CLEARANCE = 0.30
    # Hard ceilings regardless of configuration.
    LINEAR_CEILING = 1.0
    ANGULAR_CEILING = 1.5

    def __init__(self, robot, clock=time.monotonic, sim_map=None, places=None, default_map=None,
                 max_linear=0.6, max_angular=1.0):
        self.robot, self.clock = robot, clock
        self.max_linear = min(number(max_linear), self.LINEAR_CEILING)
        self.max_angular = min(number(max_angular), self.ANGULAR_CEILING)
        self.bad_since = None
        self.masks = None
        self.last_behavior = None
        self.last_robot_log = (None, -1e9)
        self.sim_source = sim_map
        self.places = places
        self.default_map = default_map
        self.last_run = None
        self.lock = threading.RLock()
        self.enabled = False
        self.owner = None
        self.heartbeat = -1e9
        self.drive_until = -1e9
        self.velocity = (0.0, 0.0)
        self.mode = "disabled"
        self.grid = None
        self.map_id = None
        self.localized = False
        self.seed_time = -1e9
        self.seed_pose = None
        self.seed_failed = False
        self.map_time = -1e9
        self.goal = None
        self.holding = False
        self.stop_verified = False
        self.events = deque(maxlen=500)
        self.log("Read-only startup. Motion is disabled.")
        self.sim_info = None
        if sim_map is not None:
            try:
                self.sim_info = sim_map.info()
            except Exception as exc:
                self.log("Simulation map unavailable: " + str(exc))

    def log(self, message):
        self.events.append(time.strftime("%H:%M:%S") + "  " + message)

    def remote_mode(self):
        """Name of a fresh control mode that blocks dashboard control, else None."""
        feedback = self.robot.feedback()
        mode = feedback.get("control_mode")
        if mode in REMOTE_MODES and self.clock() - feedback.get("control_mode_time", -1e9) < 5:
            return MODE_NAMES.get(mode, str(mode))
        return None

    def laser_match(self):
        """(fraction, points) of the fresh laser scan on the loaded map, else None."""
        if not self.grid or self.grid.get("preview") or not hasattr(self.robot, "overlay"):
            return None
        overlay = self.robot.overlay()
        points = overlay.get("scan") or []
        if (overlay.get("frame") != self.grid["frame"] or len(points) < self.MATCH_POINTS
                or self.clock() - overlay.get("scan_time", -1e9) > self.FRESH):
            return None
        return scan_match(self.grid, points), len(points)

    def fit_pose(self, around):
        """Suggest a pose where the live laser best fits the map. Computes only:
        nothing is sent to the robot, the operator checks and sends it."""
        with self.lock:
            grid = self.grid
            if not grid or grid.get("preview") or not self.map_id:
                raise ValueError("Load the robot's map first")
            pose = self.robot.feedback().get("pose")
            overlay = self.robot.overlay()
            if not pose or overlay.get("frame") != grid["frame"]:
                raise ValueError("No estimated pose in the map frame")
            points = overlay.get("scan") or []
            if len(points) < self.MATCH_POINTS or self.clock() - overlay.get("scan_time", -1e9) > self.FRESH:
                raise ValueError("No fresh laser scan to fit")
            if self.robot.feedback()["speed"] > 0.02:
                raise ValueError("Wait until the robot is standing still")
            map_id = self.map_id
        # Heavy work runs outside the lock so the 10 Hz watchdog keeps going.
        masks = self.masks
        if not masks or masks["map_id"] != map_id:
            masks = {"map_id": map_id, "coarse": wall_mask(grid, 0.30), "fine": wall_mask(grid, 0.15)}
            self.masks = masks
        c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
        local = [(c * (x - pose["x"]) + s * (y - pose["y"]), -s * (x - pose["x"]) + c * (y - pose["y"]))
                 for x, y in points]
        # Rooms have look-alike spots that fit almost as well, so only search
        # near the operator's own click, never the whole map.
        center = (number(around["x"]), number(around["y"]), number(around["yaw"]))
        candidates = fit_pose(grid, masks, local, center, 1.5, math.radians(60))
        match, x, y, yaw = candidates[0]
        rival = next((c for c in candidates[1:] if c[0] > match - 0.05 and (
            math.hypot(c[1] - x, c[2] - y) > 0.5
            or abs(math.atan2(math.sin(c[3] - yaw), math.cos(c[3] - yaw))) > math.radians(20))), None)
        before = scan_match(grid, points)
        self.log(f"Auto-fit suggests ({x:.2f}, {y:.2f}, {yaw:.2f}): {match:.0%} of laser on walls "
                 f"(estimate {before:.0%}). Not sent.")
        return {"x": x, "y": y, "yaw": yaw, "match": match, "current": before,
                "rival": {"x": rival[1], "y": rival[2], "yaw": rival[3], "match": rival[0]} if rival else None}

    def _quality_error(self, pose, minimum):
        match = self.laser_match()
        if match is not None:
            if match[0] >= minimum:
                return None
            return (f"Only {match[0]:.0%} of the laser points lie on map walls (needs {minimum:.0%}). "
                    "The green pose is probably wrong: send the initial pose again where the robot "
                    "really stands, then turn ↶ ↷ about 30° each way.")
        if any(not math.isfinite(v) or v < 0 or v > limit for v, limit in
               zip(pose["variance"], (0.25, 0.25, 0.3))):
            v = pose["variance"]
            return (f"No laser overlay to check against, and uncertainty is too high ({v[0]:.3f} m², "
                    f"{v[2]:.3f} rad²; limits 0.25 / 0.30). Enable motion and turn in place a little "
                    "so the robot refines its estimate, then confirm again.")
        return None

    def autopilot_status(self):
        """Fresh /autopilot system status dict, else None."""
        feedback = self.robot.feedback()
        if self.clock() - feedback.get("autopilot_time", -1e9) < 5:
            return feedback.get("autopilot")
        return None

    def _feedback(self, localization=False, quality=True):
        feedback = self.robot.feedback()
        now = self.clock()
        if now - feedback["odom_time"] > self.FRESH:
            raise ValueError("Robot odometry is missing or stale")
        if now - feedback["brake_time"] > self.FRESH or feedback["brake"]:
            raise ValueError("Emergency-brake feedback is missing, stale or active")
        if not math.isfinite(feedback["speed"]):
            raise ValueError("Robot speed is invalid")
        if localization:
            pose = feedback.get("pose")
            if self.grid and self.grid.get("preview"):
                raise ValueError("Preview map cannot be used for real navigation")
            if not self.localized or not pose:
                raise ValueError("Load a map, seed pose and confirm localization first")
            if now - feedback["pose_time"] > self.FRESH:
                raise ValueError("Localization pose is stale")
            if pose["frame"] != self.robot.frame or pose["frame"] != self.grid["frame"]:
                raise ValueError("Map and estimated pose frames do not match NAV_FRAME")
            if not all(math.isfinite(pose[k]) for k in ("x", "y", "yaw")):
                raise ValueError("Localization pose is invalid")
            status = self.autopilot_status() if quality else None
            # Until the autopilot reports "accurate", goals are queued onboard
            # and start later on their own, so never send one before that.
            if status and status.get("accuracy") != "accurate":
                raise ValueError(f"The robot's autopilot still reports {status.get('accuracy')} "
                                 f"({status.get('behavior') or 'no detail'}). Send the initial pose "
                                 "again; it should switch to accurate within a few seconds.")
            problem = quality and self._quality_error(pose, self.MATCH_MIN)
            if problem:
                raise ValueError(problem)
        return feedback

    def _owned(self, owner):
        if not self.enabled or owner != self.owner:
            raise ValueError("Enable motion in this browser first")
        if self.clock() - self.heartbeat > self.LEASE:
            raise ValueError("Operator heartbeat expired; stop and enable again")

    def ping(self, owner):
        with self.lock:
            if owner == self.owner and self.enabled:
                self.heartbeat = self.clock()

    def enable(self, owner, confirmed):
        with self.lock:
            if confirmed is not True:
                raise ValueError("Confirm the safety checklist first")
            if self.enabled:
                raise ValueError("Already enabled; stop before changing operator")
            self._feedback()
            blocked = self.remote_mode()
            if blocked:
                raise ValueError(f"Robot is in {blocked} mode, so it refuses dashboard control. "
                                 "Hand control back on the remote first, then enable again.")
            # Clear autonomous work BEFORE taking USER control. Never blindly
            # hand control back to a previously queued onboard behavior.
            try:
                note = self.robot.halt_autopilot()
                if note:
                    self.log(note)
                self.robot.take_control()
            except Exception:
                self.stop("Enable failed or timed out")
                raise
            self.holding = True
            self.robot.publish_velocity(0.0, 0.0)
            self.enabled, self.owner = True, owner
            self.heartbeat = self.clock()
            self.stop_verified = True
            self.mode = "ready"
            self.log("Motion enabled by operator; USER zero hold.")

    def stop(self, reason="Operator stop"):
        """Latch disabled, clear behavior and HOLD zero USER, never auto-resume."""
        with self.lock:
            self.enabled, self.owner = False, None
            self.velocity, self.drive_until = (0.0, 0.0), -1e9
            if self.goal:
                g = self.goal
                self.last_run = {"x": g["x"], "y": g["y"], "driven": g["driven"],
                                 "remaining": g["remaining"], "ended": reason,
                                 "seconds": round(self.clock() - g["started"], 1)}
            self.goal = None
            self.mode = "stopped"
            self.stop_verified = False
            errors = []
            # Zero immediately if we own USER, including while stop services wait.
            self.robot.publish_velocity(0.0, 0.0)
            try:
                self.robot.take_control()
                self.holding = True
                self.robot.publish_velocity(0.0, 0.0)
            except Exception as exc:
                errors.append(str(exc))
            try:
                note = self.robot.halt_autopilot()
                self.stop_verified = True
                if note:
                    self.log(note)
            except Exception as exc:
                errors.append(str(exc))
            if errors:
                self.mode = "STOP UNCONFIRMED"
                self.stop_verified = False
                blocked = self.remote_mode()
                if blocked:
                    errors.append(f"robot is in {blocked} mode: whoever holds the remote has control")
            self.log(reason + (": " + "; ".join(errors) if errors else "; zero hold"))
            return not errors

    def release(self):
        with self.lock:
            if self.enabled or not self.holding or not self.stop_verified:
                raise ValueError("Stop successfully before releasing control")
            self.robot.publish_velocity(0.0, 0.0)
            self.robot.release_control()
            self.holding = False
            self.mode = "disabled"
            self.log("USER control reset to NONE. Check the robot before leaving.")

    def drive(self, owner, linear, angular):
        linear, angular = number(linear), number(angular)
        if abs(linear) > self.max_linear or abs(angular) > self.max_angular:
            raise ValueError(f"Manual limits: {self.max_linear:.2f} m/s and {self.max_angular:.2f} rad/s")
        with self.lock:
            self._owned(owner)
            self._feedback()
            if self.goal:
                raise ValueError("Stop navigation before manual driving")
            if not self.holding:
                self.robot.take_control()
            self.holding = True
            self.velocity = linear, angular
            self.drive_until = self.clock() + self.DEADMAN
            self.mode = "manual" if linear or angular else "ready"

    def list_maps(self):
        maps = self.robot.list_maps()
        try:
            active = self.robot.active_map_id()
        except Exception:
            active = None
        return {"maps": maps, "active": active, "sim_map": self.sim_info}

    def load_sim_map(self):
        """Load the simulation stack's map: onboard when the robot stores the
        same map ID, otherwise as a display-only preview."""
        with self.lock:
            if self.enabled:
                raise ValueError("Stop before changing maps")
            if self.sim_source is None or self.sim_info is None:
                raise ValueError("No simulation map configured (SIM_MAP)")
            wanted = self.sim_info["id"]
            for m in self.robot.list_maps():
                if str(m["id"]).strip("{}").lower() == wanted:
                    self.log("Simulation map " + self.sim_info["name"] + " is stored on the robot")
                    return self.load_map(m["id"])
            grid = self.sim_source.grid(self.robot.frame)
            grid["preview"] = True
            self.localized, self.map_id, self.seed_time = False, None, -1e9
            self.grid = grid
            self.log("Simulation map " + self.sim_info["name"] + " is NOT stored on the robot: "
                     "preview only. Upload it with Cerebra Studio to localize and navigate on it.")
            return grid

    def load_map(self, map_id):
        with self.lock:
            if self.enabled:
                raise ValueError("Stop before changing maps")
            self.localized = False
            self.grid, self.map_id = None, None
            if not self.stop("Preparing map change"):
                raise ValueError("Cannot change map: stop is unconfirmed")
            grid = self.robot.load_map(str(map_id))
            if grid["frame"] != self.robot.frame:
                raise ValueError("Map frame differs from NAV_FRAME; check configuration")
            self.grid, self.map_id = grid, str(map_id)
            self.masks = None
            self.seed_time, self.seed_failed, self.map_time = -1e9, False, self.clock()
            self.seed_pose = None
            self.log(("Map already active onboard; kept the robot's current localization: "
                      if grid.get("already_active") else "Loaded onboard localization map ") + str(map_id))
            return grid

    def seed(self, x, y, yaw):
        x, y, yaw = number(x), number(y), number(yaw)
        with self.lock:
            if self.grid and self.grid.get("preview"):
                raise ValueError("This preview map is not stored on the robot; it cannot localize on it")
            if not self.grid:
                raise ValueError("Load the robot's map before setting initial pose")
            # Seeding is safe in USER zero hold (enabled or stopped) but not
            # while the autopilot or a manual drive is moving the base.
            if not self.holding or not self.stop_verified:
                raise ValueError("Enable motion (or press STOP) first so the robot is held at zero speed")
            if self.goal or self.mode == "manual":
                raise ValueError("Release the drive buttons / stop navigation before setting initial pose")
            if self.robot.feedback()["speed"] > 0.02:
                raise ValueError("Wait until the robot is standing still")
            validate_point(self.grid, x, y)
            self.localized = False
            self.seed_time = -1e9
            sent_at = self.clock()
            self.seed_failed = True
            status = self.autopilot_status()
            # The onboard start-up step only finishes below 0.01 uncertainty;
            # once accurate, a looser pose lets AMCL search around the click.
            tight = not status or status.get("accuracy") != "accurate"
            self.robot.seed_pose(x, y, yaw, tight=tight)
            self.seed_time, self.seed_failed = sent_at, False
            self.seed_pose = {"x": x, "y": y, "yaw": yaw}
            self.log(f"Initial pose ({x:.2f}, {y:.2f}, {yaw:.2f}) sent ({'tight' if tight else 'loose'}), "
                     "NOT proof of successful localization.")

    def confirm_localization(self, confirmed):
        with self.lock:
            if not self.grid or self.grid.get("preview"):
                raise ValueError("Load the robot's own map first")
            if confirmed is not True:
                raise ValueError('Tick "Pose, heading and laser match the real surroundings" first')
            if self.seed_failed:
                raise ValueError("The last initial pose was not delivered; send it again")
            # AMCL only refines after the base moves, so the operator turns in
            # place with manual drive and confirms while still enabled.
            if self.goal or self.mode == "manual":
                raise ValueError("Release the drive buttons / stop navigation before confirming")
            feedback = self.robot.feedback()
            # A fresh initial pose is optional: when the robot already knows where it
            # is, the operator's visual check plus the uncertainty gate suffice.
            if feedback["pose_time"] <= max(self.seed_time, self.map_time):
                raise ValueError("No new estimated pose from the robot since the map load / initial pose")
            self.localized = True
            try:
                self._feedback(localization=True)
            except Exception:
                self.localized = False
                raise
            self.log("Operator confirmed localization against the real surroundings.")

    def navigate(self, owner, x, y, yaw):
        x, y, yaw = number(x), number(y), number(yaw)
        with self.lock:
            self._owned(owner)
            self._feedback(localization=True)
            if self.goal or self.mode == "manual":
                raise ValueError("Stop the current movement first")
            validate_point(self.grid, x, y)
            pose = self.robot.feedback()["pose"]
            room = clearance(self.grid, pose["x"], pose["y"])
            if room < self.START_CLEARANCE:
                hint = drive_hint(open_spot(self.grid, pose["x"], pose["y"], pose["yaw"]))
                raise ValueError(f"The robot stands {room:.2f} m from a wall on the map, so the onboard "
                                 f"planner may refuse (\"Starting point in lethal space\"). Check the "
                                 f"localization and surroundings: {hint}. This is a map-only hint, "
                                 "not a safe route. A ghost wall left from mapping can also cause this.")
            # Switch from our USER zero hold to the onboard autopilot.
            # Any failure, including an ambiguous service timeout, latches stop.
            pose = self.robot.feedback()["pose"]
            self.goal = {"x": x, "y": y, "yaw": yaw, "started": self.clock(),
                         "arrival_since": None, "driven": 0.0, "last": [pose["x"], pose["y"]],
                         "remaining": math.hypot(x - pose["x"], y - pose["y"])}
            try:
                self.robot.publish_velocity(0.0, 0.0)
                self.robot.release_control()
                self.holding = False
                note = self.robot.navigate(x, y, yaw)
                if note:
                    self.log("Autopilot replied: " + str(note))
            except Exception:
                self.stop("Navigation request failed or timed out")
                raise
            self.mode = "navigating"
            self.bad_since = None
            self.log(f"Autopilot accepted goal ({x:.2f}, {y:.2f}, {yaw:.2f}).")

    def _report_robot(self, now):
        """Copy autopilot status changes and robot warnings into the event log."""
        feedback = self.robot.feedback()
        behavior = feedback.get("bt_status") if now - feedback.get("bt_status_time", -1e9) < 5 else None
        status = self.autopilot_status()
        behavior = behavior or (status or {}).get("behavior")
        if behavior and behavior != self.last_behavior:
            self.last_behavior = behavior
            self.log("Autopilot: " + behavior)
        for entry in (self.robot.drain_logs() if hasattr(self.robot, "drain_logs") else []):
            line = f"Robot {entry['level']} [{entry['name']}]: {entry['msg']}"
            # The planner repeats the same complaint many times per second.
            if line == self.last_robot_log[0] and now - self.last_robot_log[1] < 10:
                continue
            self.last_robot_log = (line, now)
            self.log(line)
        # Failure may be followed by "waiting" before our next watchdog tick.
        # The adapter latches its timestamp so that transition is not lost.
        failed_at = feedback.get("bt_failure_time", -1e9)
        for text, stamp in ((feedback.get("bt_status"), feedback.get("bt_status_time", -1e9)),
                            ((status or {}).get("behavior"), feedback.get("autopilot_time", -1e9))):
            if text and "did not complete" in text.lower():
                failed_at = max(failed_at, stamp)
        if self.goal and failed_at > self.goal["started"]:
            reason = self.last_robot_log[0] if self.last_robot_log[1] >= self.goal["started"] else None
            self.stop("Autopilot gave up on the goal. Last robot warning: "
                      + (reason or "no robot warning received"))
        if self.goal and not self.goal.get("stall_warned") and now - self.goal["started"] > 20 \
                and self.goal["driven"] < 0.05:
            self.goal["stall_warned"] = True
            self.log("WARNING: goal accepted 20 s ago but the robot has not moved. Autopilot says: "
                     + (behavior or "nothing") + ". Check the robot warnings above; STOP if unsure.")

    def tick(self):
        """10 Hz watchdog; browser requests cannot extend a drive implicitly."""
        with self.lock:
            now = self.clock()
            try:
                self._report_robot(now)
            except Exception as exc:
                self.log("Status report error: " + str(exc))
            if self.enabled:
                try:
                    if now - self.heartbeat > self.LEASE:
                        raise ValueError("Operator heartbeat lost")
                    feedback = self._feedback(localization=bool(self.goal), quality=False)
                    if self.goal:
                        goal, pose = self.goal, feedback["pose"]
                        # Laser dropouts in a crowded corridor are normal; only
                        # a sustained mismatch means the robot is lost.
                        problem = self._quality_error(pose, self.NAV_MATCH_MIN)
                        if not problem:
                            self.bad_since = None
                        elif self.bad_since is None:
                            self.bad_since = now
                        elif now - self.bad_since > self.NAV_BAD_GRACE:
                            raise ValueError("Localization lost during navigation: " + problem)
                        goal["driven"] += math.hypot(pose["x"] - goal["last"][0],
                                                     pose["y"] - goal["last"][1])
                        goal["last"] = [pose["x"], pose["y"]]
                        goal["remaining"] = math.hypot(goal["x"] - pose["x"], goal["y"] - pose["y"])
                        if now - goal["started"] > self.NAV_TIMEOUT:
                            raise ValueError("Navigation deadline exceeded")
                        angle = math.atan2(math.sin(pose["yaw"] - goal["yaw"]),
                                           math.cos(pose["yaw"] - goal["yaw"]))
                        arrived = (math.hypot(pose["x"] - goal["x"], pose["y"] - goal["y"]) < 0.25
                                   and abs(angle) < 0.2 and feedback["speed"] < 0.03)
                        if not arrived:
                            goal["arrival_since"] = None
                        elif goal["arrival_since"] is None:
                            goal["arrival_since"] = now
                        elif now - goal["arrival_since"] >= 1.0:
                            self.stop("Arrival observed (pose + heading + stationary), not an action result")
                    elif now >= self.drive_until:
                        self.velocity = (0.0, 0.0)
                        self.mode = "ready"
                except Exception as exc:
                    self.stop(str(exc))
            if self.holding:
                self.robot.publish_velocity(*(self.velocity if self.enabled else (0.0, 0.0)))

    def snapshot(self):
        with self.lock:
            feedback = self.robot.feedback()
            mode = feedback.get("control_mode")
            return {"enabled": self.enabled, "mode": self.mode, "holding_user": self.holding,
                    "stop_verified": self.stop_verified, "localized": self.localized,
                    "map_id": self.map_id, "seeded": self.seed_time > 0,
                    "seed_pose": self.seed_pose, "seed_time": self.seed_time,
                    "max_linear": self.max_linear, "max_angular": self.max_angular,
                    "laser_match": self.laser_match(), "match_min": self.MATCH_MIN,
                    "autopilot": self.autopilot_status(),
                    "behavior": self.last_behavior,
                    "clearance": self._clearance(feedback),
                    "goal": self.goal, "last_run": self.last_run,
                    "sim_map": self.sim_info, "default_map": self.default_map,
                    "preview": bool(self.grid and self.grid.get("preview")),
                    "control_mode_name": MODE_NAMES.get(mode, "unknown" if mode is None else str(mode)),
                    "remote_mode": self.remote_mode(),
                    "feedback": feedback, "now": self.clock(),
                    "events": list(self.events), "frame": self.robot.frame}

    def _clearance(self, feedback):
        pose = feedback.get("pose")
        if not self.grid or self.grid.get("preview") or not pose or pose.get("frame") != self.grid["frame"]:
            return None
        room = clearance(self.grid, pose["x"], pose["y"])
        result = {"metres": room, "ok": room >= self.START_CLEARANCE}
        if not result["ok"]:
            key = (id(self.grid), round(pose["x"], 1), round(pose["y"], 1), round(pose["yaw"], 1))
            if getattr(self, "_hint_key", None) != key:
                self._hint_key = key
                self._hint = drive_hint(open_spot(self.grid, pose["x"], pose["y"], pose["yaw"]))
            result["hint"] = self._hint
        return result

    def list_places(self):
        with self.lock:
            if self.places is None or not self.map_id:
                return []
            return self.places.list(self.map_id)

    def save_place(self, name, x, y, yaw):
        x, y, yaw = number(x), number(y), number(yaw)
        with self.lock:
            if self.places is None or not self.map_id or not self.grid:
                raise ValueError("Load an onboard map before saving places")
            validate_point(self.grid, x, y)
            result = self.places.save(self.map_id, name, x, y, yaw)
            self.log(f'Saved place "{str(name).strip()}" ({x:.2f}, {y:.2f}, {yaw:.2f}).')
            return result

    def delete_place(self, name):
        with self.lock:
            if self.places is None or not self.map_id:
                raise ValueError("Load an onboard map first")
            self.log(f'Deleted place "{name}".')
            return self.places.delete(self.map_id, str(name))

    def rename_place(self, old, new):
        with self.lock:
            if self.places is None or not self.map_id:
                raise ValueError("Load an onboard map first")
            result = self.places.rename(self.map_id, str(old), new)
            self.log(f'Renamed place "{old}" to "{str(new).strip()}".')
            return result
