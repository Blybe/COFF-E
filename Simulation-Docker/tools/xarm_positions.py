#!/usr/bin/env python3
"""COFF-E - confirm the named arm positions on a real xArm 6.

Run this in the Python IDE of UFACTORY Studio on the machine connected to the
real arm (or as a file: py -3 tools/xarm_positions.py). It is deliberately
one simple script with one function per position: each one moves the arm
there, waits for it to arrive and prints whether the joints really reached
it, with the same 3 deg tolerance the orchestrator checks with.

    py -3 tools/xarm_positions.py               confirm every position
    py -3 tools/xarm_positions.py serve_right   just one (or a few)

When the arm is not on this machine, point at its control box with
XARM_HOST (see HOST below).

After running the file once, the functions can also be called one at a time
from the IDE console, e.g. serve_right() then initial().

The positions mirror orchestrator/mission_model.py (ARM_POSES). "initial" is
the real arm's standard position - where it rests and drives, because the
gripper at its front makes the all-zero "home" position unsafe. That is why
there is no home() here. "transport" is the drive pose the *simulation*
uses; the others are the serving poses of the coffee workflow.

If a position fails (the controller stops the move, or the joints do not
reach it), the script says so and stops: clear the reported error in
UFACTORY Studio, move the arm somewhere safe and try that position again.

Safety: the arm MOVES. Clear the space around it, know where the emergency
stop is, and start at the slow default speed.
"""

import os
import sys
import time

HOST = os.environ.get("XARM_HOST", "127.0.0.1")
# 127.0.0.1 = the arm on this machine (UFACTORY Studio's IDE). When running
# the file from elsewhere, set XARM_HOST to the control box IP, e.g.:
#   XARM_HOST=192.168.100.50 py -3 tools/xarm_positions.py
SPEED = 20.0         # deg/s, slow on purpose - raise it once the
                     # positions are confirmed
TOLERANCE = 3.0      # deg, the tolerance the orchestrator checks with

POSITIONS = {
    #                J1    J2    J3   J4   J5   J6
    "initial":     [0.0, -90.0,  0.0, 0.0, 90.0, 0.0],  # rest / drive (the real arm)
    "serve_right": [-90.0, 20.0, -50.0, 0.0, 30.0, 0.0],
    "serve_left":  [90.0, 20.0, -50.0, 0.0, 30.0, 0.0],
    "reach_front": [0.0, 30.0, -40.0, 0.0, 10.0, 0.0],
    "look_down":   [0.0, 10.0, -90.0, 0.0, 80.0, 0.0],
    "wave_up":     [0.0, -20.0, -120.0, 0.0, 40.0, 0.0],
    "transport":   [0.0, -45.0, -30.0, 0.0, 75.0, 0.0],  # the simulation's drive pose
}

arm = None   # set by connect()


def connect():
    """The same init sequence the orchestrator uses (arm_control.py)."""
    global arm
    from xarm.wrapper import XArmAPI   # py -3 -m pip install xarm-python-sdk
    arm = XArmAPI(HOST, baud_checkset=False, check_joint_limit=False)
    deadline = time.time() + 8.0
    while not getattr(arm, "connected", False) and time.time() < deadline:
        time.sleep(0.2)
    if not getattr(arm, "connected", False):
        raise RuntimeError(f"cannot reach the xArm at {HOST}")
    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)    # 0 = position mode, like the Studio UI
    arm.set_state(0)   # 0 = ready
    print(f"connected to the xArm at {HOST}")
    show_angles("current")


def angles():
    # The SDK reports 7 values for an xArm 6 (6 joints + a gripper slot).
    return [float(a) for a in list(arm.angles)[:6]]


def show_angles(tag):
    print(f"{tag} angles {[round(a, 1) for a in angles()]}")


def move_to(name, speed=SPEED):
    """Move to a named position and confirm the joints reached it."""
    target = POSITIONS[name]
    print(f"\n[{name}]  target {[round(t) for t in target]}  at {speed:.0f} deg/s")
    code = arm.set_servo_angle(angle=[float(a) for a in target],
                               speed=float(speed), wait=True)
    if code != 0:
        print(f"    FAILED: the controller refused or stopped the move "
              f"(exit code {code})")
        print("           clear the error in UFACTORY Studio before the "
              "next move")
        show_angles("   ")
        return False
    reached = angles()
    gap = max(abs(a - t) for a, t in zip(reached, target))
    print(f"    reached {[round(a, 1) for a in reached]}")
    print(f"    largest joint difference {gap:.2f} deg -> "
          + ("PASS" if gap <= TOLERANCE else "FAIL"))
    return gap <= TOLERANCE


# One function per position, so they can be called one at a time from the
# IDE console after running this file once.
def initial():     return move_to("initial")
def serve_right(): return move_to("serve_right")
def serve_left():  return move_to("serve_left")
def reach_front(): return move_to("reach_front")
def look_down():   return move_to("look_down")
def wave_up():     return move_to("wave_up")
def transport():   return move_to("transport")


def confirm_all():
    """Every position, one at a time: from initial out to each serving pose
    and back, so every transition is one the coffee workflow makes. Stops at
    the first position that fails."""
    order = ["initial", "serve_right", "serve_left", "reach_front",
             "look_down", "wave_up", "transport", "initial"]
    print("\nthe arm will move through:", " -> ".join(order))
    print("clear the space around it - moving in 5 s (Ctrl+C to cancel)")
    time.sleep(5)
    passed = 0
    for name in order:
        if move_to(name):
            passed += 1
        else:
            print("\nstopping at the first failure: clear the reported error "
                  "in UFACTORY Studio, move the arm somewhere safe, then "
                  "run the remaining positions one at a time")
            break
    print(f"\n{passed}/{len(order)} positions confirmed")
    return passed == len(order)


def main():
    connect()
    # Position names as arguments run just those; anything else (or nothing)
    # runs the full confirmation above.
    wanted = [a for a in sys.argv[1:] if a in POSITIONS]
    if not wanted:
        return 0 if confirm_all() else 1
    for name in wanted:
        if not move_to(name):
            return 1
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\ninterrupted - stopping the arm")
        try:
            arm.stop()
        except Exception:
            pass
        raise SystemExit(130)
