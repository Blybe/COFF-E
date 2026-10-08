"""Smoke test for the simulated xArm 6 in the ufactory-studio container.

Connects with the official xArm Python SDK, reads the state, and moves
joint 1 to 20 degrees and back at low speed.

Run from the host:      python tests/xarm_smoke_test.py 127.0.0.1
Run on compose network: python tests/xarm_smoke_test.py 192.168.100.200
"""

import socket
import sys
import time

from xarm.wrapper import XArmAPI

# The SDK treats anything that is not an IP address as a serial port name.
host = socket.gethostbyname(sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1")

arm = XArmAPI(host, baud_checkset=False, check_joint_limit=False)
time.sleep(1)
print("connected:", arm.connected, "| axis:", arm.axis, "| version:", arm.version)

arm.clean_error()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(0)
time.sleep(1)

print("angles before:", [round(a, 1) for a in arm.angles])
code = arm.set_servo_angle(angle=[20, 0, 0, 0, 0, 0], speed=30, wait=True)
print("move to J1=20 code:", code, "| angles:", [round(a, 1) for a in arm.angles])
code_back = arm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0], speed=30, wait=True)
print("move back code:", code_back, "| angles:", [round(a, 1) for a in arm.angles])

arm.disconnect()
sys.exit(0 if arm.axis == 6 and code == 0 and code_back == 0 else 1)
