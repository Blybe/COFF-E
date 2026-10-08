#!/bin/bash
# Smoke test for the simulated Origin One: drives for 5 s via /robot/cmd_vel
# and checks that the odometry position changed.
#
# Usage (from the Simulation-Docker folder):
#   docker exec -i avular-origin-sim bash < tests/origin_drive_test.sh

source /opt/ros/humble/setup.bash
source /home/origin_ws/install/setup.bash

read_xy() {
    timeout 15 ros2 topic echo /robot/odom --once 2>/dev/null \
        | grep -A 2 "position:" | awk '/x:|y:/ {print $2}' | tr '\n' ' '
}

before=$(read_xy)
echo "odom before (x y): $before"

timeout 5 ros2 topic pub -r 10 /robot/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.5}, angular: {z: 0.0}}' > /dev/null 2>&1
sleep 1

after=$(read_xy)
echo "odom after  (x y): $after"

python3 - "$before" "$after" <<'EOF'
import math, sys
try:
    b = [float(v) for v in sys.argv[1].split()]
    a = [float(v) for v in sys.argv[2].split()]
except ValueError:
    print("FAIL: no odometry received"); sys.exit(1)
d = math.dist(b, a)
print(f"distance driven: {d:.2f} m")
if d > 0.5:
    print("PASS")
else:
    print("FAIL: robot did not move"); sys.exit(1)
EOF
