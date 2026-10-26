# =============================================================================
# COFF-E - mission orchestrator + dashboard
# =============================================================================
# Small ROS 2 image that ties the two simulations together:
#
#   - ROS 2 (DDS) to the Origin One simulation container: publishes
#     /robot/cmd_vel, subscribes /robot/odom and /clock, and sends Nav2
#     goals to the origin-nav container
#   - the xArm Python SDK to the simulated xArm 6 (192.168.100.200),
#     exactly like the SDK talks to a real control box
#   - the mission engine: runs mission steps in order, CHECKS every step
#     (did the odometry move? is the joint angle reached?) and stops the
#     mission as soon as a step fails
#   - the web dashboard on http://localhost:8080
#
# Based on the plain ros:humble image (not on the big Gazebo image), so it
# builds in parallel with avular-origin-sim on a first `docker compose up`.
# =============================================================================

FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# nav2_msgs: the NavigateToPose action definition (the planner itself runs
# in the origin-nav container). python3-pip for the xArm SDK + Flask.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-nav2-msgs \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Keep the versions identical to the host tools for the same reason as the
# smoke test in the README: xarm-python-sdk is the official UFACTORY SDK,
# Flask only serves the local dashboard.
RUN python3 -m pip install --no-cache-dir \
        xarm-python-sdk \
        flask \
        pyyaml

WORKDIR /app
COPY mission_model.py engine.py ros_robot.py arm_control.py server.py /app/

EXPOSE 8080

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 /app/server.py"]
