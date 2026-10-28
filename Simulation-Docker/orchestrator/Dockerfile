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

# -----------------------------------------------------------------------------
# Avular Origin One message definitions.
#
# The real robot only drives when its control mode is set to USER
# (origin_msgs/srv/SetControlMode on /robot/cmd_vel_controller/set_control_mode)
# and it navigates with autopilot behaviors
# (knowledge_base_msgs/srv/SetAndExecuteBehaviors). Those messages come from the
# Avular example workspace, pinned here to the revision the real-robot commands
# were written against - the definitions are the robot's own, so the messages
# match the ones it expects.
# -----------------------------------------------------------------------------
ARG AVULAR_EXAMPLE_COMMIT=bc9b47c498a47f6fc97c87e4f34148f4bca9ae64
# bash, not the default /bin/sh: ROS's setup.bash is a bash script
SHELL ["/bin/bash", "-lc"]
RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/* \
    && git clone --quiet https://github.com/avular-robotics/example_origin_ros.git \
        /tmp/avular_example \
    && git -C /tmp/avular_example checkout --quiet "$AVULAR_EXAMPLE_COMMIT" \
    && mkdir -p /opt/avular_msgs/src \
    && cp -r /tmp/avular_example/origin_msgs \
             /tmp/avular_example/autonomy_msgs \
             /tmp/avular_example/knowledge_base_msgs \
             /tmp/avular_example/complete_coverage_planner_msgs \
             /opt/avular_msgs/src/ \
    && rm -rf /tmp/avular_example \
    && cd /opt/avular_msgs \
    && . /opt/ros/humble/setup.bash \
    && colcon build --packages-select origin_msgs autonomy_msgs \
           knowledge_base_msgs complete_coverage_planner_msgs \
    && rm -rf /opt/avular_msgs/build /opt/avular_msgs/log
ENV AVULAR_MSGS_SETUP=/opt/avular_msgs/install/setup.bash

# Keep the versions identical to the host tools for the same reason as the
# smoke test in the README: xarm-python-sdk is the official UFACTORY SDK,
# Flask only serves the local dashboard.
RUN python3 -m pip install --no-cache-dir \
        xarm-python-sdk \
        flask \
        pyyaml

WORKDIR /app
COPY mission_model.py engine.py ros_robot.py arm_control.py server.py \
     targets.py commands.py runner.py /app/

EXPOSE 8080

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /opt/avular_msgs/install/setup.bash && exec python3 /app/server.py"]
