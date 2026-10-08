# =============================================================================
# COFF-E - Avular Origin One simulation image
# =============================================================================
# ROS 2 Humble + Gazebo Fortress image that builds and runs the official
# Avular Origin simulation:
#   - https://github.com/avular-robotics/avular_origin_simulation
#   - https://github.com/avular-robotics/avular_origin_description
#
# The Gazebo GUI runs on an internal virtual display and is streamed to the
# browser (noVNC, port 6080), so it works on Windows without an X server.
# Alternatively set USE_HOST_DISPLAY=1 with a reachable DISPLAY (WSLg/Linux)
# to open the GUI natively on the host, or HEADLESS=true for no GUI at all.
# =============================================================================

FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/home/origin_ws

# -----------------------------------------------------------------------------
# Gazebo Fortress apt repository (packages.osrfoundation.org), required by
# the ros_gz packages on ROS 2 Humble / Ubuntu 22.04.
# -----------------------------------------------------------------------------
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates curl \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Base tooling: colcon, rosdep, git-lfs, the virtual display / VNC / noVNC
# stack that streams the Gazebo GUI to a browser, and helper tools.
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        git-lfs \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-humble-teleop-twist-keyboard \
        ros-humble-rviz2 \
        # Gazebo GUI streaming (virtual display -> VNC -> browser)
        xvfb \
        x11vnc \
        novnc \
        websockify \
        fluxbox \
        # Software OpenGL rendering (llvmpipe) + X11 debug/screenshot tools
        libgl1-mesa-dri \
        mesa-utils \
        x11-apps \
        x11-utils \
        x11-xserver-utils \
        imagemagick \
    && ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Avular Origin simulation sources.
# The meshes of avular_origin_description are stored in Git LFS, so the LFS
# filters are installed before cloning.
#
# avular_origin_description is PINNED to commit 051deab (Feb 2026, the last
# revision that still contains the Gazebo plugins - DiffDrive, GPU lidar,
# RealSense camera, NavSat - inside the URDF). The current main branch
# (v2.0.0, Jul 2026) removed them ("gazebo parameters will be moved to
# avular_simulations repo"), which leaves the simulated robot without
# sensors or drive when combined with avular_origin_simulation branch
# origin_1.0/1 used below.
# -----------------------------------------------------------------------------
RUN git lfs install --skip-repo \
    && git clone --branch origin_1.0/1 \
        https://github.com/avular-robotics/avular_origin_simulation.git \
        "$ROS_WS/src/avular_origin_simulation" \
    && git clone \
        https://github.com/avular-robotics/avular_origin_description.git \
        "$ROS_WS/src/avular_origin_description" \
    && git -C "$ROS_WS/src/avular_origin_description" checkout \
        051deab9cc7e53c14637d03fce504bce76bcf0c8

# The pinned origin_one_description uses Avular's internal cmake_avular build
# helpers (parse_manifest, create_debian_package, ...). Replace its
# CMakeLists.txt with a plain ament_cmake build that installs exactly the
# same directories (launch, rviz, urdf + the python package).
RUN printf '%s\n' \
    'cmake_minimum_required(VERSION 3.16)' \
    'project(origin_one_description)' \
    '' \
    'find_package(ament_cmake REQUIRED)' \
    'find_package(ament_cmake_python REQUIRED)' \
    '' \
    'ament_python_install_package(${PROJECT_NAME} SKIP_COMPILE)' \
    '' \
    'include(GNUInstallDirs)' \
    'install(DIRECTORY launch DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})' \
    'install(DIRECTORY rviz DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})' \
    'install(DIRECTORY urdf DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})' \
    '' \
    'ament_package()' \
    > "$ROS_WS/src/avular_origin_description/origin_one_description/CMakeLists.txt"

# cmd_vel_controller: closed-source Avular deb packages that make the
# simulation prioritise control inputs the same way as the real Origin One.
RUN apt-get update && apt-get install -y \
        "$ROS_WS/src/avular_origin_simulation/cmd_vel_controller/loggercxx_3.0.0_amd64.deb" \
        "$ROS_WS/src/avular_origin_simulation/cmd_vel_controller/rclcpp-avular_3.0.0_amd64.deb" \
        "$ROS_WS/src/avular_origin_simulation/cmd_vel_controller/origin-msgs_1.0.0_amd64.deb" \
        "$ROS_WS/src/avular_origin_simulation/cmd_vel_controller/cmd-vel-controller_1.7.1_amd64.deb" \
    && rm -rf /var/lib/apt/lists/*

# Remaining ROS dependencies (ros_gz_sim, ros_gz_bridge, xacro,
# robot_state_publisher, tf2_ros, ...). The skipped keys are Avular-internal
# packages (installed from the debs above, or test/build tools only used
# inside Avular) that are not in the public rosdistro index.
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update \
    && apt-get update \
    && rosdep install --from-paths "$ROS_WS/src" --ignore-src -y \
        --skip-keys "cmd_vel_controller cmake_avular ament_copyright_avular" \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Build the ROS workspace
# -----------------------------------------------------------------------------
RUN . /opt/ros/humble/setup.sh \
    && cd "$ROS_WS" \
    && colcon build --symlink-install

# `docker exec -it ... bash` skips the entrypoint, so source ROS in .bashrc.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source $ROS_WS/install/setup.bash" >> /root/.bashrc

# Navigation stack for the origin-nav service: Nav2 (map_server, AMCL,
# planners, controllers, RViz panel) and a 3D-lidar -> 2D-scan converter.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-pointcloud-to-laserscan \
        ros-humble-topic-tools \
    && rm -rf /var/lib/apt/lists/*

# Render the robot's sensors with Ogre 1 instead of Ogre 2. Without a GPU
# (software OpenGL / llvmpipe) Ogre 2 renders the Ouster gpu_lidar so slowly
# that the simulation drops to ~7% real time as soon as anything subscribes
# to /robot/lidar/points; with Ogre 1 it runs at ~50%. The xacro is used via
# the --symlink-install build, so no rebuild is needed.
RUN sed -i 's#<render_engine>ogre2</render_engine>#<render_engine>ogre</render_engine>#' \
        "$ROS_WS/src/avular_origin_description/origin_one_description/urdf/origin_one.urdf.xacro" \
    && grep -q '<render_engine>ogre</render_engine>' \
        "$ROS_WS/src/avular_origin_description/origin_one_description/urdf/origin_one.urdf.xacro"

# -----------------------------------------------------------------------------
# Runtime
# -----------------------------------------------------------------------------
COPY entrypoint.sh /entrypoint.sh
# Guard against Windows CRLF line endings in the copied script
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh

# `docker exec -it origin-nav goto Home`: Nav2 helper from the mounted nav/ folder
RUN printf '%s\n' '#!/bin/bash' 'source /opt/ros/humble/setup.bash' \
        'exec python3 /nav/goto.py "$@"' > /usr/local/bin/goto \
    && chmod +x /usr/local/bin/goto

EXPOSE 6080
WORKDIR "$ROS_WS"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "origin_one_gazebo", "ty_test_area.launch.py", "use_cmd_vel_controller:=True"]
