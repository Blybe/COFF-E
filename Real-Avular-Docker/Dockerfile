FROM ros:humble-ros-base
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
ARG AVULAR_EXAMPLE_COMMIT=bc9b47c498a47f6fc97c87e4f34148f4bca9ae64
RUN apt-get update && apt-get install -y --no-install-recommends \
    git python3-colcon-common-extensions ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && git clone --quiet https://github.com/avular-robotics/example_origin_ros.git /tmp/avular \
    && git -C /tmp/avular checkout --quiet "$AVULAR_EXAMPLE_COMMIT" \
    && mkdir -p /opt/avular_msgs/src \
    && cp -r /tmp/avular/{origin_msgs,autonomy_msgs,knowledge_base_msgs,complete_coverage_planner_msgs} /opt/avular_msgs/src/ \
    && source /opt/ros/humble/setup.bash \
    && cd /opt/avular_msgs \
    && colcon build --packages-select origin_msgs autonomy_msgs knowledge_base_msgs complete_coverage_planner_msgs \
    && rm -rf /tmp/avular /opt/avular_msgs/build /opt/avular_msgs/log
WORKDIR /app
COPY app /app
EXPOSE 8080
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /opt/avular_msgs/install/setup.bash && exec python3 server.py"]
