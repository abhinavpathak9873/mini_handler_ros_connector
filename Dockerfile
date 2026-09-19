ARG ROS_BASE=ros:humble-ros-core
FROM ${ROS_BASE} AS runtime-base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rclpy ros-humble-std-msgs ros-humble-std-srvs \
    ros-humble-sensor-msgs ros-humble-rosidl-runtime-py \
    ros-humble-rmw-cyclonedds-cpp python3-serial \
    && rm -rf /var/lib/apt/lists/*

FROM runtime-base AS build
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential ros-humble-ament-cmake ros-humble-ament-cmake-python \
    ros-humble-rosidl-default-generators python3-colcon-common-extensions python3-pytest \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws/src/mini_handler_ros_connector
COPY . .
RUN python3 -m pytest -q tests/test_core.py tests/test_protocol.py
WORKDIR /ws
RUN . /opt/ros/humble/setup.sh && colcon build --merge-install --install-base /opt/mini_handler \
    --cmake-args -DBUILD_TESTING=OFF
RUN . /opt/ros/humble/setup.sh && . /opt/mini_handler/setup.sh && \
    python3 /ws/src/mini_handler_ros_connector/tests/ros_integration.py

FROM runtime-base AS runtime
COPY --from=build /opt/mini_handler /opt/mini_handler
COPY docker-entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /opt/mini_handler/bin/mini-handler \
    /opt/mini_handler/lib/mini_handler_ros_connector/connector \
    /opt/mini_handler/lib/mini_handler_ros_connector/mini-handler
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 PYTHONUNBUFFERED=1
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/opt/mini_handler/lib/mini_handler_ros_connector/connector", "--ros-args", "--params-file", "/opt/mini_handler/share/mini_handler_ros_connector/config/gripper.yaml"]
