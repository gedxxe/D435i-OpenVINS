ARG KALIBR_IMAGE
FROM ${KALIBR_IMAGE}

ARG KALIBR_COMMIT
ARG ALLAN_COMMIT
LABEL org.opencontainers.image.title="OVRS isolated calibration tools" \
      org.opencontainers.image.description="Pinned Kalibr and allan_variance_ros workspace" \
      org.opencontainers.image.revision.kalibr="${KALIBR_COMMIT}" \
      org.opencontainers.image.revision.allan_variance_ros="${ALLAN_COMMIT}"

USER root
COPY . /catkin_ws/src/allan_variance_ros

SHELL ["/bin/bash", "-lc"]
RUN source /opt/ros/noetic/setup.bash \
    && cd /catkin_ws \
    && catkin build allan_variance_ros
