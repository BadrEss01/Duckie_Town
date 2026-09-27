FROM ros:noetic-ros-base-focal
LABEL maintainer="Badr Essefiany"
RUN apt-get update && apt-get install -y --no-install-recommends python3-opencv python3-numpy ros-noetic-cv-bridge ros-noetic-rostest && rm -rf /var/lib/apt/lists/*
WORKDIR /catkin_ws
COPY packages/duckie_lane_following src/duckie_lane_following
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && catkin_make'
COPY launchers/default.sh /lane-launch.sh
CMD ["bash", "/lane-launch.sh"]
