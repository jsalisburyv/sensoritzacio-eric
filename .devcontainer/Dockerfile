FROM osrf/ros:jazzy-desktop-full

SHELL [ "/bin/bash" , "-c" ]

# Upgrade all packages
RUN sudo apt update && sudo apt upgrade -y

# Install essential packages
RUN sudo apt install -y wget

# RUN sudo apt-get update \
#     && sudo apt-get install ros-jazzy-webots-ros2 \
#     && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y wget gnupg \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && apt-get update
RUN sudo apt-get install -y libgz-transport13-dev python3-gz-transport13
#     && sudo apt clean

# Create overlay workspace
WORKDIR /root/ros_ws/src
COPY packages.repos .

RUN vcs import < packages.repos; \
    cd ..; \
    rosdep install --from-paths src --ignore-src -r -y; \
    source /opt/ros/${ROS_DISTRO}/setup.bash; \
    colcon build;

# Add sourcing ROS setup.bash to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
