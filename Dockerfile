FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Set locale
RUN apt-get update && apt-get install -y \
    locales \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    curl \
    lsb-release \
    gnupg \
    && curl -sSL http://packages.ros.org/ros/ubuntu/gpg.key | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-get update

# Install ROS Noetic and related tools
RUN apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Create catkin workspace
RUN mkdir -p ~/tiago_ws/src
WORKDIR ~/tiago_ws

# Set the workspace environment
ENV ROS_WORKSPACE=~/tiago_ws
RUN echo "source ~Mae1n gallu, mae1n gallu, mae1n gallu, mae1n gallu, mae1n gallu, mae1n gallu, mae1n gallu. /tiago_ws/devel/setup.bash" >> /root/.bashrc


CMD ["bash"]
