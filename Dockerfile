FROM ros:jazzy-ros-base-noble

ARG DEBIAN_FRONTEND=noninteractive

ENV ROS_WS=/ros2_ws
ENV ROS_DISTRO=jazzy

# Update and install basic dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Update rosdep
RUN rosdep update

# Create workspace
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Copy source files
COPY src ${ROS_WS}/src

# Install dependencies using rosdep
RUN apt-get update && \
    rosdep install --rosdistro ${ROS_DISTRO} --ignore-src --from-paths src -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install"

# Source the workspace in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> ~/.bashrc

# Set entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
