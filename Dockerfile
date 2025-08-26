# Layer 1: Base image
FROM ros:foxy-ros1-bridge-focal

# Set environment to prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Step 1: Install basic utilities and tools
# Layer 2
RUN apt-get update && \
    apt-get install -y git && \
    rm -rf /var/lib/apt/lists/*

# Layer 3
RUN apt-get update && \
    apt-get install -y chrony && \
    rm -rf /var/lib/apt/lists/*

# Layer 4
RUN apt-get update && \
    apt-get install -y tmux && \
    rm -rf /var/lib/apt/lists/*

# Layer 5
RUN apt-get update && \
    apt-get install -y vim && \
    rm -rf /var/lib/apt/lists/*

# Step 2: Install ROS-related packages one by one to isolate issues
# Layer 6
RUN apt-get update && \
    apt-get install -y ros-noetic-kobuki-ftdi && \
    rm -rf /var/lib/apt/lists/*

# Layer 7
RUN apt-get update && \
    apt-get install -y ros-noetic-image-transport && \
    rm -rf /var/lib/apt/lists/*

# Layer 8
RUN apt-get update && \
    apt-get install -y ros-noetic-image-geometry && \
    rm -rf /var/lib/apt/lists/*

# Layer 9
RUN apt-get update && \
    apt-get install -y ros-noetic-depth-image-proc && \
    rm -rf /var/lib/apt/lists/*

# Layer 10
RUN apt-get update && \
    apt-get install -y ros-noetic-joy && \
    rm -rf /var/lib/apt/lists/*

# Layer 11
RUN apt-get update && \
    apt-get install -y ros-noetic-ecl* && \
    rm -rf /var/lib/apt/lists/*

# Continue adding individual layers for each ROS package

# Layer 12
RUN apt-get update && \
    apt-get install -y ros-noetic-move-base-msgs && \
    rm -rf /var/lib/apt/lists/*

# Layer 13
RUN apt-get update && \
    apt-get install -y ros-noetic-base-local-planner && \
    rm -rf /var/lib/apt/lists/*

# Layer 14
RUN apt-get update && \
    apt-get install -y ros-noetic-kdl-conversions && \
    rm -rf /var/lib/apt/lists/*

# Layer 15
RUN apt-get update && \
    apt-get install -y ros-noetic-roslint && \
    rm -rf /var/lib/apt/lists/*

# Layer 16
RUN apt-get update && \
    apt-get install -y ros-noetic-pcl-conversions && \
    rm -rf /var/lib/apt/lists/*

# Layer 17
RUN apt-get update && \
    apt-get install -y ros-noetic-pcl-ros && \
    rm -rf /var/lib/apt/lists/*

# Layer 18
RUN apt-get update && \
    apt-get install -y ros-noetic-resource-retriever && \
    rm -rf /var/lib/apt/lists/*

# Layer 19
RUN apt-get update && \
    apt-get install -y ros-noetic-laptop-battery-monitor && \
    rm -rf /var/lib/apt/lists/*

# Layer 20
RUN apt-get update && \
    apt-get install -y ros-noetic-diagnostic-aggregator && \
    rm -rf /var/lib/apt/lists/*

# Layer 21
RUN apt-get update && \
    apt-get install -y ros-noetic-urdf && \
    rm -rf /var/lib/apt/lists/*

# Layer 22
RUN apt-get update && \
    apt-get install -y ros-noetic-interactive-markers && \
    rm -rf /var/lib/apt/lists/*

# Layer 23
RUN apt-get update && \
    apt-get install -y ros-noetic-xacro && \
    rm -rf /var/lib/apt/lists/*

# Layer 24
RUN apt-get update && \
    apt-get install -y ros-noetic-robot-state-publisher && \
    rm -rf /var/lib/apt/lists/*

# Layer 25
RUN apt-get update && \
    apt-get install -y libyaml-cpp-dev && \
    rm -rf /var/lib/apt/lists/*

# Layer 26
RUN apt-get update && \
    apt-get install -y python3 && \
    ln -s /usr/bin/python3 /usr/bin/python && \
    rm -rf /var/lib/apt/lists/*

# Step 3: Install cmake_modules to avoid missing dependency error during catkin_make
# Layer 27
RUN apt-get update && \
    apt-get install -y ros-noetic-cmake-modules && \
    rm -rf /var/lib/apt/lists/*

# Step 4: Additional ROS utilities: rqt_graph, rqt_image_view, and image_tools
# Layer 28
RUN apt-get update && \
    apt-get install -y ros-noetic-rqt-graph && \
    rm -rf /var/lib/apt/lists/*
    
# Layer 29: Install rqt for ROS2 Foxy
RUN apt-get update && \
    apt-get install -y \
        ros-foxy-rqt \
        ros-foxy-rqt-common-plugins \
        ros-foxy-rqt-graph && \
    rm -rf /var/lib/apt/lists/*

# Layer 30
RUN apt-get update && \
    apt-get install -y ros-noetic-rqt-image-view && \
    rm -rf /var/lib/apt/lists/*

# Layer 31
RUN apt-get update && \
    apt-get install -y ros-foxy-image-tools && \
    rm -rf /var/lib/apt/lists/*

# Set up workspace and clone repositories
# Layer 32
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/turtlebot/turtlebot.git && \
    git clone https://github.com/turtlebot/turtlebot_apps.git && \
    git clone https://github.com/turtlebot/turtlebot_interactions.git && \
    git clone https://github.com/turtlebot/turtlebot_simulator.git && \
    git clone https://github.com/turtlebot/turtlebot_msgs.git && \
    git clone https://github.com/yujinrobot/kobuki.git && \
    git clone https://github.com/yujinrobot/kobuki_msgs.git && \
    git clone https://github.com/yujinrobot/yujin_ocs.git && \
    git clone https://github.com/yujinrobot/kobuki_core.git && \
    git clone https://github.com/yujinrobot/yocs_msgs.git && \
    git clone https://github.com/ros-perception/ar_track_alvar.git && \
    cd ar_track_alvar && git checkout noetic-devel

# Step 5: Build the workspace without ROS_DISTRO conflict
# Layer 33
SHELL ["/bin/bash", "-c"]
WORKDIR /root/catkin_ws
RUN unset ROS_DISTRO && \
    source /opt/ros/noetic/setup.bash && \
    catkin_make && \
    source devel/setup.bash

# Step 6: Update bashrc with environment settings only
# Layer 34
RUN echo "export ROS1_INSTALL_PATH=/opt/ros/noetic" >> ~/.bashrc && \
    echo "export ROS2_INSTALL_PATH=/opt/ros/foxy" >> ~/.bashrc && \
    echo "export TURTLEBOT_SERIAL_PORT='/dev/ttyUSB0'" >> ~/.bashrc

# Layer 35: Set environment variables
ENV TURTLEBOT_BASE=kobuki \
    TURTLEBOT_3D_SENSOR=astra \
    TURTLEBOT_STACK=hexagons \
    TURTLEBOT_SERIAL_PORT=/dev/ttyUSB0

# Final command
# Layer 36
CMD ["bash"]


