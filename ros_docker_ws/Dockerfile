# Use ROS Noetic (Ubuntu 20.04 base)
FROM osrf/ros:noetic-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
WORKDIR /root/catkin_ws

# Install ROS dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    ros-noetic-roscpp \
    ros-noetic-rospy \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-octomap-msgs \
    ros-noetic-octomap-ros \
    ros-noetic-octomap-server \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-msgs \
    ros-noetic-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# Ensure 'python' points to 'python3'
RUN ln -s /usr/bin/python3 /usr/bin/python

# Install Python dependencies, including 'getch'
RUN pip3 install --no-cache-dir getch

# Initialize rosdep
RUN rosdep update

# Create a catkin workspace
RUN mkdir -p /root/catkin_ws/src

# Copy all Unity ROS packages into the image
COPY unity_ros_pkgs/ /root/catkin_ws/src/

# Ensure Python scripts are executable
RUN for pkg in controllerpkg converter_pkg misc octomap_mapping unity_decoder; do \
        if [ -d "/root/catkin_ws/src/$pkg/src" ]; then \
            chmod +x /root/catkin_ws/src/$pkg/src/*.py; \
        fi; \
    done

# Build the ROS workspace inside the image
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
    catkin_make && \
    source devel/setup.bash"

# Ensure ROS is sourced on login
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Copy the startup script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the container's entrypoint
ENTRYPOINT ["/entrypoint.sh"]
