# Use minimal ROS Noetic base image
FROM ros:noetic-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install required ROS packages and development tools
RUN apt-get update && apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
        git \
        python3-pip \
        ros-noetic-catkin \
        ros-noetic-vision-msgs \
        ros-noetic-rosbridge-server \
        ros-noetic-image-transport-plugins \
        ros-noetic-image-geometry \
        ros-noetic-image-view \
        ros-noetic-cv-bridge \
        ros-noetic-pcl-ros \
        ros-noetic-image-transport \
        ros-noetic-message-filters \
        ros-noetic-rviz \
        ros-noetic-rqt \
    	ros-noetic-rqt-common-plugins \
    	ros-noetic-rqt-image-view \
        ros-noetic-message-generation \
        ros-noetic-message-runtime \
        python3-catkin-tools && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Ensure 'python' points to 'python3'
RUN ln -s /usr/bin/python3 /usr/bin/python

# Install Python dependencies, including 'getch'
RUN pip3 install --no-cache-dir getch

# Create the catkin workspace inside the container
RUN mkdir -p /root/catkin_ws/src && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws/src && catkin_init_workspace"

# Clone ultralytics_ros repository
#RUN git clone -b noetic-devel https://github.com/Alpaca-zip/ultralytics_ros.git /root/catkin_ws/src/ultralytics_ros

# Instead of cloning from GitHub, copy the modified repository from the host
COPY ultralytics_ros /root/catkin_ws/src/ultralytics_ros
# Copy Unity ROS package for teleoperation into the image
COPY unity_ros_pkgs/controllerpkg /root/catkin_ws/src/controllerpkg

# Install Python dependencies inside the container
RUN pip3 install -r /root/catkin_ws/src/ultralytics_ros/requirements.txt

# Build the ROS workspace using `catkin_make`
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make"

# Source workspace in bashrc for all new shells
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Copy entrypoint script and make it executable
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set entrypoint script
ENTRYPOINT ["/entrypoint.sh"]

# Default command: keep container interactive
CMD ["bash"]

