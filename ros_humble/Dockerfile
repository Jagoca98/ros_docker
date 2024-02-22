FROM osrf/ros:humble-desktop

# Install essential packages
RUN apt-get update && apt-get install -y \
    build-essential \
    iputils-ping \
    net-tools \
    wget \
    nano 

# Create a new catkin workspace inside the container
# COPY ./src /root/catkin_ws/src
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

# Set the default entry point to start the ROS environment
# CMD ["/bin/bash"]
CMD ["tail", "-f", "/dev/null"]