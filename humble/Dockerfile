FROM osrf/ros:humble-desktop-full

# Install essential packages
RUN apt-get update && apt-get install -y \
    build-essential \
    iputils-ping \
    net-tools \
    wget \
    nano 

# Create a new catkin workspace inside the container
WORKDIR /root/catkin_ws
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Set the default entry point to start the ROS environment
CMD ["tail", "-f", "/dev/null"]