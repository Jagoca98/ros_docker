# ros_docker
This aims to be a template to work with ROS and Docker


## How to use this template

This template mount the ``catkin_ws`` folder in the container, so you can work with your ROS packages in the container. 

**1-** Git clone the template.

```bash
git clone https://github.com/Jagoca98/ros_docker.git
cd ros_docker
```

**2-** Now enter the directory of the ROS version you want to use, for example:

```bash
cd humble
```

**3-** As it's said, you need to copy your ROS workspace here and rename it as ``catkin_ws`` in order to mount it in the docker.

**4-** After that, you can build the docker image with the following command:

```bash
docker-compose build
docker-compose up
```

**5-** Now you can open a new terminal and run the following command to enter the container. Following the example of the previous step:

```bash
docker exec -it humble bash
```
The container name is the same as the folder name, so if you are using the noetic version, you should replace ``humble`` with ``noetic``.

In the case of using ROS 1 distros, ``docker-compose up`` also launches ``roscore`` at the default port (11311). If you want to use a different port, you can change it in the ``docker-compose.yml`` file.