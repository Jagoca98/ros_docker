# ROS Noetic Docker Container
This container is based on the official ROS Noetic image. It is prepared to work with display and gpus.
Original Image: osrf/ros:noetic-desktop

## Usage
Before starting the container, modify the file named `config.env` in the root directory of this repository. This file should contain the following environment variables:
```bash
USER_ID=1000                        # This should change to gain permisions - Run `id` to get your user id
USER_NAME=jaime                     # This should change to gain permisions - Run `id` to get your user name
GROUP_ID=1000                       # This should change to gain permisions - Run `id` to get your group id
GROUP_NAME=oompaloompas             # This should change to gain permisions - Run `id` to get your group name
WORKSPACE=catkin_ws                 
DOCKER_IMAGE_NAME=noetic
```

The WORKSPACE variable should be the name of the workspace you want to create in the container.


In a terminal, run the following command to start the container:
```bash
docker compose --env-file config.env up --build
```

This will start the container and run the roscore command.

In another terminal, run the following command to start a bash session in the container:
```bash
docker exec -it noetic bash
```

## TroubleShooting
If you get an error due to `runtime: nvidia` flag in the docker-compose file, you can comment it and run as normal. This will disable the GPU support.

## Contact
Contact
For any questions or feedback, feel free to reach out to the author:

Author: [Jaime Godoy](mailto:jgodoy@pa.uc3m.es)

Last Update: September 19, 2024

