# PX4 Space System docker container

## Overview
This docker file is used as a way to containerize the simulation environment of PX4 space systems. The simulation run in Ubuntu 22.04 container and uses the new gazebo garden as a simulation engine.
Even tho the work is containerized, you still need to clone the repository to your own computer, since it is easier to work with shared folders, rather than cloning the repository to inside the docker.

If it is intended to use with ROS (inside the container) you should use ROS2 humble.

## Software packed inside

Inside the container you have packed Gazebo Garden, ros2 humble, micro ros agent, px4 msgs, qt version 6.6.3 and QGroundControl (with custom plugin for gz-sim7).

## Compiling

Compiling the docker is rather simple you must ensure that you have [docker installed](https://docs.docker.com/engine/install/), after this simply build the container, as you can see in the command bellow you can specify the link and the branch for the PX4 git, this allow for more modularity when using in fork of the main work. **Be aware that when changing the branch or link you will need to compile the full container, and this is a lengthy process**

```
docker build -t px4 --build-arg PX4_SOFTWARE_REPO=https://github.com/SpaceBotsISR/PX4-Space-Systems.git --build-arg PX4_SOFTWARE_BRANCH=pr-space_cobot .
```

In the computer where this was developed the build process toke around 20 minutes, and needed 16GB of RAM and 16GB of swap space and during the gazebo compilation process the full 16 cores were occupied with build the docker.

After compiling you now need to create the container, this can be done with the followint command:
```
docker run -it --gpus all \  
  --network=host --ipc=host \  
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev/dri:/dev/dri \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  --runtime=nvidia \
  -v /home/andret/ISR/PX4-Space-Systems:/home/px4space/PX4/PX4-Space-Systems \
  --name px4_cont px4
```

Most of the flags are used are to pass a gpu to docker and be able to use it for running the gazebo simulation, if you don' t have a gpu, or is not a nvidia gpu I recommend removing the flags, since the base container is from nvidia in this docker. The last flag passed with the **-v"** options used to forward to the container the PX4-Space-Systems folder in the root computer, here we can see an example of how it is done in my computer, beware you will need to change the full path before **':'** as the path after this is relative to the interior of the container.

### First time running.

Since now we forward the **PX4-Space-Systems** folder to the docker container, the first time running the docker some actions are required,  you will need to install all the dependencies as well as compile the contents of the repository. To this simply execute the next command.  
```bash
pushd PX4/PX4-Space-Systems && \
	././Tools/setup/ubuntu.sh -y && \
	pip3 install -r Tools/setup/requirements.txt && \
	make px4_sitl && \
	popd
```

After this the container should be ready to use with the normal functionalities of the simulation tool, to test run the following command in a terminal of the container
```bash
make px4_sitl gz_x500
```
And verify if a gazebo window opens with the x_500 robot in the simulation environment. 

## Using the container

The current test done to the simulation environment involve
 - Gazebo and QGroundControl running inside the docker container
 - Gazebo, QGroundControl and ros2 nodes running inside the docker
 - Gazebo, micro-ros and ros2 nodes inside this docker, with QGroundControl in the host machine and more ros2 node in an external container 

When you open the docker this is the result of the ls command 
```
Documents  ISR	QGroundControl	   aqtinstall.log  qt
Gazebo	   PX4	QGroundControlApp  microros_ws	   ros2_ws
```

The QGroundControlApp is the executable, and the simulation toll are accessible through PX4/PX4-Space-Systems