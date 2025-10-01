# PX4 Space Systems Docker Container

## Overview

This image packages the PX4 Space Systems simulation environment inside an Ubuntu 22.04 container running Gazebo Garden. The repository is mounted into the container as a shared volume, so clone the project on your host machine first.

Use ROS 2 Humble inside the container when ROS integration is required.

## Included Software

- Gazebo Garden
- ROS 2 Humble
- micro-ROS agent
- `px4_msgs`
- Qt 6.6.3
- QGroundControl (with the custom gz-sim7 plugin)

## Prerequisites

1. Install [Docker Engine](https://docs.docker.com/engine/install/).
2. Ensure `docker compose` v2 is available:

   ```bash
   docker compose version
   ```

   The output should resemble `Docker Compose version v2.36.2`.

## Build the Image

From `Dockerfiles/`, run:

```bash
docker compose build
```

Building the image is CPU- and memory-intensive; expect up to 40 minutes and ~32 GB RAM to complete.

## Configure Shared Volumes

Shared folders are configured in `docker-compose.yml`. The default volumes mapping is:

```yaml
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - /dev/dri:/dev/dri
      - ../../PX4-Space-Systems:/home/px4space/PX4/PX4-Space-Systems
```

To add another shared folder, append an entry under the `volumes` section:

```yaml
      - ./path/on/host:/path/in/container
```

Use relative paths where possible so collaborators do not need to mirror your directory layout.

## Run the Container

Start the container (from `Dockerfiles/`):

```bash
docker compose up -d
```

Open a shell inside the running container:

```bash
docker exec -it px4_cont bash
```

**GPU note:** The configuration assumes an NVIDIA GPU. For other GPUs, remove the NVIDIA-specific options from `docker-compose.yml`. This has not been validated.

## First-Time Setup

The first run requires dependency installation and workspace builds. Inside the container shell run:

```bash
pushd PX4/PX4-Space-Systems && \
  sudo chmod +x ./Tools/setup/ubuntu.sh && \
  sudo ./Tools/setup/ubuntu.sh -y && \
  pip3 install -r Tools/setup/requirements.txt && \
  pip3 install -r Tools/setup/optional-requirements.txt && \
  make px4_sitl && \
  popd

pushd "$HOME"/PX4/ros2_ws && \
  source /opt/ros/humble/setup.bash && \
  source "$HOME"/Gazebo/install/setup.sh && \
  colcon build --symlink-install --packages-select px4_msgs ros_gz_interfaces ros_gz_bridge
```

## Quick Simulation Check

Validate the setup by launching the default simulation:

```bash
cd "$HOME"/PX4/PX4-Space-Systems
make px4_sitl gz_x500
```

Gazebo should display the X500 drone. Launch QGroundControl:

```bash
./QGroundControlApp
```

Add a joystick in settings, arm the vehicle, and apply throttle. Map tiles are currently unavailable inside the container (known issue).

## Advanced Space Cobot Simulation

### Set Up Supporting Workspaces

1. Clone the helper repository if needed:

   ```bash
   git clone https://github.com/Planning-and-Control-in-Space-Cobot/ros2 cobotGazeboUtils
   ```

2. Build the Gazebo–ROS 2 bridge workspace:

   ```bash
   pushd cobotGazeboUtils/ros2_ws_px4
   colcon build --symlink-install
   source install/setup.bash
   popd
   ```

3. Build the offboard controller workspace:

   ```bash
   pushd cobotGazeboUtils/ros2_ws_cobot
   colcon build --symlink-install
   source install/setup.bash
   popd
   ```

### Run the Simulation

1. Launch Gazebo with the Space Cobot world:

   ```bash
   cd "$HOME"/PX4/PX4-Space-Systems
   make px4_sitl gz_space_cobot
   ```

2. Start the micro-ROS agent in a new terminal:

   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

3. Launch the Gazebo–ROS 2 bridge:

   ```bash
   pushd cobotGazeboUtils/ros2_ws_px4
   source install/setup.bash
   ros2 launch gz_bridge gz_bridge.launch.py
   popd
   ```

4. Switch the robot to offboard mode:

   ```bash
   pushd cobotGazeboUtils/ros2_ws_cobot
   source install/setup.bash
   ros2 run OffBoardModeGazebo OffBoardModeGazebo
   popd
   ```

5. Publish actuator commands to start control:

   ```bash
   ros2 topic pub /space_cobot_0/motor_command actuator_msgs/msg/Actuators "{
     header: {
       stamp: {sec: 0, nanosec: 0},
       frame_id: ''
     },
     position: [],
     velocity: [1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0],
     normalized: []
   }"
   ```

## Issues

### Known Issues

- QGroundControl map tiles fail to load inside the container.
- A shell must be opened with `docker exec` to interact with the container.

### Reporting New Issues

Report container-specific bugs in the [PX4-Space-Systems GitHub repository](https://github.com/SpaceBotsISR/PX4-Space-Systems). Include reproduction steps and assign the issue to `andre-rebelo-teixeira`. For issues affecting Gazebo, PX4, or QGroundControl outside this container, contact the respective upstream maintainers.
