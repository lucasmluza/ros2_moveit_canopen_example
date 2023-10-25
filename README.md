# ROS2 MoveIt CANopen Example

This repository implements the use of different ROS2 packages to control a CANopen-based robot in a simulated environment.

## Dependencies

- Docker (tested with version 24.0.6)
- Docker Compose (tested with version v2.21.0)

## Build

### Clone this repository

```bash
git clone --recurse-submodules git://github.com:lucasmluza/ros2_moveit_canopen_example.git
```

### Build the image

```bash
docker compose build
```

### Create, start, and execute the container

For creating and starting the container:

```bash
docker compose up 
```

For executing commands in the container:

```bash
docker compose exec ros2_humble [command]
```

A simple approach is to run bash on the container, that will start interactively since the compose.yaml set the ```tty``` and ```stdin_open``` to true.

```bash
docker compose exec ros2_humble bash
```

### Install ROS2 dependencies

First, install de dependencies with rosdep. In ```~/ws_ros```:

1 - Update the apt repositories

```bash
sudo apt update
```

2 - Initialize rosdep

```bash
sudo rosdep init && rosdep update
```

3 - Install dependencies

```bash
rosdep install -i --from-paths src -y
```

### Build workspace packages

In ```~/ws_ros```:

```bash
colcon build
```

And, do not forget to source the ```install/setup.bash``` after compiling.

In ```~/ws_ros```:

```bash
source install/setup.bash
```

## Run and launch packages

### panda_rviz_viewer

For visualize the robot and control the joints position with a GUI app:

```bash
ros2 launch panda_rviz_viewer view.launch.py
```
![panda_rviz_viewer](https://github.com/lucasmluza/ros2_moveit_canopen_example/assets/36889165/cc8b42a5-a200-4fd4-9b64-44321ea590cd)

### panda_canopen

For controlling the robots joints with ROS2 services:

```bash
ros2 launch panda_canopen simulation.launch.py
```

Then, in a second bash instance of the container, the joints can be controlled by the exposed services. Examples:

- Initialize a joint:
    ```bash
    ros2 service call /panda_joint1/init std_srvs/srv/Trigger
    ```
- Set the operation mode to ```Profile Position Mode```:
    ```bash
    ros2 service call /panda_joint1/position_mode std_srvs/srv/Trigger
    ```
- Set a target position value:
    ```bash
    ros2 service call /panda_joint1/target canopen_interfaces/srv/COTargetDouble "{ target: 10.0 }"
    ```

For visualizing the robot movement with Rviz2, in a third bash instance of the container:

```bash
ros2 launch panda_canopen rviz.launch.py
```

![panda_canopen](https://github.com/lucasmluza/ros2_moveit_canopen_example/assets/36889165/d97c9c87-2020-4b4f-a40f-21a1765dff1f)

### panda_moveit

For visualize the robot, and plan and execute trajectories using the MoveIt2 plugin in Rviz2:

```bash
ros2 launch panda_moveit simulation.launch.py
```

![panda_moveit](https://github.com/lucasmluza/ros2_moveit_canopen_example/assets/36889165/522df9f0-1bb9-42a5-bc09-892a7ed8e358)

## References

This repository is based, and imported files from the following:

### panda_description

- From: https://github.com/ros-planning/moveit_resources/tree/ros2/panda_description
- Branch: humble
- Commit: #5cf047b
- License: Apache-2.0

### ros2_canopen

- From: https://github.com/ros-industrial/ros2_canopen
- Branch: humble
- Commit: #e0b5706
- License: Apache-2.0

Added as a submodule.
