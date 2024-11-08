# Autonomous Ground Vehicle
Navigation system for an Autonomous Ground Vehicle (AGV) that detects and avoids obstacles.

## Description
Simplied version of an obstacle detection and avoidance system for an AGV. The AGV navigates towards a goal while avoiding obstacles. The system uses data from a Lidar to detect and avoid obstacles. The AGV will navigate using a set of points as input. It will move from point to point while avoiding obstacles.

## Environment

All development and simulation was done using Windows Linux Subsystem (WSL)
- Ubuntu distro: 22.04.5 LTS (Jammy)
- ROS2 distro: Humble
- Gazebo distro: Fortress LTS

To avoid warnings and errors when lauching gazebo, please append the following to your ```.bashrc``` file.

```
# Source ROS environment
source /opt/ros/humble/setup.bash

# Handle seg fault error when launching gazebo
export LIBGL_ALWAYS_SOFTWARE=1

# Fix issue: QStandardPaths: wrong permissions on runtime directory /run/user/1000/, 0755 instead of 0700
chmod 0700 /run/user/1000/
```

## Documentation
Software design architecture for an AGV can be found under the ```doc``` directory in file ```AGV_Software_Design.pdf```.<br />
Under the same folder, you can find an explanation about my obstacle avoidance algorithm in file ```AGV_Obstacle_Avoidance_Algorithm.pdf```.

## Disclamer

This article assumes having basic knowledge of ROS, Gazebo, LiDARs, and Linux, since these will not be covered here.

## How to run

Build ROS node that handles navigation and obstacle avoidance

```
$ ./build_agv.sh
```

Update ```waypoint.txt``` text file with set of local coordinate points to follow, one coordinate per line. Each X,Y coordinate pair should be separated using a comma. There is no syntax check, so please follow this format.  (i.e):

```
4.0, 0.0
8.0, 0.0
12.0, 0.0
```

Run simulator launch script <br />

```
$ ./run_agv.sh
```

To run simulator with moving obstacles (actors), run script:

```
$ ./run_agv.sh -a
```

## AGV World

![Screenshot](/agv_world_preview.jpg)

The mini green boxes floating in the air represent the X,Y coordinates from the waypoint.txt file. They are a visual represantion of the path the AGV should take.
#### NOTE
These are hard-coded into the SDF file and DO NOT get dynamically generated based on the content of waypoint.txt. If you update the coordinates on the waypoint.txt, you will have to manually update the SDF file if you want the green boxes to match your new coordinates.

## Author info

[Linkedin](https://www.linkedin.com/in/miguel-a-duenas-sr1)
