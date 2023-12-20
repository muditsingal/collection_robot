# Project-Magpie

ENPM808X - Mudit Singal, Abhishekh Reddy and Abhimanyu Saxena

## Project Status

![CICD Workflow status](https://github.com/muditsingal/collection_robot/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)

[![codecov](https://codecov.io/gh/muditsingal/collection_robot/branch/dev2/graph/badge.svg)](https://codecov.io/gh/muditsingal/collection_robot)

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

## Overview

The development in project Magpie involves a collector robot that is expected to visually locate garbage items (we will be utilizing blocks with ~~April tags~~ solid colors as a generic trash item) and collect them. It does so by searching the surroundings. This project makes use of a Turtlebot robot that roams in a given area, looks for April tag blocks which represent a generic garbage item and removes them as an action for collection.

[Demonstration Video](https://www.youtube.com/watch?v=ugb6bS5b1xc)

[Sprint Planning Sheet](https://docs.google.com/spreadsheets/d/1aB_AL3CoJv4jf_V5iHIeneE0IcUH5RtSz64aUaEVvbM/edit?usp=sharing)

[Sprint Plan and Review Notes](https://docs.google.com/document/d/11TBs6DGolvmfTOMxNTo-zaF9SJSSREofYDMhL7Y_Msg/edit?usp=sharing)

## Environment and dependencies

- Ubuntu Jammy (22.04)
- ROS2 Humble Hawksbill (Even base installation is sufficient)
- Git

### Package dependencies

- `std_msgs` - Standard Messages Library
- `sensor_msgs` - Sensor Messages Library
- `geometry_msgs` - Geometry Messages Library
- `gazebo_ros_pkgs` - Gazebo simulator + ROS2 plugins with Gazebo
- `nav_msgs` - Navigation Messages Library
- `tf2_ros` - Package API for working with reference frames and transformations
- `image_transport` - Package used as a medium of publishing/subscribing images to topics
- `cv_bridge` - Translation layer between ROS2 messages and OpenCV library datatypes
- `opencv` - OpenCV package for Computer Vision
- `rclcpp` - ROS2 C++ Client Library
- `rclpy` - ROS2 Python Client Library
- `turtlebot3` - Turtlebot3 ROS2 Packages for simulations

Installing any of these packages manually is not required, they can be installed
using `rosdep` which will be discussed in a future step.

## Cloning the repository

The root directory of this repository are contents of a ROS2 package. Hence,
this needs to be cloned to a folder in the `src/` directory of a ROS2 project
workspace.

```bash
<Your ROS2 Workspace>/
├── build/
├── install/
├── log/
└── src/
    └── beginner_tutorials/ <-- Repository clones to this folder
        └── <Repository contents>
```

<p align="center">Workspace directory tree with this repository contents in it</p>

Clone the repository in the `src/` directory of your ROS2 workspace

```bash
git clone https://github.com/muditsingal/collection_robot.git collection_robot
```

>**Before building the packages:** For the first time, ensure that the package
> dependencies are installed. Run these commands from the **root workspace directory**.

```bash
rosdep init && rosdep update
```

```bash
rosdep install --from-paths src -y --ignore-src
```

Build the package in the **root directory** of your ROS2 workspace.

```bash
colcon build --packages-select collection_robot
```

Source the freshly built package

```bash
source install/setup.bash
```

## Running the simulation

Set the environment variable to choose the Waffle Pi variant of Turtlebot 3 in
simulations.

```bash
export TURTLEBOT3_MODEL=waffle_pi
```

The simulation can now be launched

```bash
ros2 launch collection_robot collector_robot.launch.py
```

## Known issues

Even though GitHub CI and CodeCov are working in the backend, the badges are not working.

- CodeCov is not generating a report due to which the badge is not working
- GitHub CI badge is not working too
