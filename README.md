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

```bash
collection_robot
├───docs
├───include
│   └───collection_robot
├───launch
├───src
└───UML
    └───initial
```

### Building Instructions

Clone the repo in src folder of ros2 workspace:

```bash
git clone https://github.com/muditsingal/collection_robot.git -b dev
```

Build the ros2 package:

```bash
colcon build --packages-select collection_robot
```




### Sprint planning Sheet:

[ENPM808X Final Project Sprint Planning Sheet](https://docs.google.com/spreadsheets/d/1aB_AL3CoJv4jf_V5iHIeneE0IcUH5RtSz64aUaEVvbM/edit?usp=sharing)

[ENPM808X Final Project Sprint Review Notes](https://docs.google.com/document/d/11TBs6DGolvmfTOMxNTo-zaF9SJSSREofYDMhL7Y_Msg/edit?usp=sharing)


