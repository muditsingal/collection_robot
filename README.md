# Project-Magpie

ENPM808X final project repo

[![codecov](https://codecov.io/gh/muditsingal/collection_robot/branch/dev/graph/badge.svg)](https://codecov.io/gh/muditsingal/collection_robot)

## Overview

The development in project Magpie involves a collector robot that is expected to visually locate garbage items (we will be utilizing blocks with April tags as a generic trash item) and collect them. It does so by searching the surroundings. This project makes use of a Turtlebot robot that roams in a given area, looks for April tag blocks which represent a generic garbage item and removes them as an action for collection.

### Repo Structure

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


