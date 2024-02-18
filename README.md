# vitirover_ws

Welcome to the GitHub repository for the Vitirover academic robot! This repository contains the ROS1 workspace needed to simulate and interact with the Vitirover in a virtual environment. Follow the instructions below to get started.

## Prerequisites

Ensure you have ROS Noetic installed on your system. This workspace has been tested and confirmed to work with ROS Noetic.

## Setup

First, clone this repository into your desired directory. If you haven't already created a directory for the Vitirover workspace, you can do so by following these steps:

```
source /opt/ros/noetic/setup.bash
```

Clone our repository:
```
git clone https://github.com/vitirover/vitirover_ws.git
```

Then, build the catkin workspace:
```
cd ~/vitirover_ws/
catkin_make
```
and source it:

```
source devel/setup.bash
```

## Install dependencies

Before starting the simulation, make sure all the necessary dependencies are installed:
```
sudo apt install ros-noetic-joint-state-publisher ros-noetic-ros-control ros-noetic-ros-controllers
```

## Start the simulation
With the workspace set up and dependencies installed, you're ready to launch the Vitirover simulation:
```
roslaunch mobile_robot gazebo_sim.launch
```
