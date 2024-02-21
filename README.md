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
sudo apt install ros-noetic-joint-state-publisher ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-teleop-twist-keyboard
```

## Start the simulation
With the workspace set up and dependencies installed, you're ready to launch the Vitirover simulation:
```
roslaunch mobile_robot gazebo_sim.launch
```

## Testing
In a new terminal, after re-sourcing ros and your workspace, you can send velocity commands to the robot's right rear wheel using:

```
rostopic pub -1 /vitirover/right_rear_wheel_velocity_controller/command std_msgs/Float64 "data: 1.0"
```
## Using cmd_vel topic 

Open a terminal and navigate to [this script](/src/mobile_robot/scripts/command_robot_gazebo.py) using:
```
cd ~/vitirover_ws/src/mobile_robot/scripts
```

and run:
```
python command_robot_gazebo.py
```

Open an other terminal and run 
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Use the commands on your keyboard (u,i,o...) to move the robot in Gazebo. 
NB: Reduce the linear and angular for a smooth motion, i.e. 0.3 m/s for linear velocity and 0.3 rad/s for the angular velocity.
## Support

For questions, issues, or feature requests, please open an issue on this GitHub repository. We're here to help make your experience with the Vitirover academic robot as smooth as possible.
