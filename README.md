# ZAL manipulators

This package connects the Robotiq end effectors *3-Finger Adaptive Robot
Gripper* and *2-Finger 2F-85 Gripper* with Universal Robot arms.

## Installation

#### Dependencies

This software requires a system setup with Robot Operating System (ROS).
It is recommended to use **Ubuntu 20.04** with
**[ROS Noetic](http://wiki.ros.org/noetic/Installation)** and
**[Catkin](https://catkin-tools.readthedocs.io/en/latest/installing.html)**.

The packages from
[Universal Robot](https://gitlab.zal.aero/stephan.rediske/universal_robot)
as well as those from
[Robotiq](https://gitlab.zal.aero/stephan.rediske/robotiq)
are required.

#### Building

```bash
# source global ros
source /opt/ros/noetic/setup.bash

# create a catkin workspace
mkdir -p catkin_ws/src && cd catkin_ws

# clone the repository
git clone https://gitlab.zal.aero/stephan.rediske/zalmani

# install dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace
catkin build

# activate the workspace (ie: source it)
source devel/setup.bash
```

## Usage

To start the simulation, run one of the top-level launch files, e.g.:

    roslaunch zalmani_gazebo ur10e_3f.launch

#### Note

For simplicity, an argument has been added to the top-level launch files
to switch between position and effort interface. Just use the flag
`ur_use_effort_controller:=false` to start the UR with position
interface and `robotiq_use_effort_controller:=false` to start the
end effector with position interface.
By default the UR starts with a position interface and the end effector
with an effort interface.

There are more flags available and for clarity, a lot of them have been
looped through to the top-level launch files. In your own application
you would usually only pass the necessary flags to the top-level files
and set all other values with the 'default=' argument in lower level
files.

For example:

    roslaunch zalmani_gazebo ur10e_3f.launch ur_use_effort_controller:=true robotiq_use_effort_controller:=false spawn_z:=1.0

spawns the UR10e with effort interface and
Robotiq 3-Finger Adaptive Robot Gripper with position interface,
the base base from the UR10e would be at 1.0 m height.
