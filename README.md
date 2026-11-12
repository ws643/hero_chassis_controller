# hero_chassis_controller


## Overview

this program is a controller that using ROS1 to controll an robot named hero , this program can be launched in Gazebo , and can achieve odometry implementation via Mecanum Wheel Forward and Inverse Kinematics

**Keywords:** chassis controller,robomaster,four-wheeled mecanum chassis,ros-control

### License 

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: ws643<br />
Affiliation: DynamicX<br />
Maintainer: ws643, 2716823087@qq.com**

Tested under ROS Noetic on Ubuntu 20.04 in docker.

## Features

- Calculates the four target wheel speeds from chassis velocity commands using inverse kinematics.
- Uses PID controllers to track the target wheel speeds.
- Calculates chassis velocity from the actual wheel speeds using forward kinematics.
- Publishes odometry on `/odom` and broadcasts the `odom` to `base_link` transform.
- Supports chassis-frame and odometry-frame velocity command modes.
- Supports keyboard control with `teleop_twist_keyboard`.
- Supports configurable chassis acceleration.

## Requirements

- ROS Noetic
- Gazebo Classic
- `rm_description_for_task`
- `teleop_twist_keyboard`

The package was developed and tested with ROS Noetic in an Ubuntu 20.04 Docker container.

## Build

From the root of your catkin workspace:

```bash
cd /catkin_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin build hero_chassis_controller
source /catkin_ws/devel/setup.bash
```
## Run the Simulation

Start the simulation and load the controller:
```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
roslaunch hero_chassis_controller hero_chassis_controller.launch
```

The launch file starts Gazebo, spawns the Hero chassis, loads the controller configuration, and starts the chassis and joint-state controllers.
To stop the simulation, press Ctrl+C in the terminal running roslaunch.


## Control the Chassis


With the simulation running, open another terminal in the same ROS container and run:
```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.5 _turn:=0.5
```

The keyboard node publishes velocity commands to /cmd_vel. The chassis controller subscribes to /cmd_vel, calculates the target wheel speeds, and sends commands to the wheel joints.
Use the keys shown in the terminal to move the chassis. Press Ctrl+C in the keyboard-control terminal to stop the keyboard node.

## Challenge Configuration
Controller parameters are configured in config/controllers.yaml.

 - wheel_radius: wheel radius.
 - wheel_track: distance between the left and right wheel lines.
 - wheel_base: distance between the front and rear wheel lines.
 - pid: PID parameters for the four wheel-speed controllers.
 - which_mode: selects the velocity-command coordinate frame. false uses the chassis frame; true uses the odometry frame.
 - a: acceleration setting used by the chassis acceleration feature.
The controller publishes odometry on /odom using the nav_msgs/Odometry message type. The odometry message describes the robot pose and velocity, with odom as the parent frame and base_link as the child frame. The same frame relationship is broadcast through TF.

