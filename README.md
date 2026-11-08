# hero_chassis_controller


## Overview

hero_chassis_controller is a ros_control controller plugin(插件) for the chassis(底盘) of
a RoboMaster Hero robot. It drives a four-wheeled mecanum chassis in simulation (Gazebo).

**Keywords:** chassis controller,robomaster,four-wheeled mecanum chassis,ros-control

### License 

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: ws643<br />
Affiliation: DynamicX<br />
Maintainer: ws643, 2716823087@qq.com**

Tested under ROS Noetic on Ubuntu 20.04 in docker.



## Installation 安装

### Installation from Packages 从软件包安装

use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source 源码编译

#### Dependencies 依赖

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen] (linear algebra library)
- Ubuntu 20.04 + docker + ROS1
- rm_msgs

  sudo rosdep install --from-paths src

#### Building 编译

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone https://github.com/ws643/hero_chassis_controller.git
	cd ../
	rosdep install --from-paths src --ignore-src -y
	catkin build

### Running in Docker Docker中运行

Docker is a great way to run an application with all dependencies and libraries bundles together. If your ubuntu system isn't ubuntu 20-04 , please make sure
to [install Docker](https://docs.docker.com/get-docker/) first and running ubuntu 20.04 in docker otherwise a strange error may be reported.

First, spin up a simple container:

	docker run -ti --rm --name ros-container ros:noetic bash

This downloads the `ros:noetic` image from the Docker Hub, indicates that it requires an interactive terminal (`-t, -i`)
, gives it a name (`--name`), removes it after you exit the container (`--rm`) and runs a command (`bash`).

Now, create a catkin workspace, clone the package, build it, done!

	apt-get update && apt-get install -y git
	mkdir -p /ws/src && cd /ws/src
	git clone https://github.com/leggedrobotics/ros_best_practices.git
	cd ..
	rosdep install --from-path src
	catkin_make
	source devel/setup.bash
	roslaunch ros_package_template ros_package_template.launch

### Unit Tests 单元测试

Run the unit tests with

	catkin_make run_tests_hero_chassis_controller

### Static code analysis 静态代码检查

#### if roslint havn't settled yet ,then ignore that part.
Run the static code analysis with

	catkin_make roslint_hero_chassis_controller

## Usage 使用方法

Describe the quickest way to run this software, for example:

Run the main node with
	source devel/setup.bash
	roslaunch hero_chassis_controller hero_chassis_controller.launch

## Config files 配置文件

Config file folder/

	default.yaml

## Launch files

* **launch_file_1.launch:** shortly explain what is launched (e.g standard simulation, simulation with gdb,...)

  Argument set 1

    - **`argument_1`** Short description (e.g. as commented in launch file). Default: `default_value`.

  Argument set 2

    - **`...`**

* **...**

## Nodes

### ros_package_template

Reads temperature measurements and computed the average.

#### Subscribed Topics

* **`/temperature`** ([sensor_msgs/Temperature])

  The temperature measurements from which the average is computed.

#### Published Topics

...

#### Services

* **`get_average`** ([std_srvs/Trigger])

  Returns information about the current average. For example, you can trigger the computation from the console with

  	rosservice call /ros_package_template/get_average

#### Parameters

* **`subscriber_topic`** (string, default: "/temperature")

  The name of the input topic.

* **`cache_size`** (int, default: 200, min: 0, max: 1000)

  The size of the cache.

### NODE_B_NAME

...

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/gdut-dynamic-x/rm_template/issues)
.


[ROS]: http://www.ros.org

[rviz]: http://wiki.ros.org/rviz

[Eigen]: http://eigen.tuxfamily.org

[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html

[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
