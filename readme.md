# SAYAN BRAHMA - ENPM808X
## ROS Beginner Tutorials - Introduction to Publisher and Subscriber  
[![License: MIT](https://img.shields.io/badge/License-MIT-brightgreen.svg)](https://opensource.org/licenses/MIT)
---

## Overview

This program is a basic introduction to a publisher and a subscriber in ROS. It creates two nodes, a talker (publisher) node and a listener (subscriber) node to demonstarte how they interact with each other.
The process is followed as given on the ROS wiki: http://wiki.ros.org/ROS/Tutorials

## Dependencies
This program works on a device running Ubuntu 16.04 and ROS Kinetic Kame.

The project has following dependencies.

1. ROS Kinetic
2. catkin
3. Ubuntu 16.04 

To install ROS Kinetic Kame in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu).

To install catkin, follow the installation steps in this [link](http://wiki.ros.org/catkin).

**NOTE:- If there is a conflict due to ROS API and no workaround exists for google styling (cpplint) went with ROS API way

## Build Instructions

To run this code in a catkin workspace:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/
git clone https://github.com/sbrahma0/beginner_tutorials.git
cd ..
catkin_make
```

If you do not have a catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/sbrahma0/beginner_tutorials.git
cd ..
catkin_make
```

## Publisher/subscriber working demo
Open a new terminal and give the following commands
```
roscore
```
Open a new terminal and give the following commands
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials talker
```
Open a new terminal and give the following commands
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials listener
```

## Launching the demo using Launch file
#### Without changing the default frequency of the talker node
Open a new terminal and give the following commands
```
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch beginner_tutorials beginner_tutorial.launch 
```
#### Changing the default frequency of the talker node
Open a new terminal and give the following commands (in the integer place if you enter 2 or less warning will be showed). This warning can also be seen in the rqt console as well as in the terminal.
```
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch beginner_tutorials beginner_tutorial.launch frequency:=<integer>
```
## Checking the rqt console
First launch the demo from either of the above mentioned methods, after that do the following:
Open a new terminal and give the following commands
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun rqt_console rqt_console
```
Open a new terminal and give the following commands
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun rqt_logger_level rqt_logger_level
```
This opens the rqt console, now by selecting the logger level we can see the response of the nodes.

## Calling the service
If the service is not called then the default string is passed, but if we call this service the string passed by the talker node can be changed

First launch the demo from either of the above mentioned methods, after that do the following:
Open a new terminal and give the following commands
```
cd ~/catkin_ws
source ./devel/setup.bash
rosservice call /change_string <"any_string">
```

