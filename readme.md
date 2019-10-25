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
