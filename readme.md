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
## To view the tf frames
The tf broadcaster in the talker node will create a tf transform between the "world" frame and the "talk" frame. To inspec the frames please follow the following steps.

Open a new terminal and give the following commands
```
cd ~/catkin_ws
source ./devel/setup.bash
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
rosrun tf tf_echo /world /talk
```
After doing this, you will get a similar output
```
sayan@sayan-Aspire-V3-574G:~$ cd ~/catkin_ws
sayan@sayan-Aspire-V3-574G:~/catkin_ws$ source ./devel/setup.bash
sayan@sayan-Aspire-V3-574G:~/catkin_ws$ rosrun tf tf_echo /world /talk
At time 1573064812.438
- Translation: [5.000, 10.000, 15.000]
- Rotation: in Quaternion [0.382, 0.596, -0.381, 0.595]
            in RPY (radian) [3.140, 1.570, 2.000]
            in RPY (degree) [179.909, 89.954, 114.592]
At time 1573064813.139
- Translation: [5.000, 10.000, 15.000]
- Rotation: in Quaternion [0.382, 0.596, -0.381, 0.595]
            in RPY (radian) [3.140, 1.570, 2.000]
            in RPY (degree) [179.909, 89.954, 114.592]
At time 1573064814.138
- Translation: [5.000, 10.000, 15.000]
- Rotation: in Quaternion [0.382, 0.596, -0.381, 0.595]
            in RPY (radian) [3.140, 1.570, 2.000]
            in RPY (degree) [179.909, 89.954, 114.592]
```
To visualize this in the graph format please open a new terminal and follow these commands
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun rqt_tf_tree rqt_tf_tree 
```
To generate the pdf usinf the view frame tool please open a new terminal and add these commands
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun tf view_frames
```
To see the generated pdf please enter the following command
```
evince frames.pdf
```
## Run gtest/rostest
There are 3 test cases writen for this behinner_tutorial- testServiceExistence, testServiceRun and testBroadcaster.
To compile and run the test, open a new terminal and enter the following commands
```
cd ~/catkin_ws
catkin_make run_tests_beginner_tutorials
```
After compilation please enter the following commands to launch thetest file
```
source devel/setup.bash
rostest beginner_tutorials test_talker.launch
```
you must get a similar output as the following
```
sayan@sayan-Aspire-V3-574G:~/catkin_ws$ rostest beginner_tutorials test_talker.launch
... logging to /home/sayan/.ros/log/rostest-sayan-Aspire-V3-574G-7934.log
[ROSUNIT] Outputting test results to /home/sayan/.ros/test_results/beginner_tutorials/rostest-test_test_talker.xml
[Testcase: testtest_talker] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-test_talker/testServiceExistence][passed]
[beginner_tutorials.rosunit-test_talker/testServiceRun][passed]
[beginner_tutorials.rosunit-test_talker/testBroadcaster][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 3
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/sayan/.ros/log/rostest-sayan-Aspire-V3-574G-7934.log
sayan@sayan-Aspire-V3-574G:~/catkin_ws$ 
```

