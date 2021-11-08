# ROS Beginner Tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-Default.svg)](https://opensource.org/licenses/MIT)

# Overview
Project is a part of 808X coursework wherein the aim is to get familiarized with the basic concepts of ROS.
Task 1: Create a publisher that publishes a custom string message which is then subscribed by the subscriber.

# Dependencies
- Ubuntu 20.04
- ROS Noetic

## Building package via command-line
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone https://github.com/viveksood97/beginner_tutorials
cd ../
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
catkin build
```
## Run
1. Launch both nodes Individually
Run roscore (Open a new Terminal)
```
cd ~/catkin_ws/
source /opt/ros/noetic/setup.bash
roscore
```
Run talker node (Open a new Terminal)
```
cd ~/catkin_ws/
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials talker
```
Run listener node (Open a new Terminal)
```
cd ~/catkin_ws/
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials listener
```
## Run cppcheck and cpplint
Run cppcheck: Results are stored in `./results/cppcheck.txt` 
```
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./lib") > results/cppcheck.txt 2>&1
```

Run cpplint: Results are stored in `./results/cpplint.txt`
```
cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" -e "^./lib/") > results/cpplint.txt 2>&1
```
