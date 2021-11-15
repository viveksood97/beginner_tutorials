# ROS Beginner Tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-Default.svg)](https://opensource.org/licenses/MIT)

# Overview
Project is a part of 808X coursework wherein the aim is to get familiarized with the basic concepts of ROS.

Task 1: Create a publisher that publishes a custom string message which is then subscribed by the subscriber.

Task 2: Write a service that modiefies the current string . Also write launch files with an argument.


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
catkin_make
```
## Run
1. Launch using launch file without string argument
```
cd ~/catkin_ws/
source ~/catkin_ws/devel/setup.bash
roslaunch beginner_tutorials launchPubSub.launch
```
2. Launch using launch file with string argument
```
cd ~/catkin_ws/
source ~/catkin_ws/devel/setup.bash
roslaunch beginner_tutorials launchPubSub.launch newString:=anyString
```
3. To change the string use the service: /change_string
```
cd ~/catkin_ws/
source ~/catkin_ws/devel/setup.bash
rosservice call /change_string AnotherString
```
4. Launch using launch file with rosbag recording on
```
cd ~/catkin_ws/
source ~/catkin_ws/devel/setup.bash
roslaunch beginner_tutorials launchPubSub.launch recordbag:=true
```
## Output
The rqt_console and rqt_logger_level output
![](output.jpg)

### Inspecting TF Frames
After launching both nodes using launch file, inspect the TF frames using tf_echo and rqt_tf_tree (Open a new Terminal)

```
rosrun tf tf_echo /world /talk
```
To genereate a pdf of tf frame
```
rosrun tf view_frames
```
To view the tf tree
```
rosrun rqt_tf_tree rqt_tf_tree
```
### Run Test
Open a new Terminal
```
cd ~/catkin_ws
source ./devel/setup.bash
rostest beginner_tutorials launchTest.launch
```

## Run cppcheck and cpplint
Run cppcheck: Results are stored in `./results/cppcheck.txt` 
```
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp) > results/cppcheck.txt 2>&1
```

Run cpplint: Results are stored in `./results/cpplint.txt`
```
cpplint $( find . -name \*.hpp -or -name \*.cpp) > results/cpplint.txt 2>&1
```
