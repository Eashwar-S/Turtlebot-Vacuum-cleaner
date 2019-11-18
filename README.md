# Turtlebot-Vacuum-cleaner
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview:
This branch describes a walker algorithm which makes the turtlebot to move in Gazebo world without collision. This branch will also give an introduction to Gazebo.

## Assumptions:

1. The user must have Ubuntu 16.04 LTS version installed.
2. The user must have ROS kinetic and catkin installed.

## Executables Added:
In CMakeLists.txt the following executables are added to execute cpp files in src folder:
```
add_executable(walker src/walker.cpp)
```

## Target Link Libraries Added:
In CMakeLists.txt the following target link libraries to link executables:
```
target_link_libraries(walker ${catkin_LIBRARIES})
```
## Added C++11 Compile option:
This is done to take advantage of C++11 features
```
add_compile_options(-std=c++11)
```
## How to build
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/Eashwar-S/turtlebot_vacuum_cleaner.git
git checkout Week10_HW
cd ..
catkin_make
```
## How to run
```
cd catkin_ws
source devel/setup.bash
roslaunch turtlebot_vacuum_cleaner turtlebot_newworld.launch rosbagRecord:=true 
```
This will record all the topics for 15 seconds and store it in rosbagRecord.bag file in results folder

## How to playback .bag file
Open a new terminal; and run the following commands:
```
roscore
```
Open another terminal; and run the following commands:
```
cd catkin_ws
source devel/setup.bash
roscd turtlebot_vacuum_cleaner/results
rosbag play walkernodeRecord.bag
```
## How to disable recording of bag file
If the recording of bagfile is not needed run the following commands
```
cd catkin_ws
source devel/setup.bash
roslaunch turtlebot_vacuum_cleaner turtlebot_newworld.launch
```
By default rosbag recording will not happen as the argument rosbagRecord by default is set to false in launch file.
## SampleoOutput
In the terminal running rosbag
```
[ INFO] [1574105034.562631674]: Connected to master at [localhost:11311]
[ INFO] [1574105034.565606394]: Opening walkernodeRecord.bag

Waiting 0.2 seconds after advertising topics... done.

Hit space to toggle paused, or 's' to step.
 [DELAYED]  Bag Time:    346.770000   Duration: 0.000000 / 29.780000   Delay: 15 
 [RUNNING]  Bag Time:    346.770000   Duration: 0.000000 / 29.780000             
 [RUNNING]  Bag Time:    346.770000   Duration: 0.000000 / 29.780000             
 [RUNNING]  Bag Time:    346.771808   Duration: 0.001808 / 29.780000             
 [RUNNING]  Bag Time:    346.780230   Duration: 0.010230 / 29.780000             
 [RUNNING]  Bag Time:    346.790223   Duration: 0.020223 / 29.780000             
 [RUNNING]  Bag Time:    346.800187   Duration: 0.030187 / 29.780000             
 [RUNNING]  Bag Time:    346.810551   Duration: 0.040551 / 29.780000             
 [RUNNING]  Bag Time:    346.820295   Duration: 0.050295 / 29.780000             
 [RUNNING]  Bag Time:    346.830346   Duration: 0.060346 / 29.780000             
 [RUNNING]  Bag Time:    346.840273   Duration: 0.070273 / 29.780000             
 [RUNNING]  Bag Time:    346.850298   Duration: 0.080298 / 29.780000             
 [RUNNING]  Bag Time:    346.860245   Duration: 0.090245 / 29.780000             
 [RUNNING]  Bag Time:    346.870210   Duration: 0.100210 / 29.780000             
 .
 .
 .
 
```