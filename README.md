# Cbot
The C-Bot is an unmanned underwater vehicle developed by the Marine Instrumentation Department, CSIR-National Institute of Oceaography, Dona Paula, Goa.

# ROS-CBOT
This repository contains the software to be used on CBOT. It contains the algorithms for control, guidance and mission planning.

## Pre-requisites
This assumes that you have ROS-Kinetic or ROS-Melodic installed on your machine. 

You should have the following packages installed:
1. UUV-Simulator (provide link)
2. CBOT-Gazebo (provide link)
3. ros_rtsp (provide link)
4. CBOT-GCS (provide link)

## Installation instructions
Create a workspace:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin init  # initialize your catkin workspace
```
The src should also contain the previously mentioned packages.

Clone this repository:
```
$ git clone https://github.com/MohitGupta007/ROS-Cbot.git
```
