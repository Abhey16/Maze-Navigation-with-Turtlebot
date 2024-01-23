# Overview
This repository contains the source code and CAD files for a **Maze Navigation with Turtlebot**. This project focuses on maze navigation using the Waffle Turtlebot, incorporating Simultaneous Localization and Mapping (SLAM) for maze mapping. A C++ script within the ROS2 framework is developed, utilizing logical cameras to guide the Turtlebot through the maze to the specified goal.

## Installation
Follow these instructions to get a copy of the project up and running on your local machine for development and testing purposes.
1. Create a directory with a subfolder named **src** on your local machine.
2. Navigate into the directory using Terminal and run:
```
colcon build
```
This will convert your directory into a ROS2 Workspace

3. Navigate to the src folder and clone the repository to your local machine:
```
git clone https://github.com/Abhey16/Maze-Navigation-with-Turtlebot.git
```
This will add ROS package in the **src** folder.

4. Navigate to the root of ROS2 directory and build the project using colcon:
```
colcon build
```
5. Source the environment to set up the necessary ROS2 variables:
```
source install/setup.bash
```
6. Now launch the Gazebo environment
```
ros2 launch group23_final gazebo.launch.py
```
7. For running the script:
```
ros2 run group23_final navigator_23.cpp
```
