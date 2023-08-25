# _Multi-robot system with Turtlebot3_Burger and Kuka_Iiwa_

Simulation in an industrial environment in which Turtlebot must search for a tool represented by an aruco marker, and then bring it to the Kuka Iiwa. The second robot simulates the picking of the object and places it on another marker on the ground.

## Features

- The project shows the modularity of a multi-robot system
- Example of a service behaviour
- Insight on the usage of sensors 

## Getting Started (pre-requirements)

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11

## Packages

This project uses some external packages provided by the University Course of Robotics Lab and the _aruco-ros_ package. 

## Install the packages

To install all the needed packages to run the simulation it is enough to clone this repository in the _src_ folder of your ros workspace and compile the packages with the following instructions:

```sh
$ git clone https://github.com/MartinaAnn/Repo_Exam.git
$ cd ..
$ catkin_make
```
## Prepare and run the simulation

On different terminals launch the following commands:

- Start the roscore:

```sh
$ roscore
```

- Launch the main node:

```sh
$ roslaunch main_folder spawn_robots_and_world.launch
```

- Launch aruco node for the detection of the markers:

```sh
$ roslaunch aruco_ros all_aruco.launch
```

- Start the server node:

```sh
$ rosrun main_folder kuka_invkin_ctrl
```

- Start the client node: 

```sh
$ rosrun main_folder move_and_search
```

If you want to visualize the images on the camera sensors you can use the tool from ROS:

```sh
$ rqt_image_view
```

## Output on image_view

- /aruco_marker_publisher/result: the topic on which the images from the turtlebot camera are streamed;
- /simple_single/result: the topic on which the images from the kuka iiwa camera are streamed.

## Outline

![Client terminal results](/Images_for_readme/finalturtle.png) 
![Server terminal results](/Images_for_readme/finalkuka.png) 
![Gazebo results](/Images_for_readme/final.png) 


