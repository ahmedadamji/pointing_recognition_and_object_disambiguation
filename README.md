
# Pointing Recognition
----------------- 

[**Pointing Recognition**](https://gitlab.com/el17ana/pointing_recognition) is a project focused on **recognising objects being pointied at.**

It is authored by [**Ahmed Adamjee**](https://www.linkedin.com/in/ahmedadamjee/).

**This project would not be possible without the [**Leeds Autonomous Service Robots Team**](https://gitlab.com/sensible-robots). I would also like to thank [**Dr Matteo Leonetti**](https://eps.leeds.ac.uk/computing/staff/771/dr-matteo-leonetti).

  
  
  
  
  

<p  align="center">

<img  src=".github/media/pose_face_hands.gif",  width="480">

<br>

<sup>An Image and some description (Add something here)</a></sup>

</p>

  
  
  

## Contents

1.  [Results](#results)

2.  [Features](#features)

3.  [Installation](#installation)

4.  [Quick Start](#quick-start)

  
  
  

## Results

### Pointing

<p  align="center">

<img  src=".github/media/dance_foot.gif",  width="360">

<br>

<sup>An Image and some description (Add something here)</a></sup>

</p>

  

### Object recognition

<p  align="center">

<img  src=".github/media/openpose3d.gif",  width="360">

<br>

<sup>Testing the 3D Reconstruction Module of OpenPose</sup>

</p>

  

### HRI

<p  align="center">

<img  src=".github/media/pose_face.gif",  width="360">

<img  src=".github/media/pose_hands.gif",  width="360">

<br>

<sup>Testing the 3D Reconstruction Module of OpenPose</sup>

</p>

  

### Runtime Analysis

DESCRIPTION

<p  align="center">

<img  src=".github/media/openpose_vs_competition.png",  width="360">

</p>

  
  
  

## Features

-  **Main Functionality**:

-  **ABCD**:

- ABCD.

-  **Input**:

-  **Output**:

-  **OS**:

-  **Hardware compatibility**:

  
  

## Installation

For users part of the University of Leeds, please request access to the [LASR](https://web.yammer.com/main/groups/eyJfdHlwZSI6Ikdyb3VwIiwiaWQiOiIxNjA0NDI1NyJ9/all) Yammer page first, and then follow [this](https://gitlab.com/el17ana/pointing_recognition/install_tiago_container) guide to install the Tiago container, which consists of most of the required software and libraries pre installed, including Ubuntu 16.04, ROS Kinetic, OpenCV, etc.

For users outside the University of Leeds, these packages need to be installed sperately. These will have to be installed seperately. After you install ROS you can then proceed to install TIAGo simulation using [this](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/TiagoSimulation) guide, following the instructions for ROS Kinetic which shold contain most of the required packages for this project to run.

You will have to request access to the [sensible robotics](https://gitlab.com/sensible-robots) gitalb page to install relevant support packages for [object recognition](https://gitlab.com/sensible-robots/lasr_object_detection_yolo), [Custom Worlds](https://gitlab.com/sensible-robots/lasr_object_detection_yolo) and ...........

For users using the Tiago Singularity Container, you need to install python packages by shelling into the writable workspace of the Tiago container by typing `./write.sh`

To install OpenPose, please follow: 

You can install MediaPipe by typing `pip install mediapipe` in your command line, please note that you need python 3.6 or higher to install this.


To install this project's package, clone the repository into the src folder of the workspace:
```
git clone https://gitlab.com/el17ana/pointing_recognition.git
```
return to the workspace and run this command:
```
catkin_make
```
Before you can start, you will need to modify a file located in `project_src/src/cylinder_manipulation.py`
In line 18 change file link to contain correct full path to the `cylinder_detector.launch` file. `*path_to_workspace*/*workspace*/src/project_src/launch/cylinder_detector.launch`

## Quick Start

 - The world configurations for this project can be changed as required,
   by modifying the model, world and map paths inside  `/launch/gazebo_rviz.launch`
 - The furniture locations can be modified inside `/config/locations.yaml`
 - The objects being used for disambiguation and their attributes can be modified inside `/config/features.yaml`

To get started, you will need three separate command line terminals with the workspace initialised.
To initialise workspace navigate into the tiago-lib directory and run these commands:
```
./run.sh
source devel/setup.bash
```
In the first terminal, to launch the simulation run this command:
```
roslaunch pointing_recognition gazebo_rviz.launch
```
In the second terminal, to launch the necessary nodes, configurations and support modules run this command:
```
roslaunch pointing_recognition support_modules.launch
```
Finally, run this command in the third terminal, to start the demonstration:
```
rosrun pointing_recognition state_machine.launch
```