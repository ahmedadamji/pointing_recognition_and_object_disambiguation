
# Pointing Recognition
----------------- 

Welcome to the [**Pointing Based Object Recognition and Disambiguation for Autonomous Service Robots Repository**](https://gitlab.com/el17ana/pointing_recognition)! Our system, designed for the PAL Robotics' TIAGo robot, focuses on enabling robots to understand and act upon human pointing gestures to achieve joint attention and facilitate natural, effective communication. 

It is authored by [**Ahmed Adamjee**](https://www.linkedin.com/in/ahmedadamjee/) and is licensed under the [MIT License](https://gitlab.com/el17ana/pointing_recognition/-/blob/master/LICENSE).
Please contact the author via the E-Mail IDs provided below regarding any requests relating to the topic:
Email: [el17ana@leeds.ac.uk](mailto:el17ana@leeds.ac.uk); [adamjiahmed@gmail.com](mailto:adamjiahmed@gmail.com)


## Watch The Presentation at RoboCup@Home 2021

[![LASR
](http://i3.ytimg.com/vi/-IYFWRORLpc/hqdefault.jpg)](https://youtu.be/-IYFWRORLpc)

## Testing Videos

[![Pointing Based Object Recognition and Disambiguation for Autonomous Service Robots - Test Videos
](http://i3.ytimg.com/vi/1f78WHAk3IQ/hqdefault.jpg)](https://youtu.be/1f78WHAk3IQ)


## Introduction

This project addresses the growing demand for domestic service robots that can assist humans in their daily lives, particularly in household tasks and monitoring people in need. The ongoing demographic shift towards an aging population and the increased acceptance of service robots due to the COVID-19 pandemic have further motivated the need for seamless and intuitive Human-Robot Interaction (HRI).

This project combines various technologies, including Gesture Recognition, Computer Vision, Object Recognition, and Autonomous Navigation, to meet the objectives of competitions like RoboCup@Home and the goals of the Leeds Autonomous Service Robots (LASR) team.

The primary aims of this project are to develop a robotic system capable of identifying objects pointed at in a domestic setting and disambiguating between closely placed objects by asking appropriate questions based on their attributes. The system should also be able to recognize and respond to multiple modes of human communication, such as speech and hand gestures.

Our repository includes detailed information on the following objectives:

 - Developing a pointing application for the specified robot hardware and use-case, as well as classifying whether a person is pointing and accurately determining the target location.
 - Disambiguating between objects found in the identified region, using object attributes and developing an approach to differentiate between objects of the same and different classes, and identifying their unique features.
 - Producing a capable system that can interact with humans using multiple modes of communication, such as speech or hand gestures, and incorporating a module for communication.
 - Analyzing and evaluating the workability and effectiveness of the system.
 
### Additional Techniques and Tools

To support the completion of this project, we have employed several cutting-edge techniques and tools, such as:

 - Training image classification using YOLOv4 to improve object recognition and identification.
 - Implementing SLAM-based Occupancy Grid Mapping for efficient robot navigation within the environment.
 - Creating 3D models using Blender for realistic simulations and incorporating them into Gazebo with assigned physics for accurate testing and evaluation.

Feel free to explore the repository, learn about our approach, and contribute to this groundbreaking project in human-robot interaction!

## Installation

For users part of the University of Leeds, please request access to the [LASR](https://web.yammer.com/main/groups/eyJfdHlwZSI6Ikdyb3VwIiwiaWQiOiIxNjA0NDI1NyJ9/all) Yammer page first, and then follow [this](https://gitlab.com/el17ana/pointing_recognition/install_tiago_container) guide to install the Tiago container, which consists of most of the required software and libraries pre installed, including Ubuntu 16.04, ROS Kinetic, OpenCV, etc.

For users outside the University of Leeds, these packages need to be installed sperately. These will have to be installed seperately. After you install ROS you can then proceed to install TIAGo simulation using [this](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/TiagoSimulation) guide, following the instructions for ROS Kinetic which shold contain most of the required packages for this project to run.

You will have to request access to the [sensible robotics](https://gitlab.com/sensible-robots) gitalb page to install relevant support packages for [object recognition](https://gitlab.com/sensible-robots/lasr_object_detection_yolo) and [configured test environments](https://gitlab.com/sensible-robots/custom_worlds/-/tree/for_pointing_recognition/).

For users using the Tiago Singularity Container, you need to install python packages by shelling into the writable workspace of the Tiago container by typing `./write.sh`

To install OpenPose, please follow: [OpenPose Library - Compilation and Installation](https://github.com/tramper2/openpose/blob/master/doc/installation.md).

You can install MediaPipe by typing `pip install mediapipe` in your command line, please note that you need python 3.6 or higher to install this.


To install this project's package, clone the repository into the src folder of the workspace:
```
git clone https://gitlab.com/el17ana/pointing_recognition.git
```
return to the workspace and run this command:
```
catkin_make
```

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
