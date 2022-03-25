Assignment 3 - Final Assignment
==================================

This is the third and final assignment of the course Research Track 1, provided by Università Degli Studi di Genova, Robotics Engineering degree.

The simulation includes a robot equipped with a laser scanner placed inside an environment in which there are obstacles. 

The objective of the project is to develop an architecture that can get the user request, and let the robot execute one of the following behaviors:
* Autonomously reach the (x,y) coordinate inserted by the user
* Let the user drive the robot with the keyboard
* Let the user drive the robot with the keyboard by assisting him to avoid collisions

The simulation environment seen in Gazebo is the following:
![simulation_environment](https://github.com/FraFerrazzi/final_assignment/blob/noetic/images/Schermata%202022-02-01%20alle%2020.54.15.png)

It can be seen the same simulation environment in Rviz, which is the following:
![rviz_environment](https://github.com/FraFerrazzi/final_assignment/blob/noetic/images/Schermata%202022-02-01%20alle%2021.35.05.png)

This solution is developed by: Francesco Ferrazzi 5262829.

Table of Contents
----------------------

- [Assignemnt 3 - Final Assignment](#assignemnt-3---final-assignment)
 * [Table of Contents](#table-of-contents)
 * [Installing and running](#installing-and-running)
 * [Project Description](#project-description)
 * [Pseudocode](#pseudocode)
 * [Possible Improvements](#possible-improvements)

Installing and running
----------------------

The simulator requires a [ROS Noetic](http://wiki.ros.org/noetic/Installation) installation and the following packages if they are not already installed:

* install teleop twist keyboard package by typing on terminal:
```bash
$ sudo apt-get install ros-noetic-teleop-twist-keyboard
```
* install the ros navigation stack by typing on terminal:
```bash
$ sudo apt-get install ros-noetic-navigation
```
* install xterm package by typing on terminal:
```bash
$ sudo apt-get install xterm
```
* to run the program is sufficient to clone the GitHub repository using the following commands:
```bash
$ git clone https://github.com/CarmineD8/slam_gmapping.git
```
```bash
$ git clone https://github.com/FraFerrazzi/final_assignment.git
```
Remember to switch on the correct branch (noetic) for both projects using:
```bash
$ git checkout noetic
```
* RUN THE PROGRAM typing on terminal:
```bash
$ roslaunch final_assignment final_assignment.launch
```

Project Description
-------------------

The objective of the assignment is to make the robot go around the environment allowing the user to choose between the driving modality specified in the introduction of the project.

In addition to let the robot drive autonomously towards the goal, drive it manually with and without driving assistance, I implemented also the following behaviors:
* cancel the goal whenever the user wants
* reset the position of the robot in the environment 
* shutdown the program 

To implement the solution, the node `main_ui` was implemented.
The structure of the project is the following:
![code_structure](https://github.com/FraFerrazzi/final_assignment/blob/noetic/images/Schermata%202022-02-01%20alle%2022.16.38.png)

`main_ui` node publishes on `gazebo` and `move_base` nodes. `gazebo` sends back to the `main_ui` node the information about the environment using the `scan` topic.

`main_ui` publishes the goal using the `move_base/goal` topic. 

To allow the user to control manually the robot the `teleop` node is used. The topic `cmd_vel` is remapped on the topic `us_cmd_vel` which is defined by me to let the `main_ui` node control if the velocity given by the user should be published, changed, or ignored. 

Pseudocode
-----------------

The general behavior of the code:
```
while main_ui node is running
while rospy not on shutdown
 print menu
 get user input
 if user input is 1:
 set_goal()
 if goal has been set:
 print_goal()
 
 elif user input is 2:
 manual mode active
 drive assistance not active
 manual_driving()

 elif user input is 3:
 manual mode active 
 drive assistance active
 manual_driving()

 elif user input is 4:
 cancel_goal()

 elif user input is 9:
 reset_world()

 elif user input is 0:
 rospy is shutdown

 else:
 user input not valid
```

The function `set_goal()` let the user insert the desired goal coordinates (x,y) and publish them in the `move_base/goal` topic.

The function `manual_mode()` for both cases `2` and `3` doesn't allow the user to define a new goal and use the normal menu. The function waits for the user's input given from the keyboard. If the command given is `b`, exit from the manual mode.

The difference between cases `2` and `3` is that the first one does not use the `assisted_driving()` function which uses the `scan` topic to get data from the Laser Scan and helps the user in driving the robot.

The pseudocode of `assisted_driving()` function is:
```
assisted_driving(msg)
 get minimum distance from obstacles on the right of the robot
 get minimum distance from obstacles on the front right of the robot
 get minimum distance from obstacles in front of the robot
 get minimum distance from obstacles on the front left of the robot
 get minimum distance from obstacles on the left of the robot

 if minimum distance from obstacle in front of the robot < threshold:
 if vel_msg linear > 0 and vel_msg angular == 0:
 set linear velocity to 0

 if minimum distance from obstacle on the front-right of the robot < threshold:
 if vel_msg linear > 0 and vel_msg angular < 0:
 set linear and angular velocity to 0

 if minimum distance from obstacle on the front left-right of the robot < threshold:
 if vel_msg linear > 0 and vel_msg angular > 0:
 set linear and angular velocity to 0

 if minimum distance from obstacle on the right of the robot < threshold:
 if vel_msg linear == 0 and vel_msg angular < 0:
 set linear and angular velocity to 0

 if minimum distance from obstacle on the right of the robot < threshold:
 if vel_msg linear == 0 and vel_msg angular > 0:
 set linear and angular velocity to 0
 
 publish the updated velocity
```
where `msg` is the message coming from the laser scanner and `vel_msg` is the velocity of the robot imposed by the user using the `Twist` message.

The function `cancel_goal()` checks if exists a goal. If a goal has been set, publish an empty message on the `move_base/cancel` topic. If the goal has not been set, tell the user that no goal was given. 


Possible Improvements
------------------

There are three main improvements that I came up with, which are:
* Make the robot know a priori if a position in the environment is reachable or not
* Allow the user to set goals sequentially and allow the robot to reach them one after the other
* Develop a better code structure separating the user interface and the rest of the code using services.