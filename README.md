Assignment 1 - Research Track 2
==================================

This is the first assignment of the course Research Track 2, provided by Università Degli Studi di Genova, Robotics Engineering degree.

The assignment is divided into three parts, which are:
* Properly comment the third assignment of the Research Track 1 course. In addition, also the new node made for the Research Track 2 course was properly commented using sphinx.
* Create a Jupyter notebook to interact with the simulation on the third assignment able to:
   * Switch between the different modalities, and manage them.
   * Plot the robot position, the laser scanner data and reached / not-reached targets.
   
  To achieve the wanted result, not just the notebook was implemented, but also a new node named `set_mode`. The node is very similar to the `main_ui` node already created for RT1. The only difference is that services and custom messages are defined to allow the communication between the ROS node and the Jupyter notebook.
* Perform a statistical analysis on the first assignment, considering two different implementations and testing which one performs betterin the circuit given, when silver tokens are randomly placed in the environment. As performance evaluator the average time required to finish the circuit was considered. The two implementations are: 
   * [My implementation](https://github.com/FraFerrazzi/Assignment-1---Research-Track.git)
   * [Professor's implementation](https://github.com/CarmineD8/python_simulator.git)

The simulation is the same as the Research Track 1 final assignment, which includes a robot equipped with a laser scanner placed inside an environment. 

The objective of the project is to develop an architecture that can get the user request, and let the robot execute one of the following behaviors:
* Autonomously reach the (x,y) coordinate inserted by the user
* Let the user drive the robot with the keyboard
* Let the user drive the robot with the keyboard by assisting him to avoid collisions
* In addition, two commands to cancel the goal and to reset the simulation environment are implemented.
* Also a way to check the status of the goal is implemented.

The simulation environment seen in Gazebo is the following:
![simulation_environment](https://github.com/FraFerrazzi/final_assignment/blob/noetic/images/Schermata%202022-02-01%20alle%2020.54.15.png)

It can be seen the same simulation environment in Rviz, which is the following:
![rviz_environment](https://github.com/FraFerrazzi/final_assignment/blob/noetic/images/Schermata%202022-02-01%20alle%2021.35.05.png)

To see the [Sphinx Documentation](#sphinx-documentation) of the `set_mode` node and the `main_ui` go to the bottom of the page.

To see the statistical analyisis go in the `statistics` directory placed in the rt2_first_assignment repository.

This solution is developed by: Francesco Ferrazzi 5262829.

Table of Contents
----------------------

- [Assignemnt 3 - Final Assignment](#assignemnt-3---final-assignment)
 * [Table of Contents](#table-of-contents)
 * [Installing and running](#installing-and-running)
 * [Project Description](#project-description)
 * [Pseudocode](#pseudocode)
 * [Possible Improvements](#possible-improvements)
 * [Sphinx Documentation](#sphinx-documentation) 

Installing and running
----------------------

This instructions are made to explain how to run the second part of the assignment, which is the one regarding the simulation. 

The simulator requires a [ROS Noetic](http://wiki.ros.org/noetic/Installation) installation and the following packages if they are not already installed:

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
$ git clone https://github.com/FraFerrazzi/rt2_first_assignment.git
```
* Remember to clone the repository into a ros like workspace and use the command:
```bash
$ catkin_make
```
 to build the project.
* RUN THE PROGRAM typing on terminal:
```bash
$ roslaunch rt2_first_assignment rt2_first_assignment.launch
```
* It is also necessary to launch the Jupyter notebook node, by going in the `Kernel` menu and selecting `Restart & Run All` after typing on terminal:
```bash
$ jupyter notebook --allow-root --ip 0.0.0.0
```

Project Description
-------------------

The project description and the pseudocode sections were mae for the Research Track 1 final assignment. Since the new node `set_mode` is very similar to the old one i suggest to read the sections anyways.

To see a more detailed documentation of the new node, go to the bottom of the page.

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


Sphinx Documentation
-------------------

This is part of the Research Track 2 assignment.

To have a look at the full documentation of the code click on the following link:

https://fraferrazzi.github.io/rt2_first_assignment/
