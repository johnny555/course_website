
Being able to move from place to place is important, but usually you need to pick up or manipulate objects when you get there. This leads us to the use of Robot Arms. Just like the Krytn robot we will begin with a form of teleoperation, and build complexity from there. 

Teleoperating a robot arm is much more complex than just driving wheels. Robot arms have many more motors and joints, and moving from one pose to another can often require sending control signals to every joints simultaneously. While checking for self-collisions along the way. To make it easy we are going to use the ROS 2 package, MoveIT2. This package will calculate all the trajectories that are necessary to get the robot to move. 

## Use Vendor Models

Beginner roboticist often want to build everything from scratch. When it comes to robot arms, this is possible, but it is a huge time sink. Rather than building from scratch, you can buy robot arms off the shelf, and configure them with MoveIT to save yourself time. 
## Action steps 

1. Run "MACI Gazebo" vs code task (ctrl+shift+P run task, "Maci Gazebo")
2. Look at the RViz interface, move the robot arm, and then click the "Plan and Execute" button. 
3. Watch the arm move.
4. Switch the planner group from "arm" to "gripper". 
5. Change the tab to the joints tab 
6. Adjust the joints of the gripper 
7. Hit Plan and execute. 
8. Watch the gripper move

##  Why Does The Gripper Have So Many Joints? 

One thing that is unusual about the gripper on MACI is the 4 joints we have to control. This is due to a problem with Gazebo that won't allow us to model mimic joints. A mimic joint is a joint that will copy the value of another joint. This actual gripper is designed so that you can control one joint, and all the other joints will follow. 

(A recent Gazebo update added mimic joints, but in a way that removed all the collisions. A gripper must collide with things in order to grip them, so it is pointless.  We will treat Gazebo like it can't do mimic joints. )

## Video Guide

https://youtu.be/Qts-iC2fEz0

