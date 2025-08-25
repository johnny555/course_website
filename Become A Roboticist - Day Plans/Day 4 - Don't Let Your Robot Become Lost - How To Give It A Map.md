
If you're building a self driving vehicle or a warehouse robot, you're going to want the robot to know where it is in the world. As a human we do this effortlessly, but for robots we need to implement systems to allow them to do it. 

In the early days of robotics, the system used for working out where the robot is in the world was seperate from the systems used to creating a map of the world. These are both hard problems in their own right, but a surprising result was that combining the mathematics of a robot discovering where it was in the world with the process of building a map of the world made both tasks easier. This is the key idea of the Simultaneous Localization And Mapping concept, which we will use today. 

But don't worry, you don't need to know the maths just yet, we can configure it without it. 

# Don't Use The Latest Thing

SLAM is a vibrant field of research, with lots and lots of competing solutions. A common mistake beginners make is to try and use the most recent, complex SLAM solution out there. These systems usually take advantage of rich sensor data, such as 3D lidars or depth cameras. But in lots of cases, you can get away with a simple SLAM system using a 2D scanning lidar. That is what we will configure today. 

# Using And Configuring A Map

1. Run Mapping task (Ctl+Shift+P, Run task, Mapping)
2. Teleoperate to drive the robot around 
3. Add the map in RViz
4. Look at the mapping.yaml config

# Launch Files 

Today is the first time we have used a different launch file. These launch files build upon each other, so its worth taking a moment to look at the two we have used.

The way I like to read launch files is to look for the python function `generate_launch_description`. This is what is called when you launch the file. 

I then look to the very end of the function, to see what is inside the `LaunchDescription`. This is the name of all the things that the launch file will start. I then walk back up the function to see and understand each part. 

## Examine `src/bar_examples/krytn/gazebo.launch.py`

This file starts gazebo, sets up the robot controllers and basic topic publishers. It also creates a bridge between the Gazebo simulation and the ROS system to transmit sensor data.

You'll see that it creates the robot by reading a file `src/bar_examples/krytn/robot_description/krytn.urdf.xacro`. This is an important file that describes the physical setup of the robot. We will look into this a bit later when we add a link to Krytn. 

## Examine `src/bar_examples/krytn/mapping.lanch.py`

The biggest thing to note here is that this file includes the previous `gazebo.launch.py` file we looked at earlier. This means that we can easily build complex systems by chaining launch files together.

You'll notice that the `slam_toolbox` refers to the `mapping.yaml` file that we looked at earlier as part of its setup. 

# Q&A

**My map looks weird, what happened?** 

`slam_toolbox` depends upon good vehicle odometry coming from your differential drive control system. If you run into a wall but keep driving forward, it will assume that its sensor readings are valid further into the wall, and that the wall has just moved further back. This results in an inconsistent map. 

**Can I save the map and just localize?**

Yes, run the save map service, and the adjust the mapping.yaml file to set mode to `localization` instead of `mapping`.


# Day 4 - Video 

https://youtu.be/dl19ve-30XU