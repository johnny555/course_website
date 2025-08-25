
If you're building an RC car, you probably don't need to visualize any of its sensors as it will always be in your line of sight. But if you want to create a robot that works autonomously, you are going to need to give it sensors. And once you give it sensors, you're going to need a way to test and check that those sensor make sense in the context of your robot. 

What if you accidentally install the sensor upside down, or in wrong place? 

# The Command Line Isn't Everything

A big mistake that beginners can make is to try to just depend on the command line for understanding your sensor data. After all, yesterday we learnt how to use the `ros2` command to echo a topic and listen to its data. The problem is that the data is more interesting when its combined, a raw laser scan is not as interesting or as useful as when its overlayed ontop of a picture. 

So we are going to use RViz to show the robot and its data. Lets see if  we can use its sensor data to drive our robot.

Action steps 

1. Start Krytn Teleoperate (Ctr+Shift+P, Run Task, Krytn Teleoperate)
2. Open a VS Code terminal and run `ros2 run rviz2 rviz2`
3. Change the "fixed frame" from `map` to `base_link`
4. Add the robot model, and change its topic to use `robot_description`
5. Add add the lidar and the depth camera, using the "By Topic" tab. 
6. Use the rqt interface to drive the robot, but looking only through the sensor visualisations. 

# Common Stumbling Blocks

Alot of students when attempting this task skip the first step and wonder why they can't see anything in their RViz. Make sure to start the Krytn Teleoperate task first, otherwise there really is nothing to show! 

# Day 3 Video

https://youtu.be/EpyGE_nUgu8