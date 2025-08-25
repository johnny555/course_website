Now we have a fully functioning robot arm, with gripper, it's time to do something with it. Right now it just sits in a void. 
## Don't Skip The Most Important Step In Robotics

When learning robotics, it often feels so hard to get to the robot working stage (which we are at right now), that we tend to skip the "make a useful robot stage". Don't skip this step! Today we will quickly mock up a table world and add our robot into it. 

## Action steps 

1. Run your packages gazebo.launch.py 
2. Open the `resource spawner` and search for `table`, add a table to the world. 
3. Now search for `soda` and add a soda to the world. 
4. Save this world, click File-> save as, and save the world (don't forget to end the world with `.sdf`)
5. Edit the world file, and remove the robot arm.
6. Modify your gazebo.launch.py to launch this world instead of an empty world. 
7. Copy the Realsense code from the maci urdf.xacro and add it to `my_robot_arm.urdf.xacro` (make sure to copy the include and the macro) as well. 
3. Add gazebo bridge entries to bridge the vision topics into ROS, also add the static tf publishers to correct the gazebo tf names. Copy this from maci project. 
8. Use MoveIT interface to control the arm to pickup the can and place it somewhere else on the table. Use RViz to enable the point cloud to improve your teleoperation capabilities. 
9. Bonus Points: Create a python script to do it. 

##  Wait... Aren't we just doing glorified teleoperation?  

Astute students might notice that we are still doing a form of teleoperation here. We are recording joint angles and having the robot replay them. Although we won't get into full perception pipelines for pick and place in this course, come back tomorrow where we will look at adding collision avoidance to the robot with the depth camera. 

Once the robot can perceive the environment, we are able to create a much more autonomous robot. 

## Help! My Simulation Runs Slow!

Adding in lots of objects can cause the Gazebo simulation to run slow. You could remove some objects or try and simplify the objects collision boxes. You could even try to recreate an object in FreeCAD but with a simpler mesh. 

## Their Task + Walkthrough Video 

https://youtu.be/O-Y-D_8Js5A