
Once you can drive your robot around autonomously, the next thing you're going to want to do is be able to pick things up and manipulate them. 

In todays lesson we are looking at how to teleoperate a robot arm. In this case we will operate the MACI robot arm. 

Controlling a robot arm is harder than you think. To describe the position of a robots gripper, you need to specify the angle of each of its joints. Think about your own arm, try grasping a cup and thinking about the angle of your shoulder, elbow wrist and fingers. You likely didn't think about it in terms of angles, but you nonetheless moved your joints to a particular angle. Now grab that cup and move it above your head. How did the angles of your joints change as you did that? Could you have gotten it to the same position but going through different angles of your joints? Finally, did you make sure that your arm dodged any obstacles in your environment? 

What seems easy for humans needs  algorithms to be written for robots. Fortunately the Robotics Operating System provides the `moveit2` library, which handles everything for us. We just need to tell the robot arm where to go. 

# Spin up the system 


- Open a terminal and run the command:  `docker run -p 6080:80 johnny555/scr:v2`
- Open a web browser: http://localhost:6080/vnc.html 



# Run the MACI Gazebo Task 

Now that VS Code is running in a dev container. Turn on the simulation by running `Ctr+Shift+P` -> "Run Task" -> "MACI Gazebo".

Now go to a web browser, type http://localhost:8080/vnc.html and hit connect. 

You should see a few screens, you can right click on the task bar at the bottom to maximise some of the windows. Lets have a look at what is running: 

### Gazebo

You should see a Gazebo simulation with a robot arm (named MACI). MACI is in an empty world. 

## RViz2 

You should see another window named RViz. This will have an image of the robot arm, and on the left a planning monitor. 

# Moving the arm 

To move the arm make sure the group "ur5" is selected in the drop down. 

There are a few ways to move the arm:

### 1. Pre-defined poses: 
Select Goal Pose as "Down" and then hit "plan and execute". You should see the robot arm move. 

### 2. Drag the visual handles

Click on the arrows and circles around the arms end effector and drag the arm visually. Now hit the "plan and execute" button to move it. 

### 3. Specify joint angles manually. 

You can click on the joints tab, this will show you a list of the robot arms joints. You can use the sliders to move them manually. Then click back to the previous tab and hit "plan and execute" to move the arm. 

# Moving the gripper 

To move the gripper, select the group "gripper" in the drop down. 

Unfortunately, the gripper can only be controlled by specifying the joint angles. Switch to the joints tab to change the gripper joints, and then switch back and hit "plan and execute" to move the gripper. 

### Collision Avoidance. 

Our MACI arm comes equipped with a depth sensor. We can use this sensor to help the arm avoid obstacles. 

1. Spawn a box in range of the arm, and in view of the depth sensor. You should see boxes in the RViz view when it is detected. 
2. Choose a goal pose such that the arms shortest path is through the box
3. Hit plan and execute and see the algorithm avoid the box. 

# Congratulations! 

You've just controlled a robot arm in simulation. All the code for this is now here, on your computer. Feel free to look around the code. You'll see in the `src/maci/launch` folder there are several files, the one we used today was `gazebo.launch.py`, feel free to open that and look around. 

We also configured the localization and mapping using parameters in `src/maci/config`. 

## Post your wins 

I'm a big believer in showing your work online. If you got the robot working, post about it on Social media such as X or LinkedIn and tag me. I'd love to see how you went, and you'll be surprised who you can meet online once you start talking about robotics! 

# Tomorrow's Robot

Tomorrow we will look at how to build your own robot using FreeCAD. I think this is an important step if you want to start creating your own robots, so I'll see you tomorrow!

# Ready for more? 

If you want to go deeper, and learn how the MACI robot works, simulate your own unique robot, and become a roboticist, check out my online course https://becomearoboticist.com. Where we spend several days diving into what makes this robot work. 

You'll also join a community of other roboticists, and hang out with me on weekly live calls as we build your robotics skills and credibility online. 