Welcome to day 2 of start creating robots course! 

If you put a human on any street and they will quickly start working out where to go and how to avoid obstacles. 

But put a robot on that same street and its likely that it will run into stuff or get lost. The robot needs an algorithm to look at its surroundings and create an internal Map. It needs to figure out where it is in this map, and then it needs to know where it needs to go in that map. It needs an algorithm to work out how to get to its destination, and it needs another one to work out how to avoid any obstacles along the way. 

Sheesh! That's a lot of algorithms. Fortunately, we can take advantage of the Robotics Operating System ecosystem to provide them all. We will use the package `slam_toolbox` to create a map of the world and figure out the robots place in it. We will use `RViz` to visualise the map and let a user specify the location for it to go. Finally we will use `nav2` to help the robot decide what path to take and how to avoid any obstacles that might appear in its path. 


# Spin up the system 

- Open a terminal and run the command:  `docker run -p 6080:80 johnny555/scr:v2`
- Open a web browser: http://localhost:6080/vnc.html 


# Run The Navigation Task

Open VS Codium, and turn on the simulation by running `Ctr+Shift+P` -> "Run Task" -> "Krytn Navigation".


You should see a few screens, you can right click on the task bar at the bottom to maximise some of the windows. Lets have a look at what is running: 

### Gazebo

You should see a Gazebo simulation with a small blue robot (named Krytn). Krytn is in a simulated coffee environment. 

## RViz2 

You should see another window named RViz. This will have a map and also display the simulated sensors. You can turn on and off the visualisations using the panel on the left, and it should be centred on Krytn. Keep all the visualisations on for now. 


# Autonomous Operation 

In the RViz window, click the "Set Goal Pose" button (with a green arrow along the top). 

Now click anywhere on the map, holding down to select the orientation that the robot will go to. You should see the robot plan its path and start moving towards the goal. 


### Collision Avoidance. 

We can play with collision avoidance by spawning obstacles for the robot. 

Give Krytn a goal pose on the other side of the coffee shop. Now go to Gazebo and click the cube icon in the toolbar on the top left. This will spawn a cube at the point you select. Place a cube in the path of Krytn and see it plan around it. 


# Congratulations! 

You've just run an autonomous robot in simulation. All the code for this is now here, on your computer. Feel free to look around the code. You'll see in the `src/krytn/launch` folder there are several files, the one we used today was `navigation.launch.py`, feel free to open that and look around. 

We also configured the localization and mapping using parameters in `src/krytn/config`. 

## Post your wins 

I'm a big believer in showing your work online. If you got the robot working, post about it on Social media such as X or LinkedIn and tag me. I'd love to see how you went, and you'll be surprised who you can meet online once you start talking about robotics! 

# Tomorrow's Robot

Tomorrow we will look at the MACI robot arm. This robot arm is in use at a Coffee Shop in Perth Australia. We will use a package called `moveit2` to control the arm. 

# Ready for more? 

If you want to go deeper, and learn how the Krytn robot works, simulate your own unique robot, and become a roboticist, check out my online course https://becomearoboticist.com. Where we spend several days diving into what makes this robot work. 

You'll also join a community of other roboticists, and hang out with me on weekly live calls as we build your robotics skills and credibility online. 