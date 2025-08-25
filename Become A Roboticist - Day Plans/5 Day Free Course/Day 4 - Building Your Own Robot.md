Welcome to Day 4 Start Creating Robots, today its time to build your own robot. 

There is alot that goes into building a robot. You need the mechanical design to be able to structurally hold the payload, as well as transmit the mechanical power to the wheels or joints using motors and gears. You need the electrical design to make sure enough power is available to drive the motors, as well as the onboard compute and sensors. Finally you need the robot to be able to actually do something useful as well, and this means it needs to be a particular shape or fit into a particular environment. 

In todays lesson we won't dive too deep into the mechanical or electrical requirements of a robot. Rather we will look at creating the broad shapes that we want to solve our particular problem, and let the actual implementation be done later. This is a proof of concept simulation, designed to help prove out the software and the concept of the robot, and give you an experience designing your own! 

# Spin up the system 


- Open a terminal and run the command:  `docker run -p 6080:80 johnny555/scr:v2`
- Open a web browser: http://localhost:6080/vnc.html 


# Open FreeCAD 

Now that VS Code is running in a dev container. Turn on the simulation by running `Ctr+Shift+P` -> "Run Task" -> "freecad".

Now go to a web browser, type http://localhost:8080/vnc.html and hit connect. 

You should see a few screens, you can right click on the task bar at the bottom to maximise some of the windows. Lets have a look at what is running: 

# Experiment with FreeCAD 

FreeCAD, and the FreeCAD Cross Workbench, allow beginner roboticists to start creating models. In this example we have a 2 wheel robot, with some text on its body.

In the left pane find the element called "Robot Name". Double click it and the change the field String which contains "My Robot", to your name. Hit ok. 

You should now see your name embossed on the robots body. 

Scroll down in the pane until you see the "Robot" element. Click that, change the workbench to "CROSS - Ros Workbench" to show the export to URDF button. Click that button, check the select all checkbox and hit generate.

This will generate a robot model with your name on it. If you know CAD, feel free to modify the shape of the body to make it yours. 


# Drive your robot 

Close FreeCAD and go back to VS Code. 

Now run  `Ctr+Shift+P` -> "Run Task" -> "Simulate the first robot". 

Go back to the web browser, and you should see a little robot with your name on it in the Gazebo simulation. You should also see another window named rqt_robot_steering with two sliders. 

Move the sliders to drive the robot around. 

# Congratulations! 

You've just run an autonomous robot in simulation. All the code for this is now here, on your computer. Feel free to use FreeCAD to modify your robots links and keep generating different robot shapes. 

You now know how to create your own robot! 

## Post your wins 

I'm a big believer in showing your work online. If you got the robot working, post about it on Social media such as X or LinkedIn and tag me. I'd love to see how you went, and you'll be surprised who you can meet online once you start talking about robotics! 

# Tomorrow's Simulation

Tomorrow we will look at how to build out better simulations using Gazebo. Because a robot is only half the story, you also need to build an environment for it to work in. 

# Ready for more? 

If you want to go deeper, and learn how building a robot with FreeCAD works and become a roboticist, check out my online course https://becomearoboticist.com. 


You'll also join a community of other roboticists, and hang out with me on weekly live calls as we build your robotics skills and credibility online. 