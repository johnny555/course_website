
# The Right Way To Drive A Robot

Ok, so you've got a robot, but its not doing anything. We need to be able to control it somehow. But what is a good interface to control a robot? If your robot is working with humans, perhaps it needs to read emotions or get some intent from body language? If its working on a factory floor, perhaps it needs to know its path and which shelf it needs to go to? 

But whatever your interface to your robot is, ultimately you will need to describe your commands in terms of fundamental physics, such as a direction of travel and an acceleration. Today we will do just that, with a simple GUI tool. It won't read your body language, but it is a straight forward way of giving your robot a command. 

This is the right way to drive your robot. 

# The Wrong Way To Drive A Robot

If you've got an engineering background, your first instinct might be to just give your wheels acceleration commands. This can take you down a rabbit hole of hardware level motor controllers, and motor selection, electronic design and power calculations. 

However, we are here to build robots, not electronics, so lets keep it really simple. 

# How To Drive A Robot

Our interface will be a desired velocity and an angular rotation to be applied to the entire robot base, rather than a particular wheel. This will make our system more general to any type of robot, with any number of wheels or even legs. 

1. Boot your system
2. Run the Krytn Teleop Task 
3. Explore the interface and look around the coffee shop. 
4. Use the RQT tool to drive the robot by moving the sliders (forward and reverse, left and right)

# Questions 

"I don't like this gui thing, I want to use the command line"

If you want to drive from the command line, you can use the command: 
`ros2 run teleop_twist_keyboard teleop_twist_keyboard` and that will let you drive it around. 

"But what about drones? Can we control drones like this?"

Currently the GUI takes in control for a 2D robot, but the actual data sent describes motion in 3D. Tomorrow we will look at the topics and messages getting sent around ROS, and you can use the tools you learn then to understand the data type and see if you can figure out how it could be used in a drone setting. 

"My rqt gui has weirdly detached"

Sometimes the rqt gui can have its settings corrupted. You can delete its settings by running the command: 

 `rm /home/ubuntu/.config/ros.org/rqt_gui.ini`

"What is this ROS thing anyway?"

ROS stands for Robotics Operating System, its a whole suite of tools that we will be using to construct the "brains" of our robot. It's been developed over decades, so we will be standing upon the shoulders of giants by using this free and open source software. 

# Day 1 Video

Here is a quick walkthrough of how to use the RQT Robot Steering Gui to drive Krytn 

https://youtu.be/njl_vYRr8vs




