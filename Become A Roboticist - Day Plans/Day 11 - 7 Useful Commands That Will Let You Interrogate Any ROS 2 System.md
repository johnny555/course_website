
A short exercise in the different things you can do with ros2 command line 

node
topic 
service 
action 
control
interface
launch

By the way, this lesson assumes that your comfortable with the Linux command line, you can checkout the bonus video on command line usage if you're new to the command line. 

## Why Use The Command Line?

Up until now, you've used the shortcut Run Tasks in VS Code to run the code. But as we get deeper and deeper into ROS, we are going to find that we need to use the terminal more and more. So todays task is to look at the different things you can do with ros2 command line.

## Why Wait Untill Day 11 To Teach This?

A lot of courses like to teach this the command line and how to build a ROS package upfront. I believe that this stuff is boring and dry, and it's not until you've really got a feel for what's going on that you are ready to sit through the boring stuff of playing with the ros2 command line. 

It's not actually that boring, it just seems boring when you are first starting out! 

## Action steps 


1. Look at `.vscode/scripts/maci_gazebo.sh` , try running each command in that file one at a time in the command line. What does each line do? 
2. Start maci_gazebo task running. This system has lots of moving parts: 
	1. Use `ros2 node list` to look at the nodes running 
	2. Use `ros2 node info` to get information about the nodes. Notice the topics and the actions.
	3. Use `ros2 action list` to get a list of all the actions in the system. What nodes are responsible for these actions? 
	4. Use `ros2 interface show` to show the data type of a topic and an action. 
	5. Use `ros2 control list_controllers` to see a list of controllers, what do they do? 
	6. Use `ros2 control list_hardware_interfaces` to see a list of hardware interfaces, how are these different from the controllers? 
	7. Use `ros2 service list` to look at the different services that are running. 
	8. Use `ros2 run maci maci_joint_move.py` to experiment with running ros2 nodes from the command line. 
3. Boot up your favourite example so far (either a Maci or Krytn) and investigate the nodes. Choose one node and investigate its topics, services and actions. Create a diagram of how that one node connects to others in the system. 

#  Creating the node diagram is tedious !

I know todays task can seem dull. 

But creating this diagram will give you a good sense for how things are supposed to sit together. And being able to use the `ros2` command to inspect your ROS system will be a valuable skill when you need to debug things in the future. 

# Video

https://youtu.be/O3obX7svyus