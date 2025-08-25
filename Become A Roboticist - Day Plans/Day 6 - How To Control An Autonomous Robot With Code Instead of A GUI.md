
Lets use a simple topic publisher to tell Krytn where to go. 
In the process see how python is used, while also listening to a the pose topic to get the waypoint. 


# "Autonomous" Means Without Human Supervision 

So you've got a robot that runs ROS, driving around autonomously. But its not really autonomous is it? YOU still have to open RViz and click where on the map you want it to go. So lets change that, lets use a python program to make our robot truly autonomous. And yes, I know we still have to hit enter on the script, but that can be much more easily automated than clicking on a particular position in on the screen. 

# Don't worry about ROS Actions ... yet

ROS is big and complex, and when you start trying to drive Nav2 you'll often get pushed towards the concept of a ROS action. We'll start talking about ROS actions later, but at this early stage I think its too complex. Instead we will just publish on a particular topic, and this will keep the code simple.

# Action Steps 

1. Start the navigation system 
2. Listen to the `/pose` topic and move Krytn to two different parts of the map. 
3. Note the value of the `/pose` topic at the different locations. 
4. Go to the code at `src/bar_examples/krytn/krytn/move_krytn_topic.py` and modify the parameters to use the two poses provided.
5. Run the build task (ctr+shift+P, Run Task, Build)
6. Open a new terminal, and run `ros2 run krytn move_krytn_topic.py` to run the node from the command line.


# How does the code work?

Our node sets up a publisher to publish to the `/goal_pose` topic. This topic is provided by the nav2 system. It listens for a message of type `pose` and then attempts to move the robot to that location. 

The `pose` message type describes (in 3D space) a position and orientation. Our code specifies a specific x, y coordinate for the pose. It also specifies the coordinate frame that the pose is in. 

Once we have constructed the pose message we publish it on the `/goal_pose` topic, and away the robot goes.



# Task Walkthrough 

https://youtu.be/p0oq7_ToCZE
