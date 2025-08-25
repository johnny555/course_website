
# Why Do Robot Systems Need To Communicate?

A robot is a system of systems. It has a control system, a perception system, a navigation system and a task orchestration system. These systems all need to coordinate with each other, and each system is itself very complex and might have subcomponents that also need to share information. 

When Willow Garage (the original company behind the Robotics Operating System) started building ROS they needed to design something that was flexible enough for robots that haven't even been built yet, with necessary systems that may not have been invented yet. How might you design such a system?  

# Discord For Robots

The way ROS approaches this is that it provides a framework for communication and system composition. But I think the easiest way to think about it is to compare it to a Discord server (or SLACK, or MS Teams).

In Discord communication is separated into topics. In a heavily moderated Discord server, these topics will be strictly enforced - you won't be able to talk about the weather in a channel dedicated to game updates. In ROS we also have topics, and they are enforced by the topic data type. This means that the only communication on a particular topic must be in the right data format. For example, if a common topic in ROS is the `/cmd_vel` topic, which stands for "command velocity", it will contain messages that describe the desired velocity in x,y,z as well as the desired angular velocity around the x,y and z axes. Any other data will either create an error or be ignored by the nodes listening to the topic. 

In Discord you also have people, these people subscribe to channels they are interested in. In ROS, the people are analogous to ROS nodes. These nodes can subscribe to a topic if they want to be updated when a new message is posted, or they can become publishers in the topic if they want to publish it themselves.


# This seems bloated!

When beginner roboticists start to understand how ROS works, it's common for them to cry out "oh this bloated!", "I don't need all this complexity!". The problem is that ROS is made to be extensible and generic, so yes, there is a lot of complexity. But as soon as you want to build more than a simple demo, you're going to need the kinds of infrastructure that ROS provides for free. You will need Publisher/Subscriber message bus, and a way to orchestrate your system. By adopting ROS you will find that you can move faster as you will be able to take advantage of all the lower level work provided by other people in the ecosystem. 

# Listen To Your Robots Topics

1. Start by booting up the Krytn Teleoperation task from Day 1 
2. Now run the following command in a terminal `ros2 topic echo /cmd_vel`
3. Try driving the robot around but listen to the commands.

# Finding out what topics exist 

You can run the command: 

`ros2 topic list` to get a list of topics that you can inspect. 

Find a topic your interested in and listen to its output. 

# Find out what nodes exist 

You can find out the nodes in the system by running 

`ros2 node list`

you can get information about a particular node by running: 

`ros2 node echo <node_name>`

Use the `ros2` command to create a graph of your robot system. 

# A Trap for old players

People who have used ROS before, might be tempted to use tools like rqt_graph to create a graph of the nodes network without using the ros2 command. This is totally ok, but if you do use that tool, make sure to still investigate the data types of the topics. It's just as important to be able to do this in ROS 2, just as it was important in ROS 1. 

# Cmd Line Crash Course

Have you used the command line before? For some people this could be their first time. If that's you, checkout the bonus material for a crash course on using the command line in ROS. It'll help you understand the language of the command line.

# Watch Day 2 Video
https://youtu.be/fHQQDtXze1I

