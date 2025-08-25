# Valuable Robots Have Actual Jobs

And so they need to have a physical body to do their work.

This challenge has focused on how to setup a simulation, but it is important to understand what it will take to bring this simulation to reality.

This section is a brief tour of some of the considerations you'll need to thing about if you want to bring your robot to life.

# Physical Robot Body

I'm going to assume that you are using a commercial robot as a starting point (like Krytn's Magni base, or MACI UR5e arm), so we won't need to choose motors or build the main structures. We will focus on the software components that you will need to consider instead.

You will also want to measure as precisely as you can any extra links you add on, and put that detail into your URDF.

# Replace The Simulation Feeds

Assuming that you don't have an exotic Gazebo plugin (like detachable joints), you'll still need to deal with the joint control and feedback, as well as sensor data.

You'll want to find libraries that provide a `ros2_control` hardware driver for your joints. This will be the easiest path. The UR5 has control drivers here:

[https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

While the Magni silver has very experimental ros2 drivers.

For your sensors, you will want to find open source drivers that can publish the sensor topics for you.

Both of these will need a fair bit of configuration and tuning. In particular you will need to calibrate your cameras to compensate for their lens distortion.

[https://github.com/Slamtec/rplidar_ros](https://github.com/Slamtec/rplidar_ros)

[https://github.com/IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros)

# System Upstart

Most robots don't have keyboards and screens attached to them.

So we need the ROS system to turn on when its computer is turned on. The best way to do this is, in my opinion, to use the linux SystemD. It's beyond the scope of this course to show you how to do this, but that's a good starting point for you to look into.

[https://www.digitalocean.com/community/tutorials/how-to-configure-a-linux-service-to-start-automatically-after-a-crash-or-reboot-part-1-practical-examples](https://www.digitalocean.com/community/tutorials/how-to-configure-a-linux-service-to-start-automatically-after-a-crash-or-reboot-part-1-practical-examples)

You will also want to consider what should happen if a node stops working. You can configure lifecycles in ROS2, but this is also quite an advanced topic.

[https://github.com/ros2/demos/blob/humble/lifecycle/README.rst](https://github.com/ros2/demos/blob/humble/lifecycle/README.rst)

# User Interface

Most users won't be willing to use RViz to interact with your robot.

You can build simple web based user interfaces, or have some other way to control it.

[https://robotwebtools.github.io](https://robotwebtools.github.io)

# Onboard Computers

Most robots will need some form of onboard compute.

Where will it live, how will it be wired to the motor controllers and sensors?

How is your robot powered? How do your computers get power?

# Networking

You need to consider how data leaves your robot and goes to some controller. Are you using wifi? What kind of networking and packets will it use?

ROS2 uses DDS, which in turn uses UDP and UDP is blocked by windows in WSL2 so you may need a linux box to control your robot.

# Safety

Consider your robots potential impact to someone's health.

You should do a safety check every time before operating your robot.

You need to make sure you can quickly shut the robot down before someone gets hurt.

It's important to take safety seriously, and if you don't know how to do a safety check and how to include safety in your work practices you should find someone to get some help!

# Show Your Work

- Write a post about what you learnt in taking this robot into the real world.