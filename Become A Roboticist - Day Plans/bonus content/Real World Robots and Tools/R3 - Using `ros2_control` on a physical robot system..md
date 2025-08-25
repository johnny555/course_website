The biggest reason to use a system like ROS is so that you can re-use your system between robots and across the simulation to real boundary. 

By using ros2_control to drive our robot (rather than using a Gazebo plugin), we have setup ourselves to be able to easily port our system across to a physical robot. However, there is just two things we need to change for our ros2_control to work. We need to find a low level controller for our motors, and we need to start the `controller_manager` node which is normally started by the gazebo plugin. 

Fortunately, for the Magni robot, I was able to find a motor control plugin for my robot that did just this. Once I got it running (with its dependencies), I could enable my motors and start sending `cmd_vel` messages to my robot base. Enabling teleoperation. 

This lesson details the steps I took to do this. To build a complete real work version of Krytn, you'll also need to install drivers for the lidar and realsense, as well as calibrate the sensor locations. But this video won't go into those steps. 

# Action Steps 

- SSH into the robot computer. I like to use the VS Code Remote Extensions port, which will gives us a window into our robot computer, just like VS Code gave us a window into the docker container running on our computer.  
- You can clone my fork of the `ubiquity_motor` repo, using branch `jazzy` here: `git@github.com:johnny555/ubiquity_motor.git`
- The code depends upon an unreleased version of the `serial` library, I used this one (ros 2 (ros2 branch of https://github.com/tylerjw/serial/tree/ros2)
- You need to create a new launch file, which doesn't start gazebo, but does start the controller manager here: https://github.com/johnny555/krytn/blob/jazzy/launch/real_robot.launch.py , this also re-uses the `diff_drive_control.yaml` file as it is given to the controller_manager node as parameters. 
- Ultimately this was to get the hardware interface so I could use ROS2 with Krytn
- To use the new hardware, it must be listed as a hardware plugin in side the `ros2_control` tages in the `krytn.urdf.xacro`. You can see that we also pass some parameters, like the serial baud rate and port id: 
```
	  <hardware>
        <plugin>ubiquity_motor/UbiquityMotorSystemHardware</plugin>
        <param name="serial_port">/dev/ttyAMA0</param>
        <param name="baud_rate">38400</param>
        <param name="left_wheel_joint_name">left_wheel_joint</param>
        <param name="right_wheel_joint_name">right_wheel_joint</param>
      </hardware>
```
- I got the plugin name from the file: https://github.com/johnny555/ubiquity_motor/blob/humble-dev/ubiquity_motor.xml 
- Finally, we should double check that the `diff_drive_control.yaml` parameters reflect the shape of physical robot, incase we aren't using a standard Magni robot.


# This seems like a lot of work, wasn't this supposed to be easy? 

Although it does seem like alot of work, most of the changes are limited to the edges of our ROS system. I.e. it's just the drivers and ROS controllers that need rework. 

The rest of the ROS system remains unchanged. And since our API's are well defined, we can be confident that this will be the case if we ever choose to change the robot base again. 

# Video 

Check out this video, where I walk you through the changes: 


https://youtu.be/xNnDS3xJVsM