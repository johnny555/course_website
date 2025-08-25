# **The 4 Step SOLn Framework For Rapidly Setting Up A Robotics System**

A big benefit of taking this challenge is the template of a working robot system.

But if you want to sit down and create something from scratch, how should you think about it?

You can use the simple SOLn framework to guide you towards building any robot simulation. The steps are:

# 1. Simulate and Spawn

Start by building a gazebo simulation of the environment you want your robot to exist in. You can look in the file `src/bar_examples/krytn/launch/gazebo.launch.py` to see how to start a gazebo simulation and spawn your robot.

# 2. Operate your robot

Getting your simulation and robot created can be tricky (and we will see more about how to do this in later weeks), but one surefire way to know everything is working is if you can teleoperate it. You can see the later parts of the file: `src/bar_examples/krytn/launch/gazebo.launch.py` which shows how to run a teleoperation node.

In this stage I often find myself tuning coefficients of friction of my collision boxes, as your robot will be sluggish to move if you're coefficients of friction are too low.

# 3. Localize Your Robot

In this step you will need to make sure that your sensors are setup correctly. You will need to configure 3 things to setup your sensors:

- Add a sensor link and model to the robots description (urdf)
    
- Enable the gazebo sensor plugin and configure it for your sensor
    
- Setup the Gazebo bridge to transmit sensor messages.
    

The example Krytn robot has this setup for both a 2D lidar and a 3D depth camera. (Check `src/bar_examples/krytn/robot_description/krytn/krytn.urdf.xacro` to see the sensor links and plugin setup, while you can check `src/bar_examples/krytn/launch/gazebo.launch.py` to see the gazebo bridge setup.

Once your sensors are setup, you will need to configure your mapping system. This is best done with a 2D lidar. You can see how to launch a mapping node in:

`src/bar_examples/krytn/launch/mapping.launch.py`

This will also require a mapping configuration file, described at `src/bar_examples/krytn/config/mapping.yaml`

This mapping node also needs something to publish odometry. Odometry is the robots estimate of how it has moved by examining its wheel rotations. Fortunately, most ROS controllers will provide some odometry topics. The Krytn robot uses the `diff-drive` controller ros2 control plugin, which gives odometry.

# 4. navigation

Once your mapping is setup, and your map has been generated, setting up navigation is straightforward. You can see the file `src/bar_examples/krytn/launch/navigation.launch.py` which stands up a standard navigation 2 system. It is tuned with the file `src/bar_examples/krytn/config/navigation.yaml`.

It's important to make sure all of the previous steps are working before trying to tune your navigation stack. Errors in your robots model, plugins, teleoperation or mapping will lead to bad performance of your navigation system.

Congratulations! That's all you need to create an autonomous robot!