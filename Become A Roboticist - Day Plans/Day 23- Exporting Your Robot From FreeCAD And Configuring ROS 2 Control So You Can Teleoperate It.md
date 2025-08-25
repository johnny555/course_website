
Just like our MACI arm needed a control system setup, our new robot will as well. 

## Action steps 

1. Select the Robot in FreeCAD, set its output path to your new path (e.g. `src/forklift_freecad`)
2. Change to FreeCAD Cross Workbench, select export URDF and generate all files into (e.g. `src/forklift_freecad`)
3. Modify the `my_first_robot` package in `src/bar_examples/my_first_robot` to point to your new project instead of `my_first_robot_freecad`. You'll need to modify `gazebo.launch.py` and `src/bar_examples/my_first_robot/robot_description/first_robot.urdf.xacro`.
4. Add valid inertia to your exported urdf, e.g. in `src/forklift_freecad/urdf/FirstRobot.urdf`
5. Add joints in the `ros2_control` tags for every new joint you added in your robot. 
6. Modify the `src/bar_examples/ros2_control.yaml` to include any extra wheels or joints your robot has.
7. Run gazebo.launch.py and see your robot in simulation. 
8. Teleoperate it. 

# Troubleshooting Guide: 

## My Robot Fails To Spawn

Double check that your inertia values are correct. 

You need nonzero mass and moments of inertia values. You can copy valid inertia from other robots if you need to. 

## Wheels Spinning But Nothing Happens?

You might find that your wheels spin but don't get any traction. If that's the case, you'll need to add friction parameters to your collision. Look at how its done in `magni.urdf.xacro`. 

If your robot falls through the floor, you may not have defined any collision shapes. Go and add collision entries in FreeCAD (or manually in the URDF) and try again. 

## My Robots Wheels Don't Spin 

Double check you have added all the joints into the `ros2_control` tags. 

Also, double check that the controllers have been started. Run 

`ros2 control list_controllers`. You should see atleast a robot_state_publisher and a diff_drive controller. 

Also run `ros2 control list_hardware_interfaces`, you should see the joints you have defined in ros2_control tags. 

If there are no controllers, check that the start controllers step is being run in your launch file. 

If there are no hardware interfaces, check that you have added the joints in your ros2_control. 

You can also run `ros2 node list` to check that the `/controller_manager` node is running.


## My Robot's wheels sink through the floor 


In Gazebo you can right click on your robot in the entity tree, and make it transparent. Then right click and click "view->collisions". This will show you your robots collision meshes. If it is sinking through the floor one of the meshes might be incorrect. 

When you identify which link needs fixing, go back to your urdf file and fix it there. 

## My robot has fallen over at an odd angle

This often means your robots centre of mass is not above its wheels. 

In Gazebo Right click on the robot in the entitiy tree and click "view -> inertia". This will show you where the inertia for each link is. You can modify the origin tag in your robots base link to adjust its inertia so that it is above your wheel base. 

## Their Task + Walkthrough Video 

Your task is to setup your robot with ROS2 control and teleoperate it around. 

https://youtu.be/gRQ5Cb1jQI4