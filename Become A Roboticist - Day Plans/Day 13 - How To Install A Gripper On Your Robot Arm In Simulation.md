
Once you have a gripper design, your are only part of the way through your robot build. Your gripper needs to be attached to something, and that also needs a design and a control system. Today we are going to install the gripper onto a robot arm, playing around with URDF and XACRO files. 

## Pay Careful Attention The Syntax

It is very easy to make mistakes in the XML files we are writing today. Pay careful attention, even things such as the lower or upper case of words can cause errors. 

There is a bonus video on URDF's and Xacro if you need a primer. 

## Action steps 

Make sure to watch the video below as you are doing these steps: 

1. Make sure you exported the project from FreeCAD into the folder yesterday. 
3. Open the my_robot_arm project, and look at the robot description file in `urdf` folder. 
4. Add inertia to the gripper base and the gripper finger by importing the inertia macro from `magni_description` package. 
5. Create a new file in the urdf folder call it `my_robot_arm.urdf.xacro`. 
6. Copy the preamble from the MACI project, up to line X. Then make sure to end it with a `</robot>` tag. This should copy the code that creates the ur5 arm. Delete the code that references the realsense. 
7. Copy the contents of the of `my_robot_arm.urdf` into `my_robot_arm.urdf.xacro` to add your gripper. 
8. Create a new joint to connect the end of the gripper `ur5_tool0` link to the base link of your gripper.
9. Modify `description.launch.py` to load your `my_robot_arm.urdf.xacro`
10. Remove Lines 25-37 of `gazebo.launch.py` to allow paths to get setup correctly (see video). 
11. Run `ros2 launch my_robot_arm gazebo.launch.py` and see if you see your gripper on the robot arm. 

##  Why Does My Arm Flop Around?  

You'll notice that your arm falls under gravity and rocks around. This is ok, we haven't setup any control yet. Once we add in the robot arm control system the arm will stop flopping around. 

# Help! My arm doesn't appear!

This usually means that some configuration isn't correct.  Check for the following:
- 

## Inertia Errors 

If you get an error message from Gazebo that looks like this: 

```
[ruby $(which gz) sim-4] Warning [parser_urdf.cc:2777] Error Code 19: Msg: urdf2sdf: link[base_link] is not modeled in sdf.[Err] [UserCommands.cc:1151] Error Code 17: Msg: A model must have at least one link.
[ruby $(which gz) sim-4] [Err] [UserCommands.cc:1151] Error Code 17: Msg: A model must have at least one link.
```
It could mean that your inertia wasn't setup correctly.

Check your robot arm has correct inertia values. In particular, you need to make sure that your mass is non-zero and your moments of inertia are non-zero. By default FreeCAD Cross exports invalid inertia values which need to be cleaned up before it will work in Gazebo. 

If it persists check that `display.launch.py` is pointing to your new `my_robot_arm.urdf.xacro` file.

## Missing Joint

If you get an error like this: 

```
[robot_state_publisher-2] Error:   Failed to find root link: Two root links found: [base_link] and [world]
[robot_state_publisher-2]          at line 265 in ./urdf_parser/src/model.cpp
[robot_state_publisher-2] Failed to parse robot description using: urdf_xml_parser/URDFXMLParser
[robot_state_publisher-2] terminate called after throwing an instance of 'std::runtime_error'
[robot_state_publisher-2]   what():  Unable to initialize urdf::model from robot description
```

It might be because you haven't specified the joint connecting the gripper with the robot arm. Make sure you specify a joint between `ur5_tool0` and `base_link`.

## Path Errors 

If you see errors like this: 

```
[ruby $(which gz) sim-4] [Wrn] [Util.cc:859] Failed to load mesh from [model://ur_description/meshes/ur5e/collision/wrist1.stl].
[ruby $(which gz) sim-4] [Err] [SystemPaths.cc:425] Unable to find file with URI [model://ur_description/meshes/ur5e/collision/wrist2.stl]
[ruby $(which gz) sim-4] [Err] [SystemPaths.cc:525] Could not resolve file [model://ur_description/meshes/ur5e/collision/wrist2.stl]
[ruby $(which gz) sim-4] [Err] [MeshManager.cc:211] Unable to find file[model://ur_description/meshes/ur5e/collision/wrist2.stl]
```

It means that you need to delete the code in `gazebo.launch.py` that sets up paths. Go and 10. remove Lines 25-37 of `gazebo.launch.py` to allow paths to get setup correctly. 

## Other Errors? 
Still not working, you can also try:

- Check for syntax errors in your urdf.xacro. You can try running xacro manually by going to a command line and running `xacro -v src/my_robot_arm/urdf/my_robot_arm.urdf.xacro`. If it outputs a urdf file then its ok, but if it outputs some errors then you can try and fix them. It's very easy to forget to end a `<` or use the wrong tag name, I do it all the time, so keep an eye out. 
-
- If the xacro works but it still won't spawn, sometimes it can be something to do with how Gazebo is reading it. You can parse it manually by running `gz sdf -p src/my_robot_arm/urdf/my_robot_arm.urdf.xacro`, this will work like the `xacro` command above, either it will output a properly formatted file, or it will give you some errors you can use as clues to go bug hunting. 

## Video 


https://youtu.be/YOZ3nH3S0Yk