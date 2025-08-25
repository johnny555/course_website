Planning and configuring all the joints on an arbitrary robot arm is time consuming. If we needed to follow the process we did for a single joint, we could spend a very long time setting up our robot arm. Fortunately, we can use MoveIT's setup assistant to do a lot of the heavy lifting for us. 

## Just Use The Defaults

MoveIT has lots of parameters and settings. I recommend just going with the default settings for as much as you can. 

## Don't skip the generate collision matrix 

An easy to miss but critical step is to generate a collision matrix for your robot. Make sure to hit the generate collision matrix button when it appears. 

The collision matrix is very important, as it lets MoveIT know that it can ignore self collisions of the robots links. This might seem like a dangerous thing to do, but in simulation the collision boxes of the arms are often simple shapes and are larger than the robot itself. They also don't model exactly how the joint mechanisms work. This means that its very common for adjacent joints to collide with each others collision boxes, even if they would not when in real use. The generate collision matrix just samples a bunch of joint positions and detects which links are in collision most of the time.

It is possible for two adjacent links to collide, but usually this is controlled using their joint ranges. That is, you set the upper and lower joint limits so that even at their limits, the joints won't actually be able to collide with each other. 

## Action steps 

1. Make sure your gripper arm's urdf.xacro is valid. MoveIT setup assistant won't work if its broken. You'll know its valid if your robot arm shows up in Gazebo after running the build task. 
2. Run `ros2 run moveit_setup_assistant moveit_setup_assistant` to get started. 
3. Configure the collision matrix, this lets your robot know which self collisions it should ignore when planning. 
4. Setup a `virtual joint`, this is an abstract thing that helps the arm planner reason about where the arm is in 3d space. 
5. Add a planning group for the arm.
6. Add a planning group for the gripper. 
7. Use the buttons to generate the ros2_control urdf modifications, ros2 control controllers and\ moveit control settings.
8. Skip setting up the perception system for now. ( we will come back to it later.)
9. Choose a new folder (for example: `src/my_robot_arm_moveit`) and generate the files into that new folder/package.
10. Look at the file `src/my_robot_arm_moveit/config/my_robot_arm.ros2_control.xacro` and copy the ros2 controller joints into your `src/my_robot_arm/urdf/my_robot_arm.urdf.xcro`
11. Copy the contents of `src/my_robot_arm_moveit/config/ros2_controllers.yaml` into `src/my_robot_arm/config/ros2_control.yaml`
12. Modify `src/my_robot_arm_moveit/config/joint_limits.yaml` so that all joints have acceleration limits. 
13. Copy the contents of `src/bar_examples/maci/launch/moveit.launch.py` and add nodes into `src/my_robot_arm/launch/gazebo.launch.py`. 
14. Add the line `moveit_config.move_group_capabilities["capabilities"] = ""` to `src/my_robot_arm_moveit/launch/move_group.launch.py` just before the return statement. 
15. Build your system
16. Run Gazebo and teleoperate your robot! 

##  My Arm Fails and is red in RViz, whats wrong? 

You likely didn't setup the collision matrix in the moveit setup wizard. Go and do it. 

# No Acceleration Limits Error

You may get an error that looks like :

```
[move_group.moveit.moveit.core.time_optimal_trajectory_generation]: No acceleration limit was defined for joint ur5_shoulder_pan_joint! You have to define acceleration limits in the URDF or joint_limits.yaml
```

To fix this, enable acceleration limits in the file `src/my_robot_arm_moveit/joint_limits.yaml`

E.g.

```
  ur5_elbow_joint:
    has_velocity_limits: true
    max_velocity: 3.1415926535897931
    has_acceleration_limits: true
    max_acceleration: 1.0
```

# Key Error "Capabilities"

You might encounter an error: 

```
[ERROR] [launch]: Caught exception in launch (see debug for traceback): Caught multiple exceptions when trying to load file of format [py]:
 - KeyError: 'capabilities'
 - InvalidFrontendLaunchFileError: The launch file may have a syntax error, or its format is unknown
```

This is due to a bug in MoveIT. We can work around it by adding the line: 
`moveit_config.move_group_capabilities["capabilities"] = ""`
in the file `src/my_robot_arm_moveit/launch/move_group.launch.py` just before the return statement. 

# My MoveIT crashes 

Sometimes MoveIT generates incorrect `joint_limits.yaml` files. In particular it generates limits that are integers, but crashes if they are not floating points. If you go into the `joint_limits.yaml` file you can add `.0` to the end of all the numbers to turn them into floating point numbers and avoid this error. 

# Recognize Objects Not Available

Careful students might observe an error that looks like: 

```
[rviz2-2] [ERROR] [1724652781.618219809] [moveit_926573201.moveit.ros.motion_planning_frame]: Action server: /recognize_objects not available
```

This is fine, it just means that we haven't setup a computer vision service for our arm. This is abit outside of the scope of this challenge, so we won't set up the service. The error, although scary sounding, is actually harmless and won't effect your robot. 

# My planner always fails

Check that your configuration is correct. Sometimes you can have lower vs upper case problems (i.e. `Gripper` instead of `gripper`), or your moveit controllers are not setup right. There are alot of parts to moveit, so it helps to go through and check each one carefully. 

That said, some movements are just really hard for the planer to figure out. You could try giving the robot a target pose that is part of the way there and see if it can get to that, and then plan to the final pose from there. 

# No action namespace specified for controller error

Sometimes your MoveIT configuration can miss putting in the ROS control namespace. 

This will result in an error that looks something like :

```
[move_group-6] [ERROR] [1741813686.422033086] [move_group.moveit.moveit.plugins.simple_controller_manager]: No action namespace specified for controller `arm_controller` through parameter `moveit_simple_controller_manager.arm_controller.action_ns`
[move_group-6] [ERROR] [1741813686.422084540] [move_group.moveit.moveit.plugins.simple_controller_manager]: No action namespace specified for controller `gripper_controller` through parameter `moveit_simple_controller_manager.gripper_controller.action_ns`
```

This happens when the moveit_controllers haven't been given the correct namespace by the moveit setup assistant. 

Look in your `moveit_controllers.yaml` file that was generated by moveit. Make sure that the field `action_ns: follow_joint_trajectory` is present for the arm and gripper controller as shown below: 

```
# MoveIt uses this configuration for controller management

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller
    - gripper_controller

  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - ur5_shoulder_pan_joint
      - ur5_shoulder_lift_joint
      - ur5_elbow_joint
      - ur5_wrist_1_joint
      - ur5_wrist_2_joint
      - ur5_wrist_3_joint
    action_ns: follow_joint_trajectory
    default: true
  gripper_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - finger_joint
    action_ns: follow_joint_trajectory
    default: true
```

What's happening here is that our ros2 control system provides a ROS 2 action at `/arm_controller/follow_joint_trajectory`, but if we don't provide the namespace then moveit will look for the action at `/arm_controller/` and won't find it. Causing an error, and stopping you from controlling the arm.
## Their Task + Walkthrough Video 

https://youtu.be/jMrV-QkSYlc

Todays task is to setup your robot using the MoveIT setup assistant and create a video/screenshot for LinkedIn/X. 