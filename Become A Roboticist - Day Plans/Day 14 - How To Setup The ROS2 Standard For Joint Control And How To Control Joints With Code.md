
# Todays Goal

Yesterday we installed the gripper on the arm, but everything flopped around. Today we will focus on configuring our single gripper joint, and controlling that on the end of our wobbly arm. 

## Just Do One Joint First

It's tempting to go straight to adding control for all the joints in our robot arm. However, I want to show you what's required for a single joint control. Controlling all the other joints is just a repeat of the same concept as a single joint. 

# Use the command line as you go

As you do each step, try running `ros2 control list_controllers` and `ros2 control list_hardware_interfaces` to see how the effects of what we are configuring.  

The `ros2 control` system operates alittle outside of the normal topics, services and actions we've seen so far. This is because control systems need to have low latency. `ros2_control` exposes interfaces to the ROS ecosystem, but guards the low lever motor control to keep things reliable. 

## Action steps 

1. Add a ros2_control tag to your urdf.xacro file. 
2. Add a ros2_controllers.yaml file to the config directory, and populate it with the following
```
# This config file is used by ros2_control
controller_manager:
  ros__parameters:
    update_rate: 100 # Hz

    gripper_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

gripper_controller:
  ros__parameters:
    joints:
      - finger_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```
3. Add a step in the gazebo.launch.py file to start the controller manager, and then request the controller manager to initialize the high level controllers.
4. Inspect your system with `ros2 control` and `ros2 action` 
6. Copy the `gripper.py` file from `src/bar_examples/maci/maci/` into the project. Rename it `gripper.py`. Adjust its parameters so that it connects to the FollowJointTrajectory action associated that you configured when setting up the controllers. . 
7. Make the `gripper.py` script executable with `chmod +x`
8. Add the new `config` and `my_robot_arm` folders to CMakeLists.txt install directive. 
9. Add the `gripper.py` to CMakeLists.txt by coping the install directive from the `maci` CMakeLists.txt so that it will be registered as a script that can be run using `ros2 run `
10. Run your node (`ros2 run my_robot_arm gripper.py -j 0.03` ) and observe the gripper joint moving. 

##  Don't Write Your Own Controller! 

At this point students often wonder, "why all this work? After all, I could just write my own low level control interface and send position commands to my motor?" While its true that you could, by configuring it with ROS2 control, we setup our system to easily take advantage of the ROS ecosystem. Most ROS packages expect that the joints will have a ros2_control interface, so they will be much more plug and play. Also, if you even change your motor controller you will have to rewrite it, but if you buy a motor which already has a ros2_control interface, you can just drive it from ROS without worrying about the lower level details. 

Ultimately the extra setup for running ros2_control is worth it in the later flexibility your system will provide. 

# Why Use CMakeLists.txt, isn't that for C++ ?

You can check out the bonus videos on creating ROS packages, but in short, yes we need to use the CMakeLists.txt to tell ROS about all the important parts of our package. The alternative is to use pythons `setup.py` package management, but it is very cumbersome to use with ROS and it's now considered deprecated in the Python ecosystem anyway. CMakeLists.txt is also much simpler. 

There are two bits that we need to add to our CMakeLists.txt file today. We need to modify the line that tells CMake to move files into our `install` directory. It should look like this after you've changed it: 

```
install(DIRECTORY launch meshes rviz urdf config
  DESTINATION share/${PROJECT_NAME}
)
```
The only change is the addition of the `config` folder, which contains our ros2_control.yaml config. 

We also need to add the a programs directive, which will look something like: 

```
install(PROGRAMS
   my_robot_arm/gripper.py
   DESTINATION lib/${PROJECT_NAME}
)
```

Make sure you also run `chmod +x my_robot_arm/gripper.py` so that the gripper.py is an executable file. 

## ROS Actions 

ROS 2 control makes heavy use of ROS actions. Actions are like an orchestrated set of topics. When you want a robots joint to move, you usually want to give it a goal position, get feedback along the way as it moves, and finally receive a result when its done. You also might want the option to cancel mid way through. This is such a common pattern, that ROS wraps up all that functionality into what it calls a ROS action. 

## Stepping Through `src/bar_examples/maci/maci/gripper.py`

We start the python script by creating an object to connect to the action.

The `main()` function is where the action is. 

We first initialise ROS and create a node
```
    rclpy.init()
    node = Node("maci_node")
```
Next we setup Argument Parser. This lets the script take input from the command line. It specifies an argument `-j` which is given a value that is the value to move the gripper. The value from the command line is stored in `args.j`. 

```
    parser = ArgumentParser(prog="maci_node")
    parser.add_argument("-j", help="set finger joint", type=float, default=0.0)
    args = parser.parse_args()
```
Next we connect to the action server, and wait for it to be available before we try to send it a command: 

```
 ac = ActionClient(node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
 ac.wait_for_server()
 print("server found")
```
Next we prepare our goal for the Action Server. The `follow_joint_trajectory` has a fairly complex goal type. Also, we have to specify a goal for every one of the grippers 4 joints. The first step is to create a `JointTrajectory` message object, and give it the names of the joints we are controlling, and then the positions those joints need to go to. Because the joints are designed so that they are parallel when they are at the same angle, we will give the joints all the same target position (with `left_inner_finger_joint` reversed):
```
    traj = JointTrajectory()
    traj.joint_names = ["finger_joint", "left_inner_finger_joint",                 
                        "right_outer_knuckle_joint", "right_inner_finger_joint"]
    pos =  [i * args.j for i in [1,  1, -1,  1] ]
    traj.points = [JointTrajectoryPoint(positions=pos)]
```
Next we need to create our Goal that we will send to the Action. We assign it the trajectory we just created, and give the action a max time of 1 second: 
```
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    goal.goal_time_tolerance = Duration(sec=1)
```

Next we specify the path tolerance (how far away the joints can be during the move), and the goal tolerance (how close the joints must get to be considered successfully at their goal).

```
    tol = [JointTolerance(name=n, position=0.001, velocity=0.001) for n in traj.joint_names]
    ptol = [JointTolerance(name=n, position=1.3, velocity=0.1) for n in traj.joint_names]
    print(traj)
    goal.path_tolerance = ptol
    goal.goal_tolerance = tol
```
Finally we send the goal to the action server, and wait for its response before ending the program: 
```
    res = ac.send_goal_async(goal)
    print("waiting for goal complete")
    rclpy.spin_until_future_complete(node, res)
```

The last few lines are basic python to run the main function whenever the file is run: 

```
if __name__ == '__main__':
    main()
```
You might also notice that the file starts with the following: 
```
#!/usr/bin/env python3
```
This flags this file as a python3 file, so that when the shell tries to run it, it knows what program it should use. 

## Gripper Doesn't Move: Goal Time Tolerance Error

If your gripper doesn't move, you might need to adjust your finger joint limits.

Try adjusting the lower limit to "-0.01", like so:

```
  <joint name="finger_joint" type="prismatic">
        <parent link="base_link"/>
        <child link="finger_link"/>
        <origin rpy="0.0 -0.0 0.0" xyz="-0.06 0.001 0.06"/>
        <axis xyz="1 0 0"/>
        <limit effort="1000.0" lower="-0.01" upper="0.078" velocity="1000.1"/>
    </joint>
```

## Video 

Your goal today is to attach your own gripper and control it with code. 

https://youtu.be/8a_s_UxrPrU