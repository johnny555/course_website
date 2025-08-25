In this course we've looked at robots with wheels, and we've looked at stationary robots with arms. 

Can we combine the two? Yes we can. Here is how: 

# Action Steps 

1. Create a URDF that combines both the MACI arm and the Krytn robot. Call it `mobile_arm.urdf.xacro`.  
2. Run `moveit_setup_assistant` to specify the config for this mobile arm. Make sure to setup a Virtual Joint for your arm. 
3. Adjust the ROS2 control system so that the controller has both the mobile base and the arm setup. Write a new `ros2_controller.yaml` for this situation. 
4. Combine the launch files from krytn and maci. We need the gazebo stuff, the bridges, and the launching of move_group. 
5. Adjust the static transforms for the new robot name. 

## Errors 

If you get the following error, you'll need to change your joint_limits acceleration from an integer (such as 10) to a double (such as 10.0)
```
[move_group-10] terminate called after throwing an instance of 'rclcpp::exceptions::InvalidParameterTypeException'
[move_group-10]   what():  parameter 'robot_description_planning.joint_limits.ur5_shoulder_pan_joint.max_acceleration' has invalid type: expected [double] got [integer]
```




# video

See the video guide below: 

https://youtu.be/c6CSpfD3eWQ