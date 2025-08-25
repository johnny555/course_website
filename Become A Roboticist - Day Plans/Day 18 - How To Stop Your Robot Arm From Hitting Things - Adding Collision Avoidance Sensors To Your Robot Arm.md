
So far we have been doing advanced teleoperation. However, with the Krytn robot we were able to get autonomous navigation. Is there a similar idea for robot arms? How can we have the robot arm avoid obstacles in its environment? 

## Grasping Is Colliding

One problem with adding collisions is that it will detect the coke can as an obstacle, and will prevent the gripper collide with it. Fortunately, we can add fake obstacles to the planning scene and then allow them to be collided with to let us pick up stuff with collision avoidance enabled. 

## Action steps 

1. Run the moveit setup assistant and edit the existing project (`src/my_robot_arm_moveit`)
2. Go to the perception settings, and setup a `PointCloud` source for your perception system. 
3. Go and generate, but untick the `move_group.launch.py` and `joint_limits.yaml` files so it doesn't overwrite the customizations we made earlier. 
4. Run gazebo.launch.py and try to position the arm so its shortest path passes through the table. 
5. Observe the arm avoiding the obstacle. 
6. Copy the code from `arm.py`, remove the part that moves the arm, and add an extra section: 
    `obj_id = 'coke2'`
    `pos = [0.28, -0.5, 0.7]`
    `quart_xyzw = [0,0,0,1]`
    `moveit2.add_collision_cylinder (obj_id, height=0.12, radius=0.03, position=pos, quat_xyzw=quart_xyzw)`
    `moveit2.allow_collisions(obj_id, True)`
    This will position a cylinder and set it disabled collisions. Change the values in pos so you get your cylinder positioned correctly. 
  7. Run `chmod +x` on your new script and add it to the list of programs in `CMakeLists.txt`. 
  8. Now build the system. 
  9. Move your gripper to above the coke can and when you are ready disable collisions and then pick up the can with your robot arm. 

##  But this is still very manual, how to make it truly autonomous? 

It's true that positioning the coke can's collision cylinder is manual. We aren't going to cover how to detect objects with computer vision, but that is how you can make this truly autonomous. Since we are using python to add the collision cylinder, you are free to use any AI tool to detect the location of the coke and put a cylinder around it. 

# Video
https://youtu.be/qcJjSahpBF0

## Todays Task

Your task is to setup collision avoidance using the depth camera and then add an object to the collision scene to allow you to pick up the coke can regardless. 

