
So, your robot exists in an interesting world, but it can't yet see. Today we are going to add sensors. 

Lets look at adding a 2D lidar, and a depth camera.

## PointCloud is all you need 

Beginner roboticists often want to add all the sensors that they can think of. 

Often your robot just needs a point cloud for it to be able to perform its actions successfully. 

Both the 2D lidar and depth camera produce point cloud style data. If you can learn to use this data type, you'll be able to handle most robot data that exists.

## Don't Add Too Many Sensors

Try to copy the sensors that exist on the robot you are simulating. But feel free to make them lower fidelity so your simulation real time factor isn't very low. Simulating sensors takes a lot of compute! 

## Watch Out! Gazebo transforms have different names 

An easy to miss step is to forget to add the transform nodes to your `gazebo.launch.py`.  These nodes translate the name that Gazebo uses to refer to the coordinate frame your sensor data comes in (e.g.`/FirstRobot/base_link/realsense_d435`) to the frame name that ROS uses (e.g. `realsense_d435`).

## Action steps 

1. Look at the sensors in the krytn example. 
2. Copy the xacro includes for the sensors you want. 
3. Add a joint to connect the sensor to your robots base_link, adjust its parameters to get the sensor where you want it.
4. Modify your gazebo.launch.py to bridge the sensor topics across. 
5. Add the transform nodes so that RViz can translate between Gazebo and ROS tf frames
6. Use RViz to check that everything is working. 


# Help my RViz is dropping messages 

You might find that RViz outputs a message like: 

```
[INFO] [1724986638.870550968] [rviz]: Message Filter dropping message: frame 'FirstRobot/base_link/lidar_2d_v1' at time 7.200 for reason 'discarding message because the queue is full'
```

These are ok as the system is starting up, but if it persists, and your robot can't show any data, you may not have added the transform nodes to your Gazebo.launch.py.

(the transform nodes look like: 
```
    static_pub = Node(package="tf2_ros",
                      executable="static_transform_publisher",
                      arguments=["0","0","0","0","0","0", "lidar_2d_link", "FirstRobot/base_link/lidar_2d_v1", ])
```
)

# Video

https://youtu.be/xK3nlqM5K1E
