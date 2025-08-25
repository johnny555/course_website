
Todays task is to setup mapping and navigation for your robot. You'll need to have configured a 2D lidar for this to work. 

## Pay Attention To Your Odometry 

It's very important to make sure your odometry is working correctly. The `slam_toolbox` is dependent upon good odometery, and the `nav2` stack needs it as well. 

The odometry is published by your diff drive controller that is configured in `config/ros2_control.yaml`. Make sure it reflects the physical characteristics of your robot such as wheel separation and wheel diameter. 

## Look for weird things in your map 

Even if slam toolbox publishes a map, it can be incorrect. 

Look for black dots indicating objects that don't exist, and patterns that look like receding walls. If your mapping is weird, then go back and try to figure out if your odometry is broken. 


## Adjust your sensors 

You can adjust your sensors parameters (e.g. `src/bar_examples/sensors/lidar_2d/lidar_2d.urdf.xacr`) to make mapping easier. Consider changing the range of the lidar so your robot can always see the walls as it moves around. 

Also consider changing the minimum so that the lidar discards any hits of the lidar on its body. 

## Action steps 

1. Copy the `krytn/mapping.yaml` config
2. If you changed the name of your base_link, you'll need to modify it in the config file. 
3. Copy the part of krytns mapping.launch.py file that starts up the `slam_toolbox` node. Modify its parameters to point to your projects name (i.e. `first_robot`)
4. Copy the rviz launch part from the `mapping.launch.py` 
5. Copy the navigation.yaml from krytn project into config folder
6. If you have changed your base_link name, search navigation.yaml and replace `base_link` with your base_links name. 
7. Copy the code from krytn `navigation.launch.py` file into your launch file, adjust the config to point to your `navigation.yaml`, not krytns one. 
8.  Launch your system, adjust your RViz to your liking.
9. Save your rviz config into your package's config folder. Now modify your launch file so that config is booted on start-up. 


##  Message Filter discarding message

You might encounter an error like: 

```
[slam_toolbox]: Message Filter dropping message: frame 'FirstRobot/base_link/lidar_2d_v1' at time 5.200 for reason 'discarding message because the queue is full'
```
It's normal for a few of these to happen as the system is turning on. However, if they start filling up the terminal you may need to fix it. In the case above, it's because the static transform publisher needed to be fixed. 

However, other times it can be because a node is using system time instead of simulation time. If that is the case, find the responsible node and set the `use_sim_time` parameter to true. Also make sure that your Gazebo bridge is copying across the `/clock` topic. 


# Video

[https://youtu.be/EQz3guggaqY](https://youtu.be/EQz3guggaqY)

Your task is to setup mapping and navigation for your robot. Post on LinkedIn/X a video or screenshot. 