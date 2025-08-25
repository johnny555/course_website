
In the real world, your robots sensors won't always be 100% perfect.

Or, perhaps you'll be forced to design a robot that doesn't have complete sensor coverage of its surroundings. 

In todays short video, we will explore with how changing the sensor parameters can effect our map quality. 

We will be modifying the values of the sensor tag in the file: 
`src/bar_examples/sensors/lidar_2d/lidar2d_urdf.xacro`. 

# Action Steps :

1. Run the Krytn Mapping example (ctl+shift+p, Run Task, Krytn Mapping)
2. Observe the mapping quality by teleoperating for a while.
3. Now modify the parameters of the sensor, after each modification, look at how the mapping is effected. 
	1. Try giving a forward facing lidar by changing the min/max angle values
	2. Try increasing and reducing the range 
	3. Try adding noise

# Video Guide: 

https://youtu.be/6H9SxljrEtg