Todays lesson is a walkthrough of the different transforms in our ROS examples. 

A transform is just a local coordinate frame, it's a convenient way to talk about the location of something in 3D space. 

Lets use 2D maps to give us some intuition about coordinate frames. 

Imagine that you are trying to find explain to someone where your computer is in your room. You might say, it's about 1m to the left of the door. That's your first transform frame, your room. Now, to figure out where computer is in your house, someone would need to know where in your house your room is. Perhaps its the 3rd bedroom at the end. The second transform is therefore your house plan. Now to know where your computer is in your city, you might need to know your street address. The third transform is the map of your city. Finally, someone might want to know where in on earth your computer is, this would be the world frame. Now, if someone knew the exact layout of your room, your house's floorplan, your city map and the location of your city in the world, they could work backwards through the frames to get the exact location of your computer in the global coordinate frame. Yes, they could just use GPS, but somethings are hard to measure in an absolute sense like GPS, but are easy to measure in a sense between two coordinate frames. 

In ROS, every link has its own transform. This is what a URDF describes. Also, the mapping systems all use transforms to describe their relationships. You have the robot's base frame, then the robots odometry link, a map frame and optionally a world frame. 

# Action Steps :

1. Run your favourite example so far
2. Turn on RViz and run the tf option
3. Experiment with tf to see how the 

# Video Walk Through

Watch this video to have a look at alot of the frames that are used by the course. 

https://youtu.be/-C4q_TYJs90