
Today its time to play with autonomous navigation, built on top of the mapping system.

# Why Do We Need Navigation?

Yesterday we setup mapping, so your robot will know where it is in the world. Next you need to give it the ability to figure out how it should move. But this isn't as straight forward as you might expect - we need algorithms for navigating around obstacles in our path, and we need algorithms to help us stop when an unexpected obstacle suddenly veers into view. 

# Don't Worry About All The Options!

We are going to configure Nav2, one of ROS's most well maintained code bases. It's very easy to get overwhelmed by the sheer number of parameters and settings in Nav2 (check out `config/navigation.yaml` to see what I mean). But for beginners, you can simply grab the example configuration, tweak it alittle bit, and get going very quickly. The default configuration is very robust and it'll serve you well in these simulations. 

# Two important concepts: 
Before we get into it, there are two important concepts: 

**Paths** - A potential path that the robot could traverse. 

**Cost maps** - a map of how "costly" it is for the robot to traverse an area. Higher costs indicate a wall or high chance of collision. 

# Steps 

1. Run the Krytn Navigation Task 
2. Use the "set goal pose" tool to set a goal pose 
3. Use RViz to show the local and global cost maps 
4. Use RViz to show the global and local path plans 
5. Examine the navigation.yaml code

# Day 5 Video
https://youtu.be/RKcVsCXfHCE