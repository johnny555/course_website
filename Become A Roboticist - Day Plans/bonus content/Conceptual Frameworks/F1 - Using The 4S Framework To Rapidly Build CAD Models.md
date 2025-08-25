# How To Sketch a Robot in FreeCAD for ROS

If you want to try and re-use an existing robot, but you don't have the URDF and model files, then the 4S method is a structured framework for creating usable robots for simulation.

You can pair this framework with the SOLn framework from week 1. The 4S method addresses the "Simulate and Spawn" part of the SOLn framework.

# Example: Swarm Farm Robots

Swarm Farm Robotics is an Australian based robot company that creates small modular robots for automating broad acre harvesting.

We're going to use their robot as an example for the 4S method.

# 1. Sketch

Our first step is to sketch over the robot.

Use FreeCAD to import an image. Then identify each of the links in the robot and use the reference images to create sketches of each link. For repeated links (such as wheels), just create one version of it and create a clone of it in the structure step.

Don't worry about accuracy, just make rough sketches of each link.

# 2. Structure

A robot is nothing without its structure.

Use the FreeCAD Cross workbench to assemble the links into the correct shape. Continue using the images as a reference.

Create all the links in the robot, but where possible re-use components. For example, all the wheels are the same part, so we will just reference each wheel to the same FreeCAD body.

Adjust the joints so that all the links line up correctly.

# 3. Specify

Now its time to add some polish to the models.

For each link, attempt to add extra detail. You can keep the original coarse models as well, as they can be good for adding collision meshes. It doesn't have to be a complete recreation, but you'll be surprised how a little bit of extra detail can make a robot model go from ok, to great!

We should also add inertial properties to our links at this stage. This will help our robot be more realistic as it operates in our simulation.

# 4. Ship

Select the robot and export it as a project using FreeCAD.

Load it in ROS and use the `display.launch.py` to fine tune your joint parameters. You can also now open the mesh files in Blender and add colour.

Congratulations, you have successfully built a robot from nothing but a few images.

# Todays Task

- Pick a robot and try to follow the 4S method to create it.
    

# Show Your Work

- Post a screenshot of your robot sketches