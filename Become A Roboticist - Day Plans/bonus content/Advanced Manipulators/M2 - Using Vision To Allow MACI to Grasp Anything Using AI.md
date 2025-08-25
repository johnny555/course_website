## What is the problem?

Now that we can grasp objects with the collision avoidance on, all we need to do is to know where an object is, and we could go and pick it up. 

But how can we use our sensors to figure this out? 

## Colour is not robust

One way we could detect an object is by its colour. 

In the simulation the soda can is very red. We could simply filter out all the red pixels in the image and find our object. 

The problem is that this won't work for anything that isn't red, and so we can't really use that. 

# Warning: Dependencies

This lesson assumes you have completed the lesson `Setting Up Jupyter` and that you have your `venv` ready to do python coding. 

Also we base this off the `my_robot_arm` example, but make sure to adjust the location of the realsense so it is looking side on at the soda can. As we are using Large Language Models the perspective needs to be a view that is common in the internet, and there are lots of side shots of soda cans, but not many top down shots.

If you use a top down view, the CLIPSeg model may not be very confident in its location estimates of the can. 

## Action steps 

1. Install the dependencies with pip inside your virtual environment. 
   `pip install opencv-python ipympl transformers torch pyntcloud py-cylinder-fitting`
2. Start by setting up subscribers to get the colour images and depth images from ROS, as well as configuring moveit2 interface
3. Resize the image to the right shape for clipseg and pass it through the model with the prompt `a can of soda`
4. Resize the extracted segmentation image and then use it to filter the depth points in the depth image 
5. Use the depth image to points to convert the depth cloud to points 
6. Filter the points to remove any outliers (i.e. floor hits)
7. Fit a cylinder to the points using pycylinder fitting. 
8. Add the cylinder to the planning scene
9. Move the arm to the pre-grasp position (just go above the cylinder by a few cm)
10. Disallow collisions on the cylinder so it can be grasped
11. Move the arm to the location of the cylinder and then close the gripper. 
12. Move the can wherever you want to! 


##  Will this work on non-cylindrical objects? 

At this stage, no. This pipeline is designed for detecting cylindrical objects. 

If you want to do general objects, you'll need to do 2 things differently. 
1. Fit arbitrary meshs to the point clouds. 
2. Find figure out good grasp angles for general shapes. 

Currently these are very difficult problems (although AI is making head roads here!), so for simplicity this demo just looks at picking up cylindrical objects that can be segmented using text. 

Thus it should work for bottles, kettles, jars, etc. 


## Why not use a model like yolo (You Only Look Once)

Yolo is a great image detection model. 

However you are limited to only the classes that yolo is trained on. If you want to use it for different classes you'll need to do some fine tuning on a collected dataset. 

By using a ClipSEG we don't have to do any training, it will one-shot figure out what to do! 

# "I want to see the code! "

You can see the code for this in the `solutions` branch on the `bar_ws` repo. Here: https://github.com/johnny555/bar_ws/blob/solutions/src/jupyter-scaffold/notebooks/Coke%20Detection.ipynb

## Video walkthrough

https://youtu.be/NshLTYjdMGY
