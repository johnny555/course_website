
Now its time to focus on the environment your robot is going to be in. 

## Tip 1: Don't be a perfectionist

Don't worry about having a high level of detail. You'll be able to get away with a lot less than you think. What is the minimum needed in the simulation? You won't need any background objects. 

## Tip 2: Just Arrange Other Peoples Work

You can get free models from alot of places on the internet. https://app.gazebosim.org lets you use a browser to view the models you can download within Gazebo's resource spawner. 

Arranging other peoples work is much easier than creating something new from scratch.

The best form of other peoples work is an actual scan of a real environment. You can use ScaniVerse app to create a 3D model. See bonus video for a quick guide. 

## Tip 3: Don't be afraid to make a model with FreeCad

Sometimes what you want just doesn't exist. 

You know how to make stuff with FreeCAD, so just make a 3D model. You can use Blender to colour the model so it looks good afterwards. 

## Action steps 

1. Use the resource spawner to spawn in items to help build your environment. 
2. You can also search the listing at https://app.gazebosim.org
3. Save your environment
4. Edit your saved `.sdf` file and remove your robots description, so that you won't spawn with two robots. 
5. Either use the resource spawner or use the links from app.gazebosim.org to create your world. 
6. Check out http://sdfformat.org to see the specification for SDF files, in particular for changing the pose of an included file or changing its name. 
7. Adjust your gazebo.launch.py to load your world instead of an empty world. 
8. As you create your world, think about the specific task that you want to demonstrate your robot performing


## Their Task + Walkthrough Video 
https://youtu.be/vzMDXvjC8IE

