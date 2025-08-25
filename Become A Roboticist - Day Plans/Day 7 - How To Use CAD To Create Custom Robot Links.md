## Why Create Custom Links in CAD?

It's a great idea to buy an off the shelf robot for your project. This will save you a lot of time dealing with low level electronics and drivers. However, if its an off the shelf robot, it probably will need some customisation to actually make it useful for your use case.

Today we are going to look at using FreeCAD to sketch over images to create a 3D model of a tray for Krytn. This will also give us a chance to get used to FreeCAD and its way of building models. 
## Use Real Robots As Templates

It can be tempting to just imagine a shape out of thin air, hoping that your mental model will be enough. With training, many engineers and artists can do this, but for us beginners there is an easier way.  Just copy the shape of something that already exists! This will make sure the robot model is realistic. 

By realistic I mean something that is likely to be manufacturable and also strong enough to do what it needs to do. It's very easy in CAD to design objects that will fall apart under their own weight, or too heavy to be usable or impossible to manufacture. By looking to replicate what exists we can side-step this early on. When you become a skilled mechanical engineer feel free to skip this advice and make whatever you can dream of! 

## Action steps 

1. Open FreeCAD (ctr+shift+P -> Run Task -> Freecad )
4. Go File->Import , and import pictures of tray from `/workspace/src/bar_examples/krytn/images`
5. Use the ruler on the images to calibrate them to the correct scale. 
6. Use the transform tool to move the images so they line up with each other and with where you want the origin to be. 
7. Chang to workbench "Part Design" and create a new part and a new body. 
8. Create a new part and a new body
9. Think about the largest part of the shape, and sketch that. 
10. Pad & pocket that shape and now start refining it by iterating over the pad and then the cut tool.
11. Save your FreeCAD design file so we can use it tomorrow.

##  Why Not Use Some Other CAD Tool Instead? 

Why FreeCAD when there are lots of other almost-free and paid CAD programs out there. However, FreeCAD has two things which make it the best tool to use. 
1. It will be free forever. This means that if you stop being a student or work somewhere without a CAD budget, you won't be have to relearn how to build CAD models all over again. 
2. The FreeCAD Cross Workbench add on integrates extremely well with ROS 2. This will reduce the amount of effort you need to spend to get your model working. 

However, if you want to you can create your robots mesh using any tool, and just import it into FreeCAD to take advantage of the FreeCAD Cross Workbench. 


## Todays Video

https://youtu.be/w4IxnBUk_W8

Create a FreeCAD model of Krytn's tray. 