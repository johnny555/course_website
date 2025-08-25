
## Why design a gripper?

Robot arms are great, but to be really useful we need to give them hands, or an "end effector". There are lots of different types of end-effectors, but the most common are 2-finger grippers and suction grippers. We're going to look at a simple 2-finger gripper.

So far we've looked at creating links, but designing a simple gripper will give you exposure to controlling the most fundamental robot component, a single joint. Every robot is just a combination of links and joints, so with these two concepts you have the basics of robotics. 

## You don't want hands... yet


When beginners think of robot end-effectors, they often go straight to human like hands. The problem with human like hands is that they are very complex and often extremely fragile. To grasp something with a hand requires complex coordination of all the joints, and the motors for those joints are often small and too weak to do anything useful. It's much better to start with simpler gripper designs, ideally something that is fit for purpose for the particular task you want the robot to perform.

Leave building hands for when you've designed a few simple grippers. 

# The Three Steps 

## Step 1: Sketch The Base Link

This is the hand of the gripper. The part that will connect the wrist to the finger. 

Open FreeCAD and build an L-Shape. The L part will act like a thumb that the finger will press against. 

1. Open FreeCAD and go to the Part Design workbench
2. Create a new part and inside that a new body. 
3. Use sketch tool and pad to create an L-shape for the base link
4. rename the part and body to `base_link_part`, and `base_link_body`

## Step 2: Design the finger

The finger presses against the base link of the gripper. 

You can design it to simply be a straight finger, but you could also make it curved if you want to be really good a picking up a particular shape. 

Some engineers will design their grippers so that even if the grasp is perfectly aligned, the object will slide into the right location under pressure. Ultimately we are going to try to pick up a coke can, so you could consider modifying the finger (and the base link) to make gripping a can easier. 

1. Create a new part and a new body
2. Create the finger shape, with sketch and pad. 
3. Rename the part and body to `finger_part`, `finger_body`. 

## Step 3: Export To ROS

Now use FreeCAD Cross Workbench to export the gripper into your ROS system. 

1. Choose the FreeCAD Cross Workbench 
2. Create a new robot, set its output path parameter, choose `src/my_robot_arm`. 
3. Create two new links. 
4. For each link, add one of the parts you designed in steps 1 and 2. 
5. Right click on and show hidden so you can rename the `Label2` field to something sensible. 
6. Add a joint, set the parent as the hand of the gripper and the child as the finger 
7. Change the joint type to prizmatic, 
8. Select the Robot and hit the Export as URDF button to export 
9. Use the generated `display.launch.py` launch file to see if your gripper works, and check you setup the joint correctly. You'll probably find that it needs some tweaking. 
10. Open the file `src/my_robot_arm/urdf/my_robot_arm.urdf` and adjust the joint tag. You'll want to set the `axis` tag to a vector that points in the direction you want the prismatic joint to move in. You'll need to fix the limits so that the joint can move in a reasonable range. Repeat step 9 until you're gripper works as you expect. 

Here is a code snippet of my working joint for reference: 

```
  <joint name="finger_joint" type="prismatic">
    <parent link="base_link"/>
    <child link="finger_link"/>
    <origin rpy="0.0 -0.0 0.0" xyz="-0.06 0.001 0.06"/>
    <axis xyz="1 0 0"/>
    <limit effort="1000.0" lower="0.00" upper="0.078" velocity="1"/>
  </joint>
```

##  Types of Joints - Prizmatic, revolute or fixed

Most robots use rotational joints. These joints are easily driven by electric motors attached to gearboxes. As the name suggests, rotational joints rotate around an axis. In ROS they are called revolute or continuous. The difference between them is that revolute joints have limited rotation ability (for example an elbow joint), while continuous joints don't have rotation limits, for example a wheel on your car.

Todays gripper uses a prismatic joint. This type of joint can also be driven by electric motors or solenoids, and works by moving along a particular axis rather than rotating around it. 

You will also see the joint type "fixed" often in URDFs. This just means that the joint cannot move at all. 


## Video

https://youtu.be/Et8JDEv6FQA

