

Congratulations! You've modelled a link, but now its time to put it on your robot. There are still a few steps to go, and we are going to have to dive into how URDF's work a little bit to get there. 

The biggest thing to watch out for when writing URDF's is syntax errors. It's easy to accidentally forget a trailing `/` or `>` , and they can be very difficult to track down. 

# Don't Skip Inertia! 

In the instructions that follow, it's easy to skip the inertia step, as FreeCAD cross will allow you to export without inertia. And for this specific robot everything will work without it. However, when you start trying to create links that are connected to movable joints you will get errors if those links don't have inertia. So its good practice to start adding your inertia in now. 

## Action steps 

1. Load FreeCAD Cross workbench
2. Import the krytn robot model using the FreeCAD Cross Workbench "Import URDF" link (mode is at `src/bar_examples/krytn/robot_description/krytn.urdf.xacro`.
3. Add a new link to krytn. 
4. Add your design from yesterday as both a real and a visual in that link new. 
5. Add a new joint to connect it to your link. 
6. Export the project as `krytn_with_tray` 
7. Build and then run the rviz to see it working/check the export worked as you expect.
8. Identify the part of the exported URDF that contains the tray link and the tray joint.
9. Copy that into the krytn urdf at `src/bar_examples/krytn/robot_description/krytn.urdf.xacro`
10. Use an inertia macro from `src/bar_examples/magni_description/robot_description` to give your new link some inertia. 
11. Now run krytn launch file. Drive krytn with the new tray. 

# URDF and XACRO, what are these things? 

URDF, or Universal Robotic Description Format is a standard way of describing the physical properties of a robot. It describes the shape and location of the physical links as well as the joints of the robot. It also describes many of the coordinate frames attached to the robot. 

XACRO (XML Macros) is a language for writing XML files. URDF's are actually XML files in their syntax, so XACRO is often used with URDF's. The reason why XACROS are used is to make the URDF files simpler and more re-usable. You can see the `krtyn.urdf.xacro` has several lines that include other files, and `src/bar_examples/magni_description/magni.urdf.xacro` shows examples of macros that are used to make the robot easier to maintain. 

The XACRO files are not valid URDF. The `xacro` command needs to be used to convert a xacro file to a raw URDF. These URDF are typically much longer than the original XACRO. 

# But The Robot Model Isn't Realistic 

You may have noticed that your model looks good, but doesn't include important things such as screws or mounting brackets. These things are important when you go to physically manufacture a robot, but in our situation, we are just creating a simulation. We can skip some of the neccesary details for manufacturing. It will also make our simulation run faster as our computer won't need to consider a high poly count. This is one reason why getting CAD drawings into robot simulations can be difficult, as a cad drawings often contain much more detail than you need for a robot simulation. 

# So what actually happened here? 

All of the work from the last two days was to be able to paste the following into your `krytn.urdf.xacro`

```
  <link name="krytn_tray">
    <visual>
      <!--Krytn tray part/Body.-->
      <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.0"/>
      <geometry>
        <mesh filename="package://krytn_tray/meshes/krytn_tray_ex_Krytn_Tray.dae"/>
      </geometry>
    </visual>
    <collision>
      <!--Krytn tray part/Body.-->
      <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.0"/>
      <geometry>
        <mesh filename="package://krytn_tray/meshes/krytn_tray_ex_Krytn_Tray.dae"/>
      </geometry>
    </collision>
    <xacro:box_inertial_with_origin x="0.398" y="0.221" z="0.150" mass="1.0">
      <origin xyz="-0.0 0 0.00" rpy="0 0 0" />
    </xacro:box_inertial_with_origin>
  </link>
  <joint name="tray_joint" type="fixed">
    <parent link="base_link"/>
    <child link="krytn_tray"/>
    <origin rpy="0.0 -0.0 1.5707963267948966" xyz="-0.3 0.01 0.168"/>
  </joint>
```

Lets walk through the code to see what it does: 

## Tray Link 

In its most basic form, a link just specifies a name and some meshes. 

The code `<link name="krytn_tray">` gives the link its own name that the rest of the URDF can refer to it. It also specifies a unique coordinate frame that the `tf` system will use to refer to the link. 

This link contains 3 important sub tags, `visual`, `collision` and `inertia`. 

Both `visual` and `collision` specify the mesh's to be used. Usually the visual mesh is very complex, while the collision mesh is a coarse approximation. In this case we are using the visual mesh for both as its not that complex. 

The code: 
```
<geometry>
	<mesh filename="package://krytn_tray/meshes/krytn_tray_ex_Krytn_Tray.dae"/>
</geometry>
```
Is what specifies the mesh. You can see that it is still referring to the `krytn_tray` package that we created. 

The `inertial` tag is hidden by the use of the xacro macro:
```
<xacro:box_inertial_with_origin x="0.398" y="0.221" z="0.150" mass="1.0">
	<origin xyz="-0.0 0 0.00" rpy="0 0 0" />
</xacro:box_inertial_with_origin>
```
This is a xacro macro which evaluates the appropriate values for inertia based on the inputs (`x`,`y`,`z` and `mass`). You can see the definition of this macro in the file `src/bar_examples/magni_description/urdf/inertial.urdf.xacro`. 

## Tray Joint 

The `joint` tag is much shorter, but it is still fairly complex:

`<joint name="tray_joint" type="fixed">`

This gives the joint a name, and also specifies the type of joint. Joints are usually `fixed` which means they don't move. `revolute` or `continuous` which means they move like an elbow or a wheel, and `prismatic` which means they move along an axis. 

The next tags specify which links the joint connects to: 
```
<parent link="base_link"/>
<child link="krytn_tray"/>
```
Finally the origin of the joint is provided: 

`<origin rpy="0.0 -0.0 1.5707963267948966" xyz="-0.3 0.01 0.168"/>`

This specifies what transformation must be done to get to the new origin of the child link from the origin of the parent joint. These numbers can be hard to determine, and its the reason why I like to use FreeCAD cross to do this task. 

## Watch The Video


Todays task is to create your own link and add it to your robot. Then create a video or screenshot and share it on LinkedIn/X. 

https://youtu.be/I9owHSSDKbI

