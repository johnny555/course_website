
Using actual captured meshes from the world is a great way to make your simulations pop. It turns out it is pretty easy too!

# Use Scaniverse or a photogrammetry tool

I used the free tool [https://scaniverse.com](https://scaniverse.com)

It works with the lidar on my IPhone pro, so the meshes come out very good. However, if you can find a photogrammetry app that can export meshes you should be able to do similarly.

# Import into Blender , Export as .dae

You can import the mesh into blender. Once there you can modify the mesh, or just export it back into a `.dae` file for Gazebo. Don't forget to send it to the wsl partition of your computer. It'll look something like:

`\\wsl.localhost\Ubuntu\home\johnv`

Note that the `\\wsl.localhost\Ubuntu\` part is the important bit that takes you to the WSL drive. You should find where your `bar_ws` is after that.

# Create a gazebo model.

Once you've exported, you will want to create a gazebo model. The structure of the gamecity model looks like this:

![[Pasted image 20240903171920.png]]

You should put your mesh in the `meshes` folder. You can copy the gamecity folder from `src/bar_examples/gamecity/models/gamecity` if you want to. Then all you need to do is change the references to gamecity to whatever your model is.

## model.config

Model.config is mostly meta data, fill it out as appropriate:

```
<?xml version="1.0"?>

<model>
  <name>gamecity_full</name>
  <version>1.0</version>
  <sdf version="1.5">model.sdf</sdf>

  <author>
    <name>John Vial</name>
    <email>john@soundelve.com</email>
  </author>

  <description>
    A model of gamecity cafe, Perth Australia.
  </description>

</model>
```

## model.sdf

This is like a URDF but with slightly different syntax. Replace the gamecity references with references to your mesh

```
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name='gamecity'>
    <link name='link'>
      <visual name='visual'>
        <geometry>
          <mesh>
            <uri>model://gamecity/meshes/gamecity.dae</uri>
          </mesh>
        </geometry>
      </visual>

      <collision name='collision'>
        <geometry>
          <mesh>
            <uri>model://simple_gamecity/meshes/simple_gamecity.dae</uri>
          </mesh>
        </geometry>
      </collision>

      <pose>0 0 0 0 -0 0</pose>
      <inertial>
        <pose>0 0 0 0 -0 0</pose>
        <mass>1</mass>
        <inertia>
          <ixx>1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1</iyy>
          <iyz>0</iyz>
          <izz>1</izz>
        </inertia>
      </inertial>
      <enable_wind>false</enable_wind>
    </link>
    <static>true</static>
    <self_collide>false</self_collide>
  </model>
</sdf>
```

# Modify the world

Now that you have created a model, you can import it into a gazebo world. If you look in the file `gamecity_world.sdf` you will see the following tags:

```
    <include>
      <uri>models://gamecity</uri>
      <name>gamecity</name>
      <pose>-3.90177 -5.01456 0 0 -0 0</pose>
    </include>
```

Replace the gamecity references with the name of your model, and now that model should load when you load that world in gazebo.

https://youtu.be/X8qwDAh2ez8