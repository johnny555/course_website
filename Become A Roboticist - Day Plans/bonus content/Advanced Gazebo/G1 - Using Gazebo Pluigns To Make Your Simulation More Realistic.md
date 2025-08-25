# Gazebo Plugins

All you get from a basic gazebo simulation is simple physics of gravity, momentum and collisions.

To have a useful robot simulation you need to control and sensors, but sometimes you need even more. Some plugins can be used to simulate suction grippers or underwater sensors and more.

In this session we are going to look at the important gazebo plugins in the course so far, and go even deeper on how they are configured.

# Plugins So Far

If you look at the MACI and Krytn robots urdf.xacro files, you will find a set of tags like:

```
<gazebo>
  <plugin> 
    <!--... some stuff here ..  -->
  </plugin> 
</gazebo> 
```

This part of the code describes the plugins for that particular robot. The Krytn robot has the most interesting set:

```
  <gazebo>

    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <ros>
        <remapping>/diff_drive_base_controller/odom:=odom</remapping>
        <remapping>/diff_drive_base_controller/tf_odometry:=tf</remapping>
      </ros>
      <parameters>$(find krytn)/config/diffdrive_control.yaml</parameters>
    </plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </gazebo>
```

You can see 3 plugins are specified here:

- ros2 control plugin
    
- gazebo sensor system
    

Lets dive into 2 of the plugins.

# Basic Plugin Syntax

Every Gazebo plugin will have this structure in your urdf or even sdf:

```
<plugin filename=”some_name” name=”some::library::name” >
<!-- Some config here --> 
</plugin>
```

It will specify a filename, which should be provided by a ros package. It will also specify a name, which is the name of the class in the file. This file will be a compiled binary, so you can't just read the file.

However, unless you are writing your own Gazebo class, you won't need to change this anyway. Just be aware that these values are important and they need to be correct for your plugin to load into gazebo properly.

# Ros2_control

The ROS2_control is one of the most complex gazebo plugins we have used. Here is it for Krytn

```
  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <ros>
        <remapping>/diff_drive_base_controller/odom:=odom</remapping>
        <remapping>/diff_drive_base_controller/tf_odometry:=tf</remapping>
      </ros>
      <parameters>$(find krytn)/config/diffdrive_control.yaml</parameters>
    </plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </gazebo>
```

You'll see that this code does alot, it remaps topic names, loads a config file and makes the controller in the namespace `/krytn`. This is why all of the controller topics are prefixed with /krytn. If you compare this to the maci ros2 control example, you will see this remapping isn't there.

Here is the ros2_control snippet for `MACI`:

```
        <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
            <parameters>$(find maci)/config/ros2_control.yaml</parameters>
        </plugin>
```

It's much simpler, but it still needs a control config.

Lets examine part of the krytn config file, `diffdrive_control.yaml`

```
controller_manager:

  ros__parameters:
    update_rate: 100 # Hz
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    diff_drive_base_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_base_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.326
    wheels_per_side: 2
    wheel_radius: 0.128
    wheel_separation_multiplier: 1.0
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0
    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: base_footprint
```

The configuration here has two controllers, and contains the specification for the diff_drive base controller.

Looking at the maci controller is abit different:

```
controller_manager:
 ros__parameters:
  update_rate: 100 # Hz

  maci_controller:
   type: joint_trajectory_controller/JointTrajectoryController

  gripper_controller:
   type: joint_trajectory_controller/JointTrajectoryController

  maci_joint_state_broadcaster:
   type: joint_state_broadcaster/JointStateBroadcaster

  use_sim_time: True

maci_joint_state_broadcaster:
 ros__parameters:
  use_sim_time: True

maci_controller:
 ros__parameters:
  joints:
   - ur5_shoulder_pan_joint
   - ur5_shoulder_lift_joint
   - ur5_elbow_joint
   - ur5_wrist_1_joint
   - ur5_wrist_2_joint
   - ur5_wrist_3_joint
  command_interfaces:
   - position
  state_interfaces:
   - position
   - velocity

gripper_controller:
 ros__parameters:
  hold_on: true
  joints:
   - finger_joint
   - left_inner_finger_joint
   - right_outer_knuckle_joint
   - right_inner_finger_joint
  command_interfaces:
   - position
  state_interfaces:
   - position
   - velocity
```

Although the urdf of maci controller was simple, its configuration file hides alot of complexity.

You can see that its controller manager runs 3 controllers instead of 2. Although the configuration of these joint trajectory controllers is much simpler than the config of the diff drive controller. Nonetheless, it still requires describing all the joints that are to be controlled.

## ros2_control tags

The final part of the ros2_control plugin is the specification of the joints in the urdf under ros2_control tags. For Krytn this looks like (with some non-simulation specific stuff removed) :

```
<!-- Configure Control of Joints -->

  <ros2_control name="Krytn" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    <joint name="left_wheel_joint">
      <command_interface name="velocity">
        <param name="min">-10</param>
        <param name="max">10</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort" />
    </joint>
    <joint name="right_wheel_joint">
      <command_interface name="velocity">
        <param name="min">-10</param>
        <param name="max">10</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort" />
    </joint>
  </ros2_control>
  ```

These tags tell ros2 control what to expect for each joint. A command interface is something that can be controlled, while state interfaces are sensors. In this setup the joints can be controlled by specifying a joint position, while they will report their position, velocity and effort back. The `robot_state_publisher` controller takes the job of reporting this information back into the ROS system.

Note that unlike the other tags, the `ros2_control` tags don't need to be inside `gazebo` tags. This is because `ros2_control` can be used on real hardware as well. So by configuring your system with `ros2_control` you automatically make it easier to port your simulation to physical hardware.

# Sensor Plugin

Lets look at how the sensor plugin works. It's basic syntax is pretty simple:

```
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
```

All of the complexity is hidden inside of `<sensor>` tags. Lets check out 

```
     <gazebo reference="lidar_2d_link">
            <sensor name="lidar_2d_v1" type="gpu_ray">
                <topic>lidar</topic>
                <pose>0 0 0.0525045525 0 0 0</pose>
                <ray>
                    <scan>
                        <horizontal>
                            <samples>360</samples>
                            <resolution>1</resolution>
                            <min_angle>-3.14</min_angle>
                            <max_angle>3.14</max_angle>
                        </horizontal>
                    </scan>
                    <range>
                        <min>0.40</min>
                        <max>30</max>
                        <resolution>0.01</resolution>
                    </range>
                    <noise>
                        <type>gaussian</type>
                        <mean>0.0</mean>
                        <stddev>0.001</stddev>
                    </noise>
                </ray>
                <always_on>0</always_on>
                <update_rate>10</update_rate>
                <visualize>false</visualize>
            </sensor>
        </gazebo>
```

The `reference` parameter in the `gazebo` tag lets gazebo assign this sensor to a specific link on your robots body.

The `sensor` tags `type` parameter, tells the sensor plugin which type of sensor this is.

The `topic` tag tells gazebo to publish the data from this sensor on this specific topic. This is handy if you want your simulation and your real systems to be identical.



The remaining setting change parameters specific to this sensor.

# Other Gazebo Plugins

There are lots of gazebo plugins. But the best way to find out what is available is to read through the list of examples:

https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds

Make sure to be on the gazebo v8 branch. 




https://youtu.be/CPQf-HPnyHc