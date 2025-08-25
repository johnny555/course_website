
## What is the problem? 

Just like Krytn wasn't truly autonomous until it could be driven with code, our new robot arm isn't autonomous until it can be controlled with code either. 

## Build On The Shoulders Of Giants, Don't Reinvent

Beginners often try to build things from first principles. In this case we could try to look at the action servers of MoveIT and build our own interface. Yet we don't need to do that because someone has made a python library to do it already. We can happily use it, `pymoveit2`.

## Action steps 


1. Copy the `arm.py` file from `src/bar_examples/maci/maci/arm.py` and paste into `src/my_robot_arm/my_robot_arm/arm.py`
2. Adjust the call to the MoveIT2 node to use the correct end_effector_name (`'gripper'`) and group_name (`'arm'`)
3. Add an entry into `src/my_robot_arm/CMakeLists.txt` to add the `arm.py` as a program that can be run. 
   `install(PROGRAMS`
	   `my_robot_arm/gripper.py`
	   `my_robot_arm/arm.py`
   `DESTINATION lib/${PROJECT_NAME}`
    `)`
4. Modify the linux permissions of `arm.py` file so that it can be executed (`chmod +x src/my_robot_arm/my_robot_arm/arm.py`)
5. Listen to the `/joint_states` topic to find out the joint positions of your desired goals. 
6. Modify the code to also move to a target joint position after moving to the home configuration. 
7. Build and then run using `ros2 run my_robot_arm arm.py`

##  Error: "Service 'plan_kinematic_path' is not yet available. Better luck next time!"

This cheeky error might mean that your names of your services are incorrect, or something is holding up the planner. You can try again and see if it gets resolved. 

## Error: Can't Find Planning Group ur5

If you see an error like this: 
```
[move_group-6] [ERROR] [1724655699.524358393] [move_group.moveit.moveit.planners.ompl.planning_context_manager]: Cannot find planning configuration for group 'ur5'
```
Then you need to adjust your call to the MoveIT constructor in python to use the correct planning group `arm` instead of the one for the maci arm `ur5`.


# Walkthrough of arm.py 

Here is a step by step guide through the `arm.py` file: 

First step tells the interpreter to use python3 when running this file:
```
#!/usr/bin/env python3

```
Next we import the libraries we will need, including pymoveit2. 
```
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread
from pymoveit2 import MoveIt2
```
Now we setup the two joint states we want to move to:
```
home = [0., 0., 0., 0., 0., 0.]
pos_2 = [0.03651937399763931,
            0.6163979002202862,
            -1.0578528842289465,
            2.1342398474310653,
            -0.29110002780261596,
            -1.6980087753472988]
```
Next we define the main function, and begin with standard ROS 2 setup to create a node and setup our logging system. 
```
def main():

    rclpy.init()

    node = rclpy.create_node(node_name="my_robot_arm_control")

    logger = node.get_logger()
```
We create a callback group for the `MoveIt2` constructor, and then call the constructor. This object hides the complexity of dealing with MoveIT by providing an easy to use interface. 
```
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(node=node,
                      joint_names=['ur5_shoulder_pan_joint', 'ur5_shoulder_lift_joint',                                  'ur5_elbow_joint','ur5_wrist_1_joint',                                  'ur5_wrist_2_joint','ur5_wrist_3_joint'],
                      base_link_name='ur5_base_link',
                      end_effector_name='gripper',
                      group_name='arm',
                      callback_group=callback_group )
  ```
  
Because we setup callbacks, we now need to start the threads and executors, which we do so here:

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()
    ```
Now lets use `moveit2` to move to the home position, and wait until it is finished.
```
    logger.info("Move To Home Position")
    moveit2.move_to_configuration(home)
    moveit2.wait_until_executed()
```
Now to move to the next position and wait until complete.
```
    logger.info("Move To Position 2")
    moveit2.move_to_configuration(pos_2)
    moveit2.wait_until_executed()
```
Finally we shutdown our node and clean up our threads. 
```
    rclpy.shutdown()
    executor_thread.join()
```

This last line will run the `main()` function whenever this file is called as a python script. 
```
if __name__ == '__main__':

    main()
```


## Todays Task
https://youtu.be/YmNF-PG84gs

Your task is use PyMoveit2 to control your robot arm to a position you choose. 