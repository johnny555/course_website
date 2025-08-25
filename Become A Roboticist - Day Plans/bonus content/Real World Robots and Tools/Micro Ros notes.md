The Easy Way To Connect Your ROS System To Real World Microcontrollers - A Quick Primer

Micro ROS is a framework that let you publish and subscribe to ROS topics in your micro controllers code without needing to know anything about how ROS communicates:

[https://micro.ros.org](https://micro.ros.org/)

I'm going to use my Work In Progress (WIP) motor controller example to show you how its done.

Although its WIP, its still enough to demonstrate how easy connecting your Micro controller into ROS can be.

[https://github.com/johnny555/st3215_control](https://github.com/johnny555/st3215_control)

To get setup, I'm using [platform.io and targeting an ESP-32-s3 wifi chip.](http://platform.io/)

It's a VS Code extension.

I like [platform.io](http://platform.io/) because it helps manage dependency management in a structured way, and keeps my configuration tidy.

[https://platformio.org](https://platformio.org/)

I'm also basing my example heavily off the linorobot2_hardware example here:

[https://github.com/linorobot/linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware)

But cutting out a bunch of stuff I don't need.

In this example, I'm going to be publishing and subscribing to a joint_states topic.

I'm going to parse the joint_states topic to receive motor commands, and I'm going to publish joint_states to let the ROS system know the motor positions.

We are going to need to include a bunch of headers, such as :

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>

#include <rclc/rclc.h>

#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>

#include "jointstate.h"

[https://github.com/johnny555/st3215_control/blob/main/src/controller.cpp](https://github.com/johnny555/st3215_control/blob/main/src/controller.cpp)

These headers are just the standard ROS stuff, and also my special jointstate.h class to make things a bit easier for me.

We will create some global variables to keep track of my nodes, executors, timers, and publishers/subscribers:

rclc_executor_t executor;

rclc_support_t support;

rcl_allocator_t allocator;

rcl_node_t node;

rcl_timer_t control_timer;

rcl_publisher_t joint_state_publisher;

Its very important to choose the ROS transport type.

But its also just 1 line of C code.

I'm choosing to use serial here, but you can also use WIFI if your micro controller supports it!

set_microros_serial_transports(Serial);

[https://github.com/johnny555/st3215_control/blob/c1a415aa9f8cb9505bd1f8e1e1b7494e3d99b7df/src/controller.cpp#L261C1-L262C1](https://github.com/johnny555/st3215_control/blob/c1a415aa9f8cb9505bd1f8e1e1b7494e3d99b7df/src/controller.cpp#L261C1-L262C1)

I've borrowed my main loop from the linorobot_hardware2 example.

[https://github.com/johnny555/st3215_control/blob/c1a415aa9f8cb9505bd1f8e1e1b7494e3d99b7df/src/controller.cpp#L277](https://github.com/johnny555/st3215_control/blob/c1a415aa9f8cb9505bd1f8e1e1b7494e3d99b7df/src/controller.cpp#L277)

It basically listens to see if an agent is connected, and then runs the createEntities function if so.

The create entities function just runs various included init functions on the global variables initialised earlier:

[https://github.com/johnny555/st3215_control/blob/c1a415aa9f8cb9505bd1f8e1e1b7494e3d99b7df/src/controller.cpp#L187](https://github.com/johnny555/st3215_control/blob/c1a415aa9f8cb9505bd1f8e1e1b7494e3d99b7df/src/controller.cpp#L187)

i.e.

rclc_node_init_default(&node, "st3215_control", "", &support)

Make sure to initialise everything, including nodes, and executor

For the subscriber we first need to initialise it, telling it the ROS message type and the topic name:

rclc_subscription_init_default(

&joint_state_subscriber,

&node,

ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),

"desired_joint_state")

Then we need to add it to the executor, giving it the call back that it will run:

rclc_executor_add_subscription(

&executor,

&joint_state_subscriber,

&joint_msg_sub,

&joint_callback,

ON_NEW_DATA

Now in the subscription call back, we just cast the message to the correct type and use it:

void joint_callback(const void * msgin)

{

sensor_msgs__msg__JointState *msg = (sensor_msgs__msg__JointState *) msgin;

...

It's important to setup the memory correctly, and for complex message types like joint state this can be tricky.

I created the joint_state.h class to solve this for me:

[https://github.com/johnny555/st3215_control/blob/main/lib/jointstate/jointstate.cpp](https://github.com/johnny555/st3215_control/blob/main/lib/jointstate/jointstate.cpp)

Upon construction the object makes a properly created message.

Although I later found this part of the micro ROS documentation (under micro-ROS utilities):

[https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/](https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/)

That suggests I could have just used an inbuilt utility instead of manually initializing my memory.

Usage of the publisher is even easier than the subscriber.

Just fill the message with the data that you want to, and run this line:

rcl_publish(&joint_state_publisher, &joint_msg, NULL);

One final thing.

You need to run a micro_ros agent and configure your micro-controller to talk to it (over serial or WIFI).

It is this agent that lets you publish and subscribe from your micro controller like magic.

So it must also be running for everything to work.

That's it!

I hope you found this useful.

If this was too far down the rabbit hole, don't worry.

I have a free 5-day course that will give you an easy intro into robotics with ROS.

It won't cover micro-ROS, but it'll get you started down the path.

https://[startcreatingrobots.com](http://startcreatingrobots.com/)