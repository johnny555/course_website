

Normally Python programs do one thing at a time, they just execute their scripts line by line. Usually this is exactly what you want, but this means that a python script will ignore other events when its running. 

This makes using Python in ROS abit tricky, as we need to listen to events all the time. The most common example is writing a topic subscriber. As nodes can publish on topics whenever they want, the code needs to be able to respond to topic messages whenever they come in. But it also needs to be able keep the node running. If it didn't then the node would shutdown and the system would stop.

A function that runs when an event happens, such as message appearing on a topic, is called a "callback". 

We will look at the basic topic subscriber example next, and then look at what happens when we want even more complex behaviour. 

# A topic subscriber

It's very common to subscribe to a topic in ROS. 

Look at this python node. You can see that it has a callback for running something when a message appears on a topic. You'll also notice the `spin` command. This command lets the node do two things at once by shifting its focus between them. It will run the things necessary to keep the node running, and then go and run any call-backs that need to happen in response to messages: 

```
#!/bin/python3

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class BasicSubscriber(Node):

    def __init__(self):
        super().__init__('basic_subscriber')
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10)

    def lidar_callback(self, msg):
        self.get_logger().info(f'I heard: {msg}')
 

def main(args=None):
    rclpy.init(args=args)
    basic_subscriber = BasicSubscriber()
    rclpy.spin(basic_subscriber)
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
```

If you can get your code to use only topic subscribers, you might be able to get away without needing threads, and just use `node.spin` instead.

However, as your systems complexity grows, you might start needing threads to avoid deadlocks. 

# Deadlocks and Threads 

Deadlocks occur when your program needs to wait on itself. 

We can see this if we introduce a more complex concept of a Service. 

A service is like a ROS topic, but it must return a value to the caller. Lets create a system that forces the robot to stop if it gets too close to a wall. 

It'll do this by providing a service which checks how close the robot is to a wall. If the robot is too close, it will return `True` and the node will publish cmd_vel of 0 until it is switched off.

Here is the code: 

```
#!/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
import numpy as np

class CollisionWarning(Node):
    def __init__(self):
        super().__init__('collision_warning')
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10)
        self.vel_pub = self.create_publisher(
            Twist,
             '/cmd_vel',
            10)
        self.close = False
        self.trigger_service = self.create_service(Trigger, '/test_collision', self.trigger_callback)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.client_service = self.create_client(Trigger, '/test_collision')

    def lidar_callback(self, msg):
        self.get_logger().info(f'I heard: {msg}')
        min_range = np.min(msg.ranges)
        if min_range < 1:
            self.close = True
        else:
            self.close = False

    def timer_callback(self):
        self.client_service.call(Trigger.Request())

    def trigger_callback(self, req, res):
        if self.close:
            msg = Twist()
            self.vel_pub.publish(msg)
        return res

def main(args=None):
    rclpy.init(args=args)
    collision_warning = CollisionWarning()
    rclpy.spin(collision_warning)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

If you try running this, you will find that it works until the first timer runs. After that nothing happens. This is because it is stalled in the timer callback that calls the service.

# Solve with threads and callback groups

So we need to give python more threads, as this code wants to be able to wait for a service call, while also running the service at the same time. 

However, we will also need to specify a thing called a "callback_group". The problem with just adding more threads, is that depending upon the speed of the service call timer, we could end up with all of our threads waiting for a response from a service call. We need to separate the call-backs into groups. This will help us avoid deadlocks. Using them is easy, we can simply add a the argument `callback_group=MutuallyExclusiveCallbackGroup()` to the things that could be in deadlock. This will make sure that they cannot run at the same time. 

```
#!/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import numpy as np

class CollisionWarning(Node):

    def __init__(self):
        super().__init__('collision_warning')
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10)

        self.vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.close = False
        self.trigger_service = self.create_service(Trigger, '/test_collision', self.trigger_callback, )
        self.timer = self.create_timer(1.0, self.timer_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.client_service = self.create_client(Trigger, '/test_collision', callback_group=MutuallyExclusiveCallbackGroup())

    def lidar_callback(self, msg):
        self.get_logger().info(f'Lidar msg received ')
        min_range = np.min(msg.ranges)
        if min_range < 1:
            self.get_logger().info(f'Too close')
            self.close = True
        else:
            self.close = False
            self.get_logger().info(f'Far enough away')

    def timer_callback(self):
        self.get_logger().info(f'Timer running')
        self.client_service.call(Trigger.Request())

    def trigger_callback(self, req, res):
        self.get_logger().info(f'Service request running')
        if self.close:
            msg = Twist()
            self.vel_pub.publish(msg)
        return res

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    collision_warning = CollisionWarning()
    executor.add_node(collision_warning)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```


You should also notice the slight change to the main function. Instead of calling `rclpy.spin`, we create a `MultiThreadedExecutor`. This executor is what manages the multi threads. We then add the node to the executor and run the spin function on it instead. 

Hopefully this helped give you a primer on Threads and callbacks, and helps to demystify the code in the more advanced examples. 

# Video Guide 

https://youtu.be/c8njYP3e9E4