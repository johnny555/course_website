## What is the problem? 

Robotics and AI is a great pairing. 

Robots can be the literal arms and legs of an AI that is attempting to achieve some task. But so far, we haven't really used any artificial intelligence or machine learning. All of our systems have been excellent algorithms, optimising cost functions such as cost-maps or following heuristics for path planning. 

The problem is our tools aren't quite setup yet. Most AI systems use python with a package manager like `pip` or `conda`. However ROS uses the systems package manager `apt` by default, and this doesn't always have everything that an AI developer needs. ROS produced a stop gap with the use of the `rosdep` package, but it still won't have every package that an AI developer needs. 

Finally, most AI developers use very interactive shells, such as Jupyter notebooks, to prototype their systems. If we want to start doing computer vision, we will need to setup these systems. 

# Common Mistakes: 
## Watch out for the tokens 

One thing to watch out for as we setup Jupyter is that when jupyter starts it will need a token. This is a good security measure, but can get annoying. 

Make sure that you look through your terminal output to get the token.

## Add the colcon ignore file!

Your juptyer will launch just fine from your venv if you follow these steps without adding the colcon ignore file. 

The problem is that when you next run `colcon build`, colcon will try to build everything in the `myenv` folder and get really confused. We can tell `colcon` to ignore the folder by adding an empty `COLCON_IGNORE` file to the base of the `myenv` directory. 

## Alternative Setups 

There currently isn't a "right" way to setup Jupyter in ROS, but there are lots of options. 

### Anaconda With RoboStack: 

A good alternative is: https://robostack.github.io, which lets you use the `conda` package management system. Unfortunately the set of packages you can install with this framework is limited (although growing). Another problem with conda is that it uses its own binary versions of python, normally this is ok, but if we want to be able to launch our nodes with a `ros2 launch` command like everything else in our system, we will run into python library compatibility errors that are horrendous to debug.

### Docker Compose 

A really great way to keep your systems separate is to use docker. 

We could setup a docker compose file that spins up AI services with development container. We could then write simple python scripts to call these services. This would be a very robust and production ready approach. However, it's very complex and time consuming to setup and would require us to refactor how we launch our whole system.

## Our Approach: VENV

To get access to Jupyter, we are going to use the built in python module `venv`. 

`venv` will let us create a small ecosystem of python packages that we can use to run Jupyter and do AI stuff, without messing up our setup or too much administrative work. 

## Action steps 

1. Open a VS Code terminal at `/workspace` and run the command : `python3 -m venv myenv`
2. Add a special empty file to get colcon to ignore the env `touch myenv/COLCON_IGNORE`
4. Just like we have to source our `install/setup.bash` we will also need to source this python environment: `source myenv/bin/activate`
5. We can now install some python packages. Lets get Jupyter : 
   `pip install jupyterlab numpy matplotlib
4. Lets try running jupyter from the command line: `jupyter lab`
5. You'll get some output, click on the link that looks like `https://localhost:8888/lab?token=<some random numbers>
6. Now we have checked that it works, lets setup a launch file to run this:

```
from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import ExecuteProcess

from launch.actions import IncludeLaunchDescription


def generate_launch_description():  

    jupyter = ExecuteProcess(
        cmd="jupyter lab --LabApp.token='' --notebook-dir=/workspace/src/jupyter-scaffold/notebooks/".split(' '),
        output="screen"
    )
    gazebo = IncludeLaunchDescription(join(get_package_share_directory('my_robot_arm'), 'launch','gazebo.launch.py'))

    return LaunchDescription([jupyter, gazebo])
```
   Note that I've setup a new ROS project, and turned off the notebook token to make setting things up easier. 
   7. Run your new launch file, click on the link in the log output. 
   8. Now we are in Jupyter, here is a simple recipe to get data out of ROS and into your notebook. A subscriber node that puts data into a global variable: 
   
```
import rclpy
import cv2
from sensor_msgs.msg import Image, PointCloud2
from matplotlib import pyplot as plt
from threading import Thread
rclpy.init()

node = rclpy.create_node("basic_jupyter_example")
msg = None
depth = None
points = None

def depth_listener_callback(data):
    global depth
    depth = data

def points_listener_callback(data):
    global points
    points = data

def listener_callback(data):
    global msg
    msg = data

subs = [node.create_subscription(
            Image,
            '/realsense/'+n,
            cb,
            10) for (n, cb) in [("image", listener_callback),
                                ("depth_image", depth_listener_callback),
                                ]]

node.create_subscription(
            PointCloud2,
            '/realsense/points',
            points_listener_callback,
            10)

# Spin the node in background thread(s) and wait a bit for initialization
executor = rclpy.executors.MultiThreadedExecutor(4)
executor.add_node(node)
executor_thread = Thread(target=executor.spin, daemon=True, args=())
executor_thread.start() # Using threads, our node will keep spinning while we code 

```
9. Lets also do some basic image manipulation. Lets show the image, and then filter everything that isn't red from the image: 
```
import numpy as np

height = msg.height
width = msg.width
channels = msg.step // width
img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, channels)
plt.imshow(img)
plt.show()

# Create a basic mask for the colour red
mask =img[:,:,0] > 190

new_img = img.copy()
new_img[mask] = 0

plt.imshow(new_img)
plt.show()
```

## "But I really want conda, not pip!" 

If you want to use Anaconda's `conda` to manage your python, you can try running it in a separate docker container or using RoboStack. 

To get this to work, you'll have to setup your system to use RoboStack from the start. I think it'll be a pretty significant effort, but if you really want to do it, its ok. Good luck!

## "I don't like jupyter notebooks, do I have to do this?"

You're still going to need more than just the packages from `apt` to run your AI in a python script. So the `venv` ideas are still useful. 

But yes, if you want to you can just use python scripts to do the same thing. Remember that any plots or GUI's you run will open inside the webbrowser vnc at `localhost:6080` if you do it this way. 

# "I want to see the code! "

You can see the code for this in the `solutions` branch on the `bar_ws` repo. Here: https://github.com/johnny555/bar_ws/tree/solutions/src/jupyter-scaffold

# Video walkthrough 

https://youtu.be/H2E8RKmvXu8
