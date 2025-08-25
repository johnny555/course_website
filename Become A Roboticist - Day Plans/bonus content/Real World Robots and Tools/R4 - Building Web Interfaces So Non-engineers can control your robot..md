

**Why Build Web Interfaces?**

- Everyone knows how to use a website
    
- Lots of people know how to build a website (or even ChatGPT)
    
- Allows remote control with only an internet connection.
    

# System Architecture

We need a bridge, a web frontend/backend and something to control in ros2 .

## ROS Bridge Suite.

We will use the ROS Bridge Suite:

[https://github.com/RobotWebTools/rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)

To create a bridge between the ROS system and web technology (Web Sockets).

## Web Front-End/Back-End

We will use the Javascript library ROSLibJS

[https://github.com/RobotWebTools/roslibjs](https://github.com/RobotWebTools/roslibjs)

and the python web server Flask

[https://flask.palletsprojects.com/en/3.0.x/](https://flask.palletsprojects.com/en/3.0.x/)

## ROS Topic

Rather than use an Action, we will use the topic `/goal_pose` provided by the Nav2 system.

Note that since this is not an Action, it won't provide feedback.

# System Components

The system has the following files that you should examine:

```
src/bar_examples/krytn/launch/start_web.launch.py
```

This file starts the rossuite_bridge and starts the flask app.

```
src/bar_examples/krytn/krytn/static/index.html
```

This file connects to the ros bridge, connects to the topic and then adds onClick handlers to buttons so that they will drive the robot.

```
src/bar_examples/krytn/krytn/static/roslib.min.js
```

This file contains the roslib library that we are using. If you are familiar with javascript you can use npm to build this yourself.

```
src/bar_examples/krytn/krytn/app.py
```

This file is the flask app. It sets the `index.html` as the homepage and serves data out of the `/static` folder.


## CMakeLists.txt

Don't forget to update the CMakeListst.txt, you need to add the [app.py](http://app.py) file as well as pass the `static` folder to the `lib` directory so that Flask can find it. It should look something like this:

```
install(PROGRAMS
   krytn/app.py
   krytn/move_krytn_action.py
   krytn/move_krytn_topic.py
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
   krytn/static/
   DESTINATION lib/${PROJECT_NAME}/static
)
```

# Start the system

You can start the system by running Krytn Navigation (Ctl+shift+p run task -> Krytn navigation).

And then start the web server by running `ros2 launch krytn start_web.launch.py`.

Open a web browser at `localhost:5000` and see if you can control the robot through the web interface.
https://youtu.be/QofjksTDpHE