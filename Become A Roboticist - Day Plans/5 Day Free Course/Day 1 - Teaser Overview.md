Welcome to Start Creating Robots, 

Over the next 5 days we will look into some of the most important aspects of creating robots, with a focus on software and simulation. 

We are going to: 

- Control a robot that drives autonomously around a simulated world.  
- Experiment with Computer Aided Design to deploy our own, unique robot into a simulated world. 
- Use sophisticated software to control a robot arm with a gripper attached.
- Explore how to rapidly create robotic simulations

All the while we will use free, open source tools and everything will be running on your computer. In the end you will have access to the source code to do all of the above things. 

Todays main task is to get setup and ready. You can then play around with the examples, which we will look into in the following days. 

# Who is this for? 

This course is for anyone looking to get started creating robots. You might be a mid career professional or a student. The idea is to help you get a Robotics Operating System setup on your computer so you can experiment with robots!

You'll need some familiarity with coding to get the most out of this challenge, but its not necessary. Instead just some curiosity is all you need, and a computer, of course!

# Why Should You Learn Robotics?

Robotics is going to be critical to at least 2 important long term challenges facing humanity. 

- Climate change: Robots will help humans adapt to climate change, allowing industrial operations to continue as the climate becomes more inhospitable.
- Aging population: Robots will be able to help care for our elderly population as the number of elderly people far outstrips the number of workers

This is by no means the only places that robotics will be useful, but if you are a student or mid-career professional, getting the basics of robotics and becoming a roboticist will be valuable future skills.

# Getting Setup

Modern robotic systems are very complex and have lots of dependencies. They run on Linux and can be fragile to operating system changes. To make it as easy as possible for you get started creating robots, we are going to use a few tricks: 

1. We will look at our simulation and GUI controls through a web browser. 
2. We will use Docker to contain all the operating system complexity 

These instructions will work on Windows, Mac or Linux. The hardest part will be setting up Docker, but I'll rely on the official instructions to guide you. 

## 1. Installing Docker

Go to https://www.docker.com and download Docker Desktop for your operating system. Install it to your computer, following the official instructions from www.docker.com

## 2. Open a Terminal 

Once docker is installed (and running), open a terminal and type 

`docker run -p 6080:80 johnny555/scr:v2`

This will download the development environment, including the source code and important programs. 

## 3. Connect to the docker container 

Once the above step has finished (you'll see a message saying "vnc has entered a running state"), open a web browser and go to: 

http://localhost:6080/vnc.html

This will give you access to the docker container. On the left will be a panel, you can click the fullscreen option to go full screen, or resize your window as you like. 

## 4. Open VS Codium

On the desktop of the docker container there is a link to "VS Codium", open that and then use it to open the folder `/home/ubuntu/start_creating_robots2`. 
This is where all the source code for our examples lives. 
## 5. Run Gazebo Test

Lets start Gazebo. Gazebo is the most commonly used simulator for robotics with the Robotics Operating System, and is what we will be using in the course. 

In the VS Codium window, hit `Ctr+Shift+P` select "Tasks: Run Task" and then select "Gazebo Test". 

A terminal will open, with lots of text, but nothing else will happen. It's time to go to a web browser. 

## 6. Play with Gazebo

A new window will open, this is Gazebo, our physics simulator. 

Click the orange play button to start the physics simulation. you should see the purple ellipse fall over. Feel free to play with the controls and get a feeling for how Gazebo works. 

Try to spawn your own objects in, in particular try to spawn in a cube by clicking on the cube icon in the top left and then somewhere in the simulation world. You'll need this for the next few lessons :) 

## 7. Bonus: Play with other commands 

Congratulations! You've setup your computer. If you want a sneak peak at what we will be playing with over the next few days, try the following: 

### Autonomous Wheeled Robot

`Ctr+Shift+P` -> `Tasks: Run Task` -> "Krytn Navigation"

### Controlling A robot Arm

`Ctr+Shift+P` -> `Tasks: Run Task` -> "MACI Gazebo"

### Building Your Own Robot

`Ctr+Shift+P` -> `Tasks: Run Task` -> "Freecad"



## 8. Consider Signing up for Become A Roboticist 

If you've already seen enough and you want to take the next step, come and sign up for my next challenge cohort at www.becomearoboticist.com . In that course we will go deeper into how each of these systems works over 28 days, and help build your profile as a roboticist on the internet. 


### 9. Watch Video

Now you've read through all the text description, watch me step you through it in the video below: 

- add video 


See you tomorrow! 

John


