
Welcome to the Become A Roboticist challenge. 

I know you are ready to get started, but you have a choice to make. Do you want to have an easy time with this course, a hard time or a dangerous time?

You see, I have 3 sets of instructions for you. I'll present the easy way first, because its what I recommend for beginners. But if you don't like it, you're free to do something else. Just know that I won't be able to help you as easily. 

The problem we have to overcome is that the Robotics Operating System is always pinned to a specific version of Ubuntu. (Ubuntu is one of the leading distributions of Linux). This means that getting it to work on non Ubuntu systems, or even Ubuntu systems of a different version number can be tricky or even impossible. I'm assuming that your computer isn't the exact OS that we need (for this course its Ubuntu 24.04).

Why did the makers of ROS do this? It's because ROS makes use of the ubuntu/Debian operating systems package manager to get all the programs and code libraries. This means that ROS needs to keep in lock-step with Ubuntu's Operating System upgrades, otherwise it might find that it doesn't have access to the right C++ libraries. 

The biggest mistake that beginners often make is to jump right in and install a fresh Ubuntu on their computer. This is the hard path, as installing Ubuntu, although easy at first, comes with its own huge set of configuration challenges, as well as the unfortunate side effect of potentially deleting all the data on your hard-drive.

# The Easy Path

So here is the easy path I think you should try first: 

1. Download & Run Docker Docker Desktop (www.docker.com)
2. Download VS Code (https://code.visualstudio.com/download)
3. If on Windows, install WSL and open an ubuntu shell
4. Go to your home directory (type `cd ~` and then clone the course repo by running the command: `git clone https://github.com/johnny555/bar_ws.git`
5. Move into the new directory by typing `cd bar_ws`. 
6. Now type `code .` to open VS Code in the course directory 
7. Install Dev Containers VS Code extension 
8. Open Folder in dev container 
9. Hit Ctrl+Shift+P (Cmd+shift+p on Mac) find "Tasks: Run Task" hit enter and run "Gazebo Test" task. 
10. Open a web browser to localhost:6080, and login through VNC
11. Play with Gazebo. 

# The Dangerous Path

You might not like that you have to open a web browser to view your windows. If you are on Windows 11 or Linux, you can use your systems graphical drawing system (X11) to render the windows instead. 

This puts you a dangerous middle ground and many of my students have struggled to get this to work for them correctly. But if you like to live dangerously, I have provided some files that can activate this feature for you. 

1. In the course repo go into the folder`.devcontainer`.
2. Rename the file `docker_compose.yaml`, to `docker-compose-easy.yaml`. There are two other docker-compose files, one for windows and one for ubuntu. Rename the one for your operating system to `docker-compose.yaml`. 
3. Rename `devcontainer.json` to `devcontainer-old.json` and rename the file `devcontainer-docker-compose.json` to `devconainer.json`.
4. Now in VS Code run "Dev Containers: Rebuild Container", to rebuild the container. 
5. Now try running Ctrl+Shift+P -> "Tasks: Run Task" -> "Gazebo Test" .

You should see the window popup in your OS instead of needing the web browser. 

Good luck! 

# The Hard Path 

You might be reading this thing "This seems slow and complicated, can I just install Ubuntu?"

Yes you can. Start by install Ubuntu 24.04. 

1. Follow the official ROS documentation to install ROS 2 Jazzy here : https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

2. clone the course repo (to `~/bar_ws`), cd into its directory and run: 

`sudo bash .devcontainer/install_dependencies.sh` this will install the dependencies for the course and get you setup.

3. Install VS Code so you can run the tasks, and configure the ROS VS Code extension. 

4. Finally, install FreeCAD (https://www.freecad.org/downloads.php) and the FreeCAD Cross Workbench (https://github.com/galou/freecad.cross). 

5. Double check that you can run Gazebo and FreeCAD Cross workbench. 

Good luck! 

# Day 0 Video: 

Here is the Day 0 video for how to setup on the easy path: 

https://youtu.be/QEVvZvUYo_Q