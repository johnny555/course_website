In the previous video we saw how to setup a ROS package, and we added a few dependencies to the `package.xml` file. Yet, this doesn't actually install the dependencies, we need something that can read a `package.xml` and download the dependencies. Thankfully, this is exactly what the command line tool `rosdep` does.

I prefer to use `rosdep` with an entire ROS workspace, because its just faster to do it that way, but you can do it one package at a time if you want. 

The nice thing about `rosdep` is that it will decide what package manager to use based upon the dependency. So if its something that can be downloaded with `apt-get` it will use that, but if it is something that needs `pip`, then it will use that instead. 

Rosdep works by looking up a public github repo that contains a list of packages and how to get them. You can check out the rosdep list here: https://github.com/ros/rosdistro/blob/master/jazzy/distribution.yaml

# Action Steps 

1. Initialise rosdep by running `sudo rosdep init`
2. Update rosdep by running `rosdep update`
3. Run `apt-get update` to update apts packages. 
4. cd into your workspace (e.g.  `cd /workspace` if in dev container) and run  `rosdep install --from-path src --ignore-src -y -r --rosdistro=jazzy`

https://youtu.be/oi4mqmO7IGQ

