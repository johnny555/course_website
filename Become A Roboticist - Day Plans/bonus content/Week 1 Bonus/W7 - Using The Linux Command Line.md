# Opening the terminal in VS Code

Make sure that you are in the dev container (it will have the text dev container down the bottom left with a green background). Now look along the top menu. Click Terminal -> New Terminal

A terminal window will pop up in the bottom of your screen here and then you can start typing in commands.

# Vocabulary of The Command Line

![](https://s3.us-west-2.amazonaws.com/content.podia.com/8sbj8ocf1k7dgf8een53oz8wd414)

It starts with the name of the command, followed by some arguments separated by spaces. Some arguments will have flags, like `-l` or `--long` these flags can have arguments attached to them, such as `-l arg2`.

## list directory command

You can see the vocabulary rules in action with the ls command.

`ls` With no args lists the current directory

`ls src` Lists the `src` directory.

`ls -h` Lists the current directory in human readable format.

`ls --human-readable` is the same as `ls -h`

`ls -h --sort time`, lists current directory in human readable format, sorted by time.

`ls --help` prints out help for using the ls command.

## Basic commands

`pwd` prints the current working directory

`cd` change your current directory

`ls -lah` list current directory in a list, showing all and in a human readable format.

# The Command Prompt

If you look at your command prompt, you will see there is a bunch of stuff before you start typing. This contains some helpful information, and although its often customized by linux users, what you see above is the default in use for our VS Code environment.

![](https://s3.us-west-2.amazonaws.com/content.podia.com/zesw3kdsr92sxmcmx6nf1086sq48)

The image above shows the structure. It starts with the username, followed by an `@` symbol and the name of the computer you are in. Since we are inside the dev container it'll be referencing the containers name. Finally a `:` separates the current path, this is handy as it means you won't need to run `pwd` to find out your current path most of the time.

# Common Commands In This Challenge

`git` For managing source in a single repo

`vcs` For managing multiple git repos at once.

`ros2` For interacting with the ROS system.

`colcon` For building our ROS system.

`source` Source is for basically getting variables and running some setup scripts

`echo` Repeats whatever arguments it is given.

Try using `--help` on the above commands

# Environment Variables

Your command line is actually a programming language called bash!

You can set variables in bash by going

`NAME=value`

and you can use the `echo` command to check variables have been set using:

`echo $NAME`

Note the `$` in front of the name of the variable.

Try and see what the value for the variables `ROS_DISTRO` and `IGN_GAZEBO_RESOURCE_PATH` are.

# Looking like a hacker

Now you know the basics of command line, you can download a fun program off github, EDEX-UI.

[https://github.com/GitSquared/edex-ui](https://github.com/GitSquared/edex-ui)

This program gives you a normal command prompt, but wraps it in a science fiction feeling interface. It's a lot of fun! Enjoy!