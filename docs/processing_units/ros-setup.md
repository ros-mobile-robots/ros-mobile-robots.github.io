# ROS Installation

The robot setup is supposed to run on Ubuntu Mate 20.04 Focal Fossa. [ROS Noetic](http://wiki.ros.org/noetic) is intended to run with this Ubuntu version. To install ROS follow the [installation instructions](http://wiki.ros.org/noetic/Installation/Ubuntu).

!!! info
    In the [1.4 Installation step](http://wiki.ros.org/noetic/Installation/Ubuntu#Installation-1) you have to choose
    how much of ROS you want to install. For the development pc you can go with the `sudo apt install ros-noetic-desktop-full`
    command. For the robot install the `ros-noetic-robot` Ubuntu package. Other system dependencies will be installed
    with the [`rosdep`](http://wiki.ros.org/rosdep) command, explained in the following section.

Another program that is required to run ROS nodes written with the `rospy` client library is `python-is-python3`. Install it with:

```console
sudo apt install python-is-python3
```

## Dependencies

After having git cloned one or more ROS packages, such as [`diffbot`]({{ diffbot_repo_url }}), 
it is necessary to install system dependencies of the packages in the catkin workspace.
For this, ROS provides the [`rosdep`](http://wiki.ros.org/rosdep) tool. 
To install all system dependencies of the packages in your catkin workspace make use of the 
following command ([source](http://wiki.ros.org/rosdep#Install_dependency_of_all_packages_in_the_workspace)):

```console
rosdep install --from-paths src --ignore-src -r -y
```

This will go through each package's `package.xml` file and install the listed dependencies that are currently
not installed on your system.


## Build Tool: `catkin_tools`

To work with ROS we will use [`catkin_tools`](https://catkin-tools.readthedocs.io/en/latest/index.html) 
instead of [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make). `catkin_tools` provide commands such as [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) which we will use instead of [`catkin_make`](https://wiki.ros.org/catkin/commands/catkin_make) because the `catkin_tools` are more actively developed than `catkin_make`
[ref](https://robotics.stackexchange.com/questions/16604/ros-catkin-make-vs-catkin-build).

!!! note
    It is recommended to use `catkin_tools` instead of the default `catkin` when building ROS workspaces. 
    `catkin_tools` provides a number of benefits over regular `catkin_make` and will be used in the documentation. 
    All packages can be built using `catkin_make` however: use `catkin_make` in place of `catkin build` where appropriate.

!!! bug
    The current way to install `catkin-tools` in the [documentation](https://catkin-tools.readthedocs.io/en/latest/installing.html#installing-on-ubuntu-with-apt-get) 
    from the Ubuntu package repository doesn't work. Follow the steps below instead for now.

!!! success
    As of now the correct way to install `catkin-tools` is to use the following command:
    ```
    sudo apt-get install python3-osrf-pycommon python3-catkin-tools
    ```
    For your reference, you can read more about it in [this open issue](https://github.com/catkin/catkin_tools/issues/594).

After sucessfully installing `catkin_tools` we can create and initialize a workspace (called `ros` for this project) with the [commands listed in the build_tools documentation](https://catkin-tools.readthedocs.io/en/latest/quick_start.html):

!!! note
    Note that we already `source`d the `setup.bash` while following the [ROS installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu).


```bash
fjp@ubuntu:~/git/2wd-robot/ros$ mkdir -p ~/git/2wd-robot/ros/src    # Make a new workspace and source space
fjp@ubuntu:~/git/2wd-robot/ros$ cd ~/git/2wd-robot/ros              # Navigate to the workspace root
fjp@ubuntu:~/git/2wd-robot/ros$ catkin init                         # Initialize it with a hidden marker file
Initializing catkin workspace in `/home/fjp/git/2wd-robot/ros`.
----------------------------------------------------------------
Profile:                     default
Extending:             [env] /opt/ros/melodic
Workspace:                   /home/fjp/git/2wd-robot/ros
----------------------------------------------------------------
Build Space:       [missing] /home/fjp/git/2wd-robot/ros/build
Devel Space:       [missing] /home/fjp/git/2wd-robot/ros/devel
Install Space:      [unused] /home/fjp/git/2wd-robot/ros/install
Log Space:         [missing] /home/fjp/git/2wd-robot/ros/logs
Source Space:       [exists] /home/fjp/git/2wd-robot/ros/src
DESTDIR:            [unused] None
----------------------------------------------------------------
Devel Space Layout:          linked
Install Space Layout:        None
----------------------------------------------------------------
Additional CMake Args:       None
Additional Make Args:        None
Additional catkin Make Args: None
Internal Make Job Server:    True
Cache Job Environments:      False
----------------------------------------------------------------
Whitelisted Packages:        None
Blacklisted Packages:        None
----------------------------------------------------------------
Workspace configuration appears valid.
----------------------------------------------------------------
```

## Command Overview of `catkin_tools`

To [create packages](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#adding-packages-to-the-workspace), which will be covered in the next posts in more depth, we will use [`catkin create pkg PKG_NAME`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html#catkin-create-pkg).

Building the workspace is done with [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#building-the-workspace).

```bash
fjp@ubuntu:~/git/2wd-robot/ros$ catkin build
----------------------------------------------------------------
Profile:                     default
Extending:             [env] /opt/ros/melodic
Workspace:                   /home/fjp/git/2wd-robot/ros
----------------------------------------------------------------
Build Space:        [exists] /home/fjp/git/2wd-robot/ros/build
Devel Space:        [exists] /home/fjp/git/2wd-robot/ros/devel
Install Space:      [unused] /home/fjp/git/2wd-robot/ros/install
Log Space:         [missing] /home/fjp/git/2wd-robot/ros/logs
Source Space:       [exists] /home/fjp/git/2wd-robot/ros/src
DESTDIR:            [unused] None
----------------------------------------------------------------
Devel Space Layout:          linked
Install Space Layout:        None
----------------------------------------------------------------
Additional CMake Args:       None
Additional Make Args:        None
Additional catkin Make Args: None
Internal Make Job Server:    True
Cache Job Environments:      False
----------------------------------------------------------------
Whitelisted Packages:        None
Blacklisted Packages:        None
----------------------------------------------------------------
Workspace configuration appears valid.

NOTE: Forcing CMake to run for each package.
----------------------------------------------------------------
[build] No packages were found in the source space '/home/fjp/git/2wd-robot/ros/src'
[build] No packages to be built.
[build] Package table is up to date.
Starting  >>> catkin_tools_prebuild
Finished  <<< catkin_tools_prebuild                [ 10.0 seconds ]
[build] Summary: All 1 packages succeeded!
[build]   Ignored:   None.
[build]   Warnings:  None.
[build]   Abandoned: None.
[build]   Failed:    None.
[build] Runtime: 10.1 seconds total.
```

## Environment Setup

Finally the newly built packages have to be [loaded in the environment](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#loading-the-workspace-environment) using `source`.

```bash
fjp@ubuntu:~/git/2wd-robot/ros$ source ~/git/2wd-robot/ros/devel/setup.bash # Load the workspace's environment
```

!!! tip
    To avoid tediously typing the above `source` command, it is convenient to create an alias in your `~/.bashrc` or `~/.zshrc` similar to the following:
    ```
    alias s='source devel/setup.bash'
    ```
    or using the absolute path
    ```
    alias sa='source ~/git/2wd-robot/ros/devel/setup.bash'
    ```
    It is recommended to use the correct setup script for the shell you use ([`bash`](https://en.wikipedia.org/wiki/Bash_(Unix_shell)), [` zsh`](https://en.wikipedia.org/wiki/Z_shell), etc.).
    In case you are unsure, you can check with the `echo $SHELL` command which will most likely output `/bin/bash`.


!!! info
    Instead of `source` it is possible to use the `.` command instead. Don't confuse it though with the current directory, which is also represented as `.`.

## Resources

Although the [catkin tutorial](https://wiki.ros.org/catkin/Tutorials) uses `catkin_make` it provides a helpful guide to create a workspace
