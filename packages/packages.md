# Diffbot ROS Packages

The following describes the easiest way to make use of diffbot's ROS packages inside the [ros-mobile-robots/diffbot](https://github.com/ros-mobile-robots/diffbot)
repository.

The following steps will be performed on both, the workstation/development PC and the single board computer (SBC).

## Git: clone diffbot repository

After setting up ROS on your workstation PC and the SBC (either [Raspberry Pi 4B](https://ros-mobile-robots.com/rpi-setup/) or [Jetson Nano](https://ros-mobile-robots.com/jetson-nano-setup/)),
create a ros workspace in your users home folder and clone the [`diffbot` repository]({{ diffbot_repo_url }}):

```
mkdir -p ros_ws/src
git clone https://github.com/ros-mobile-robots/diffbot.git
```

## Obtain (system) Dependencies

The `diffbot` repository relies on two sorts of dependencies:

- Source (non binary) dependencies from other (git) repositories.
- System dependencies available in the (ROS) Ubuntu package repositories. Also refered to as pre built binaries.


### Source Dependencies

Let's first obtain source dependencies from other repositories. 
To do this the recommended tool to use is [`vcstool`](http://wiki.ros.org/vcstool)
(see also https://github.com/dirk-thomas/vcstool for additional documentation and examples.).

!!! note
    [`vcstool`](http://wiki.ros.org/vcstool) replaces [`wstool`](http://wiki.ros.org/wstool).

Inside the cloned `diffbot` repository, 
make use of the `import` command and the `diffbot.repos` file containing the required source repositories:

```
vcs import < diffbot.repos
```

This will clone all repositories which are stored in the `diffbot.repos` that get passed in via stdin in YAML format.

!!! note
    The file `diffbot.repos` contains relative paths and will clone the listed repositories in the parent folder from where
    the `vcs import` command is called. When it is called from inside the `diffbot` repository, which should be located
    in the `src` folder of a catkin workspace, then the other repositories are also cloned in the `src` folder.

For the SBC not all dependencies in `diffbot.repos` are needed.
Instead the `diffbot_robot.repos` is here to clone the [`rplidar_ros`](https://github.com/Slamtec/rplidar_ros) repository.

```
vcs import < diffbot_robot.repos
```

Now that additional packages are inside the catkin workspace it is time to install the system dependencies.

### System Dependencies

All the needed ROS system dependencies which are required by diffbot's packages can be installed using
[`rosdep`](http://wiki.ros.org/rosdep) command, which was installed during the ROS setup.
To install all system dependencies use the following command:

```
rosdep install --from-paths src --ignore-src -r -y
```

!!! info
    On the following packages pages it is explained that the dependencies of a ROS package are defined inside its `package.xml`.
    
 
After the installation of all dependencies finished (which can take a while), it is time to build the catkin workspace. 
Inside the workspace use [`catkin-tools`](https://catkin-tools.readthedocs.io/en/latest/) to build the packages inside the `src` folder.

!!! note
    The first time you run the following command, make sure to execute it inside your catkin workspace and not the `src` directory.
    
```
catkin build
```

Now source the catkin workspace either using the [created alias](ros-setup.md#environment-setup) or the full command for the bash shell:

```
source devel/setup.bash
```

## Examples

Now you are ready to follow the examples listed in the readme.

!!! info
    TODO extend documentation with examples
