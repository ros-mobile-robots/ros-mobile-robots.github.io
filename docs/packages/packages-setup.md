# Diffbot ROS Packages

The following describes the easiest way to make use of diffbot's ROS packages inside the [ros-mobile-robots/diffbot](https://github.com/ros-mobile-robots/diffbot)
repository.

The following steps will be performed on both, the workstation/development PC and the single board computer (SBC).

!!! tip "Development PC: use the dev container"
    On the development PC, the [dev container](../development/dev-container.md) does all of the following steps for you, in a Docker image with ROS Noetic, Gazebo and RViz. The steps below are still needed on the robot's SBC.

## Git: clone diffbot repository

After setting up ROS on your workstation PC and the SBC (either [Raspberry Pi 4B](../processing_units/rpi-setup.md) or [Jetson Nano](../processing_units/jetson-nano-setup.md)),
create a ros workspace in your users home folder and clone the [`diffbot` repository]({{ diffbot_repo_url }}). Git is described in [Git and GitHub](../getting-started/git-and-github.md):

```
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/ros-mobile-robots/diffbot.git
```

To use a released version instead of the latest code, clone a tag, for example the latest release, `1.1.0`:

```
git clone --depth 1 --branch 1.1.0 https://github.com/ros-mobile-robots/diffbot.git
```

## Obtain (system) Dependencies

The `diffbot` repository relies on two sorts of dependencies:

- Source (non binary) dependencies from other (git) repositories.
- System dependencies available in the (ROS) Ubuntu package repositories. Also referred to as pre built binaries.


### Source Dependencies

Let's first obtain source dependencies from other repositories. 
To do this the recommended tool to use is [`vcstool`](http://wiki.ros.org/vcstool)
(see also https://github.com/dirk-thomas/vcstool for additional documentation and examples.).

!!! note
    [`vcstool`](http://wiki.ros.org/vcstool) replaces [`wstool`](http://wiki.ros.org/wstool).

The `diffbot` repository has three `.repos` files that list these source dependencies.
They clone into `src/`, so run `vcs import` from the root of the catkin workspace (`~/ros_ws`) and pass in the file for the machine you're on:

| Machine | Command | Clones |
|:--------|:--------|:-------|
| Development PC | `vcs import < src/diffbot/diffbot_dev.repos` | `rplidar_ros`, `remo_description` |
| Remo's SBC | `vcs import < src/diffbot/remo_robot.repos` | `rplidar_ros`, `remo_description` |
| DiffBot's SBC | `vcs import < src/diffbot/diffbot_robot.repos` | [`rplidar_ros`](https://github.com/Slamtec/rplidar_ros) (Slamtec), `raspicam_node` |

`vcs import` reads the YAML file from stdin and clones every repository it lists.

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

Now source the catkin workspace either using the [created alias](../processing_units/ros-setup.md#environment-setup) or the full command for the bash shell:

```
source devel/setup.bash
```

## Examples

Now you are ready to follow the examples listed in the readme.

!!! info
    TODO extend documentation with examples


## Optional Infos

### Manual Dependency Installation

To install a package from source clone (using git) or download the source files from where they are located (commonly hosted on GitHub) into the `src` folder of a ros catkin workspace and execute the [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) command. Also make sure to source the workspace after building new packages with `source devel/setup.bash`.

```console
cd /homw/fjp/git/diffbot/ros/  # Navigate to the workspace
catkin build              # Build all the packages in the workspace
ls build                  # Show the resulting build space
ls devel                  # Show the resulting devel space
```

!!! note

    Make sure to clone/download the source files suitable for the ROS distribution
    you are using. If the sources are not available for the distribution you are
    working with, it is worth to try building anyway. Chances are that the package
    you want to use is suitable for multiple ROS distros. For example if a package
    states in its docs, that it is only available for
    [kinetic](http://wiki.ros.org/kinetic) it is possible that it will work with a
    ROS [noetic](http://wiki.ros.org/noetic) install.
