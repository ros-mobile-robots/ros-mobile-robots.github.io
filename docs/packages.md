# Diffbot ROS Packages

The following describes the easiest way to make use of diffbot's ROS packages inside the [ros-mobile-robots/diffbot](https://github.com/ros-mobile-robots/diffbot)
repository.

The following steps will be performed on both, the workstation PC and the single board computer (SBC).

After setting up ROS on your workstation PC and the SBC (either [Raspberry Pi 4B](https://ros-mobile-robots.com/rpi-setup/) or [Jetson Nano](https://ros-mobile-robots.com/jetson-nano-setup/)),
create a ros workspace in your users home folder and clone the `diffbot` repository:

```
mkdir -p ros_ws/src
git clone https://github.com/ros-mobile-robots/diffbot.git
```

Then it is necessary to install all the needed ROS dependencies which are required by diffbot packages.
The [`rosdep`](http://wiki.ros.org/rosdep) command, which was installed during the ROS setup can be used to install all dependencies with the following command:

```
rosdep install --from-paths src --ignore-src -r -y
```

!!! info
    On the following Packages pages it is explained that the dependencies of a ROS package are defined inside its `package.xml`.
    
 
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
