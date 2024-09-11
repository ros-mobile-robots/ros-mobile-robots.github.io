## Single Board Computer Software Preparation

This section guides you through preparing the software environment on your Single Board Computer (SBC), such as a Raspberry Pi, to operate your ROS-based mobile robot. Here, we'll cover essential steps like:

- [Hardware Interface Configuration](#hardware-interface): Setting up the necessary interface between your SBC and the robot's microcontroller.
- [Source Dependency Management](#source-dependencies): Downloading and installing the required software source code using tools like vcstool.
- [Binary Dependency Installation](#binary-dependencies): Installing additional software packages necessary for compiling the robot's ROS packages.
- [Building the ROS Workspace](#build-ros-workspace): Compiling all the software components on the SBC for your robot to function properly.

Following these steps will prepare your SBC's software environment to interact with the robot and run ROS effectively.

!!! note
    This section assumes you have already installed the appropriate operating system (e.g., Ubuntu Mate 20.04) on your SBC.
    See the previous pages ([Raspberry Pi setup](./rpi-setup.md) or [Jetson Nano setup](./jetson-nano-setup.md)) on how to do this.

### Hardware Interface

Another interface setup that is needed to work with the microcontroller, is to add your
user to the `dialout` group on both machines, the SBC and the development PC. This can
be done with the following command, followed by a system reboot:

```console
sudo adduser <username> dialout
```

### Source Dependencies

When you clone the diffbot repository in a new catkin workspace, you will find two
YAML files, `diffbot_dev.repos` and `remo_robot.repos`, that list required source
dependencies together with their version control type, the repository address, and a
relative path where these dependencies are cloned. `remo_robot.repos` is here to clone
source dependencies on the real robot.

To make use of such YAML files and clone the listed dependencies, we use the commands
from [`vcstool`](http://wiki.ros.org/vcstool), which replaces
[`wstool`](http://wiki.ros.org/wstool):

1. Install vcstool using the command:

    ```console
    sudo apt install python3-vcstool
    ```

2. In a new catkin workspace, clone the diffbot repository inside the src folder:

    ```console
    ros_ws/src$ git clone https://github.com/ros-mobile-robots/diffbot.git
    ```

    for a specific tag (e.g. 1.0.0) you can use the following command:

    ```console
    ros_ws/src$ git clone --depth 1 --branch 1.0.0 https://github.com/ros-mobile-robots/diffbot.git
    ```

3. Make sure to execute the `vcs import` command from the root of the catkin
workspace and pipe in the `diffbot_dev.repos` or `remo_robot.repos`
YAML file, depending on where you execute the command, either the development
PC or the SBC of Remo to clone the listed dependencies:

    ```console
    vcs import < src/diffbot/diffbot_dev.repos
    ```

4. Execute the next command on the SBC of the robot:
    
    ```console
    vcs import < src/diffbot/remo_robot.repos
    ```

### Binary Dependencies

After obtaining the source dependencies with `vcstool`, we can compile the workspace.
To successfully compile the packages of the repository, binary dependencies must be
installed. As the required dependencies are specified in each ROS package's `package.xml`, 
the rosdep command can install the required ROS packages from the Ubuntu repositories:

```console
rosdep install --from-paths src --ignore-src -r -y
```

### Build ROS Workspace

Finally, the workspaces on the development machine and the SBC of the robot need to
be built, either using `catkin_make` or catkin tools. `catkin_make` comes pre-installed with ROS.

=== "catkin tools"

    ```console
    catkin build
    ```

=== "`catkin_make`"
    
    ```console
    catkin_make
    ```
