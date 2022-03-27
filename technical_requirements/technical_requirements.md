# Technical requirements

In the https://github.com/ros-mobile-robots organization on GitHub
are the required ROS packages to set up a differential drive robot. 
One of the main software repositories is https://github.com/ros-mobile-robots/diffbot.
It includes packages for simulation and the configurations and software to operate a real
robot and interact with it from a development PC. For the hardware, you can build your
own two- or four-wheeled differential drive robot similar to the one present in the 
`diffbot_description` package or 3D print a more stable Remo robot with the stl files in
https://github.com/ros-mobile-robots/remo_description. 

The next two sections describe the technical requirements for the software and hardware.

!!! info
    This technical requirements page is here to give you an overview of what is required to get your robot up
    and running. You can already follow the steps in practice but they will be also
    mentioned in later sections (in more detail) when they are really needed.

## Software requirements

The following sections give an overview about which software will be used:

- [Operating Systems](#operating-system)
- [Git](#git)
- [Remote Control](#remote-control)
- [Hardware Interface](#hardware-interface)
- [Source Dependencies](#source-dependencies)
- [Binary Dependencies](#binary-dependencies)
- [Build ROS Workspace](#build-ros-workspace)

More detailed software setup instructions are found in the chapter about
[Processing Units](#processing-units).

### Operating System

For the development PC, you should have ROS Noetic installed on
[Ubuntu 20.04](https://releases.ubuntu.com/20.04/) or using
[Windows Subsystem for Linux (WSL) 2](https://docs.microsoft.com/en-us/windows/wsl/)
running on **Windows 11**.

!!! note
    [Windows 11 is required for GUI features](https://docs.microsoft.com/en-us/windows/wsl/tutorials/gui-apps),
    such as Gazebo and RViz. WSL 2 on Windows 10 only provides command line support, although you
    can install an X11 server, such as [VcXsrv](https://sourceforge.net/projects/vcxsrv/),
    explained [here](https://jack-kawell.com/2020/06/12/ros-wsl2/).

On the Single Board Computer (SBC) (e.g. Raspberry Pi 4 B) that is mounted on Remo,
we use [Ubuntu Mate 20.04](https://ubuntu-mate.org/download/arm64/focal/) for arm64 architecture.

### Git

As the software is hosted on GitHub which uses git as a version control system it needs to be
present in your used operating system. 

=== "Ubuntu"

    On Ubuntu this is usually the case, which you can check with:

    ```console
    $ git --version
    git version 2.25.1
    ```

=== "Windows"

    On Windows you need to install Git using a package manager such as
    [chocolatey](https://chocolatey.org/) or downloading it from https://git-scm.com/downloads.

To clone large stl files from the Git repository we use [git-lfs](https://git-lfs.github.com/). 
On both Ubuntu flavors it needs to be installed with the following terminal command:

```console
sudo apt install git-lfs
```

### Remote Control

On both the development PC and the SBC of the robot, you need a connection to the
same local network and to enable the ssh protocol, to connect from the development PC
(client) to the robot, which is running an open-ssh server. Install it on Ubuntu Mate
20.04 with the following:

```console
sudo apt install openssh-server
```

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


## Hardware requirements

The repository at https://github.com/ros-mobile-robots/remo_description contains the 
robot description of Remo. Remo is a modular mobile robot platform, which is based on 
NVIDIA's JetBot. The currently available parts can be 3D printed using the provided 
stl files in the [`remo_description`](https://github.com/ros-mobile-robots/remo_description) repository. 
To do this, you either need a 3D printer with a recommended build volume of 15x15x15 cm or to use
a local or online 3D print service. Further details are found in [hardware setup](hardware_setup/3D_print.md).

On the following [components](#components) page you find a bill of materials and more details about each part.