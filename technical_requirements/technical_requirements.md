# Technical requirements

To get started with building a ROS-based mobile robot, this section outlines the essential software and hardware you'll need. We'll explore key resources from the https://github.com/ros-mobile-robots organization on GitHub and provide options for building your own robot or using a pre-designed platform. Later sections will delve deeper into specific software installation and hardware components.

!!! info
    This technical requirements page is here to give you an overview of what is required to get your robot up and running.
    You can already follow the steps in practice but they will be also mentioned in later sections (in more detail) when they are really needed.

## Software requirements

One of the main software repositories is https://github.com/ros-mobile-robots/diffbot.
It includes packages for simulation and the configurations and software to operate a real
robot and interact with it from a development PC (or dev machine).

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

## Hardware requirements

For the hardware, you can build your
own two- or four-wheeled differential drive robot similar to the one present in the 
`diffbot_description` package or 3D print a more stable Remo robot with the stl files in
https://github.com/ros-mobile-robots/remo_description.

The repository at https://github.com/ros-mobile-robots/remo_description contains the 
robot description of Remo. Remo is a modular mobile robot platform, which is based on 
NVIDIA's JetBot. The currently available parts can be 3D printed using the provided 
stl files in the [`remo_description`](https://github.com/ros-mobile-robots/remo_description) repository. 
To do this, you either need a 3D printer with a recommended build volume of 15x15x15 cm or to use
a local or online 3D print service. Further details are found in [hardware setup](hardware_setup/3D_print.md).

On the following [components](#components) page you find a bill of materials and more details about each part.
