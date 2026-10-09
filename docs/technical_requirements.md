# Getting Started

DiffBot and Remo are differential drive robots that run [ROS 1 Noetic](https://wiki.ros.org/noetic). DiffBot is a two- or four-wheeled robot you build yourself; Remo is a modular, 3D printed platform based on NVIDIA's JetBot. The code for both is in the [diffbot](https://github.com/ros-mobile-robots/diffbot) repository. It includes the simulation, the software for the real robot, and the tools to work with it from your PC.

This page shows which steps you need, depending on what you want to do.

## Choose your path

### Simulation

No robot needed: the robot runs in Gazebo on your PC.

1. [Set Up Your PC](getting-started/set-up-your-pc.md): Linux, or Windows with WSL 2, and Docker.
2. <a id="git"></a>[Git and GitHub](getting-started/git-and-github.md): install Git and clone diffbot.
3. [Use the Dev Container](development/dev-container.md): it has ROS Noetic, Gazebo and RViz, and builds the workspace. Then start the simulation.

### Real robot

Do the simulation path first; your PC then works with the robot. In addition:

1. **Hardware:** the [Components](components.md) to buy, and for Remo the [hardware setup](hardware_setup/overview.md): 3D printing, electronics, assembly.
2. <a id="remote-control"></a>**The robot's computer**, a Raspberry Pi 4 B:
    1. [Raspberry Pi Setup](processing_units/rpi-setup.md): Ubuntu MATE 20.04, and an SSH server so you can log in from your PC.
    2. [Git and GitHub](getting-started/git-and-github.md): install Git and clone, without login.
    3. [ROS Setup](processing_units/ros-setup.md): ROS Noetic and catkin tools.
    4. [Packages Setup](packages/packages-setup.md): the workspace with diffbot and its dependencies.

    [Jetson Nano Setup](processing_units/jetson-nano-setup.md) describes an older setup with Ubuntu 18.04 and ROS Melodic.

3. **The firmware** for the [microcontroller](processing_units/teensy-mcu.md), a Teensy, which drives the motors and reads the encoders.
4. [ROS Network Setup](processing_units/ros-network-setup.md): connect your PC and the robot.

### ROS on your PC without the container

If you prefer to install ROS directly on an Ubuntu 20.04 PC: [ROS Setup](processing_units/ros-setup.md), then [Packages Setup](packages/packages-setup.md). The dev container is the recommended way, because it sets up the same environment for everyone.

## What you need

<a id="operating-system"></a>

| | Simulation | Real robot |
|:--|:--|:--|
| **PC** | Linux, for example Ubuntu 24.04; or Windows 10 build 19044 or later, or Windows 11, with WSL 2 | The same. With Windows, connecting to the robot needs Windows 11 22H2 or later ([why](getting-started/set-up-your-pc.md#real-robot-from-windows-optional)) |
| **Robot** | – | DiffBot or Remo parts, a Raspberry Pi 4 B and a Teensy, see [Components](components.md) |
| **3D printing** | – | For Remo: a 3D printer with a build volume of about 15×15×15 cm, or a print service. The STL files come from the [Gumroad download](hardware_setup/3D_print.md) or [Remo Insiders](insiders/index.md#remo-stl-files) |
