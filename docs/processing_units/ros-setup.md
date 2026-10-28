# ROS Installation

The robot's computer runs [ROS Noetic](https://wiki.ros.org/noetic) on Ubuntu 20.04 Focal Fossa ([Raspberry Pi Setup](rpi-setup.md)). ROS Noetic reached its end of life in May 2025, at the same time as the standard support of [Ubuntu 20.04](https://ubuntu.com/20-04); Ubuntu MATE's own support for 20.04 [ended in April 2023](https://ubuntu-mate.community/t/ubuntu-mate-20-04-lts-reaches-end-of-life/26477). This project still uses them, and the ROS package server keeps the final Noetic packages, so the installation below works. The experimental [Robot Container](robot-container/index.md) runs Noetic in Docker on a newer host OS instead; the way to ROS 2 is in [diffbot#107](https://github.com/ros-mobile-robots/diffbot/issues/107).

!!! tip "All in one for Remo: `raspi-setup.sh`"
    The diffbot repository's [`raspi-setup.sh`](https://github.com/ros-mobile-robots/diffbot/blob/noetic-devel/raspi-setup.sh) runs the commands of this page and of [Packages Setup](../packages/packages-setup.md) in one go, for Remo. On the robot:

    ```console
    curl -O https://raw.githubusercontent.com/ros-mobile-robots/diffbot/noetic-devel/raspi-setup.sh
    bash raspi-setup.sh
    ```

    The sections below explain what the script does. [Hardware Interfaces](hardware-interfaces.md) is still needed: it gives your user access to I2C and the serial port. For DiffBot, follow the pages: its camera driver needs the extra steps on [Raspberry Pi Camera](../packages/raspicam_node.md), which the script doesn't do yet.

## Install ROS Noetic

The key that signs the ROS packages expired on 1 June 2025 and was renewed. The [`ros-apt-source`](https://github.com/ros-infrastructure/ros-apt-source) package sets up the package source together with the current key, as the [ROS signing key migration guide](https://discourse.openrobotics.org/t/ros-signing-key-migration-guide/43937) recommends.

??? warning "Robot set up before June 2025? Remove the old package source first"
    Older instructions, including the [ROS wiki](https://wiki.ros.org/noetic/Installation/Ubuntu) and earlier versions of this page, add the package source to `/etc/apt/sources.list.d/ros-latest.list` and the key with `apt-key`. If your robot has that file, `sudo apt update` fails because the key from back then has expired (`EXPKEYSIG`). Remove the old entry and key before the commands below, as the [migration guide](https://discourse.openrobotics.org/t/ros-signing-key-migration-guide/43937) describes:

    ```console
    sudo rm /etc/apt/sources.list.d/ros-latest.list
    sudo apt-key del "C1CF 6E31 E6BA DE88 68B1 72B4 F42E D6FB AB17 C654"
    ```

Download and install `ros-apt-source`:

```console
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros-apt-source.deb
```

The second command asks GitHub for the latest version of `ros-apt-source`. GitHub answers 60 such requests per hour from one IP address, for example for a classroom behind one router. If `dpkg` reports that the file isn't a Debian archive, try again later, or set the version by hand, for example `export ROS_APT_SOURCE_VERSION=1.3.0`, and repeat the last two commands.

Then install ROS for the robot, `ros-noetic-robot`, without the GUI tools: RViz, Gazebo and rqt run on your PC, in the [dev container](../development/dev-container.md). Install the tools to set up and build the workspace with it, among them the C++ compiler (`build-essential`), and `python-is-python3`, which nodes written with `rospy` need:

```console
sudo apt update
sudo apt install ros-noetic-robot python3-rosdep python3-catkin-tools python3-vcstool build-essential python-is-python3
```

Load ROS in every new terminal:

```console
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Dependencies: rosdep

[`rosdep`](https://wiki.ros.org/rosdep) installs the system dependencies that ROS packages list in their `package.xml`. Set it up once:

```console
sudo rosdep init
rosdep update --rosdistro noetic
```

Noetic is past its end of life, so a plain `rosdep update` skips it, and `rosdep install` then can't find the ROS packages. `--rosdistro noetic` includes it.

The dependencies of diffbot's packages are installed later, once they are in the workspace: see [Packages Setup](../packages/packages-setup.md#system-dependencies).

## Build Tool: `catkin_tools`

To work with ROS we will use [`catkin_tools`](https://catkin-tools.readthedocs.io/en/latest/index.html) 
instead of [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make). `catkin_tools` provide commands such as [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) which we will use instead of [`catkin_make`](https://wiki.ros.org/catkin/commands/catkin_make) because the `catkin_tools` are more actively developed than `catkin_make`
[ref](https://robotics.stackexchange.com/questions/16604/ros-catkin-make-vs-catkin-build).

!!! note
    It is recommended to use `catkin_tools` instead of the default `catkin` when building ROS workspaces. 
    `catkin_tools` provides a number of benefits over regular `catkin_make` and will be used in the documentation. 
    All packages can be built using `catkin_make` however: use `catkin_make` in place of `catkin build` where appropriate.

`catkin_tools` came with the `python3-catkin-tools` package above. Create and initialize a workspace, `catkin_ws`, with the [commands from the catkin_tools quick start](https://catkin-tools.readthedocs.io/en/latest/quick_start.html). ROS is already loaded through `~/.bashrc`:


```bash
fjp@ubuntu:~$ mkdir -p ~/catkin_ws/src           # Make a new workspace and source space
fjp@ubuntu:~$ cd ~/catkin_ws                     # Navigate to the workspace root
fjp@ubuntu:~/catkin_ws$ catkin init                         # Initialize it with a hidden marker file
Initializing catkin workspace in `/home/fjp/catkin_ws`.
----------------------------------------------------------------
Profile:                     default
Extending:             [env] /opt/ros/noetic
Workspace:                   /home/fjp/catkin_ws
----------------------------------------------------------------
Build Space:       [missing] /home/fjp/catkin_ws/build
Devel Space:       [missing] /home/fjp/catkin_ws/devel
Install Space:      [unused] /home/fjp/catkin_ws/install
Log Space:         [missing] /home/fjp/catkin_ws/logs
Source Space:       [exists] /home/fjp/catkin_ws/src
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
fjp@ubuntu:~/catkin_ws$ catkin build
----------------------------------------------------------------
Profile:                     default
Extending:             [env] /opt/ros/noetic
Workspace:                   /home/fjp/catkin_ws
----------------------------------------------------------------
Build Space:        [exists] /home/fjp/catkin_ws/build
Devel Space:        [exists] /home/fjp/catkin_ws/devel
Install Space:      [unused] /home/fjp/catkin_ws/install
Log Space:         [missing] /home/fjp/catkin_ws/logs
Source Space:       [exists] /home/fjp/catkin_ws/src
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
[build] No packages were found in the source space '/home/fjp/catkin_ws/src'
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
fjp@ubuntu:~/catkin_ws$ source ~/catkin_ws/devel/setup.bash # Load the workspace's environment
```

!!! tip
    To avoid tediously typing the above `source` command, it is convenient to create an alias in your `~/.bashrc` or `~/.zshrc` similar to the following:
    ```
    alias s='source devel/setup.bash'
    ```
    or using the absolute path
    ```
    alias sa='source ~/catkin_ws/devel/setup.bash'
    ```
    It is recommended to use the correct setup script for the shell you use ([`bash`](https://en.wikipedia.org/wiki/Bash_(Unix_shell)), [` zsh`](https://en.wikipedia.org/wiki/Z_shell), etc.).
    In case you are unsure, you can check with the `echo $SHELL` command which will most likely output `/bin/bash`.


!!! info
    Instead of `source` it is possible to use the `.` command instead. Don't confuse it though with the current directory, which is also represented as `.`.

Next, on the robot: [Hardware Interfaces](hardware-interfaces.md), for access to USB, GPIO and I2C.

## Resources

Although the [catkin tutorial](https://wiki.ros.org/catkin/Tutorials) uses `catkin_make` it provides a helpful guide to create a workspace
