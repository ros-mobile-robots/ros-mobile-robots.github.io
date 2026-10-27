# Robot Container

The robot's computer can run ROS Noetic in a [Docker](https://docs.docker.com/get-started/docker-overview/) container instead of a native installation. ROS Noetic needs Ubuntu 20.04, whose standard support ended in May 2025 ([ROS Setup](ros-setup.md)). With the container, the Raspberry Pi runs a current host OS, Ubuntu Server 24.04, and the container brings Ubuntu 20.04 and ROS Noetic along. Ubuntu 24.04 is also the OS of ROS 2 Jazzy, the planned next ROS version for the robots ([diffbot#136](https://github.com/ros-mobile-robots/diffbot/issues/136)), so the SD card doesn't have to be set up again for it.

!!! warning "Experimental: not tested on the robot yet"
    CI builds the image for arm64, the Raspberry Pi's architecture, and for amd64, and tests it without hardware ([How it's tested](#how-its-tested)). It hasn't driven a robot yet: the test with teleop and the lidar on a Raspberry Pi with Remo is still open in [diffbot#109](https://github.com/ros-mobile-robots/diffbot/issues/109). The native setup, [Raspberry Pi Setup](rpi-setup.md) and [ROS Setup](ros-setup.md), stays documented as the alternative.

## What runs where

| Where | What | Set up on |
|:--|:--|:--|
| Your PC | The [dev container](../development/dev-container.md) with ROS Noetic, Gazebo and RViz, and usually the ROS master | [Set Up Your PC](../getting-started/set-up-your-pc.md) |
| The robot's host: the Raspberry Pi with Ubuntu Server 24.04 | Docker Engine, the udev rules for the serial devices, a folder for ROS logs and bags | This page |
| The robot container, on the Raspberry Pi | ROS Noetic and the robot's packages: the bringup, the hardware interface, the controllers, rosserial for the [Teensy](teensy-mcu.md), the robot models and the lidar driver | Built from the diffbot repository |

Each command below says where it runs. The Teensy is still flashed from the PC ([Microcontroller](teensy-mcu.md)): the container only talks to it over its USB serial port.

## What's in the image

The image is defined in the diffbot repository's [`.docker/robot`](https://github.com/ros-mobile-robots/diffbot/tree/noetic-devel/.docker/robot) folder. Its [`Dockerfile`]({{ diffbot_repo_url }}/.docker/robot/Dockerfile) starts from the official [`ros:noetic-ros-base`](https://hub.docker.com/_/ros) image, which exists for arm64 and amd64, and builds the packages that the bringup and the lidar need, for both robots:

- `diffbot_bringup`, `diffbot_base`, `diffbot_control`, `diffbot_description` and `diffbot_msgs` from the diffbot repository;
- `remo_description` and the lidar driver `rplidar_ros`, from [`remo_robot.repos`]({{ diffbot_repo_url }}/remo_robot.repos).

Gazebo, navigation and SLAM aren't in it: they run on the PC. The camera isn't either: the Raspberry Pi camera driver, `raspicam_node`, needs the legacy camera stack, which current OSes no longer have ([diffbot#111](https://github.com/ros-mobile-robots/diffbot/issues/111)). The image is about 3.9 GB on amd64; the official base image is 2.9 GB of that.

The container runs as the user `ros` with UID 1000, not as root, and its workspace is `~/catkin_ws`.

## Set up the robot's host

These steps run once, on the Raspberry Pi. They follow Ubuntu's and Docker's documentation, but haven't been run on a Raspberry Pi for this robot yet.

1. **Ubuntu Server 24.04.** Flash *Ubuntu Server 24.04 LTS (64-bit)* to the microSD card with [Raspberry Pi Imager](https://www.raspberrypi.com/software/), which lists it under the other general-purpose operating systems. In Imager's settings, set your user, the Wi-Fi and SSH, then boot the Raspberry Pi and log in over SSH from your PC. Ubuntu supports the [Raspberry Pi 4 B](https://ubuntu.com/hardware/docs/boards/how-to/ubuntu_supported/raspberry-pi/) with this release.
2. **The clock.** ROS compares time stamps from the robot and the PC, so both clocks have to be synchronized. Ubuntu does that by default with systemd-timesyncd; check that `timedatectl` on the robot shows `System clock synchronized: yes` ([time synchronization](https://documentation.ubuntu.com/server/how-to/networking/timedatectl-and-timesyncd/)).
3. **Docker Engine and Compose.** Ubuntu's own packages are enough, as on the PC ([Set Up Your PC](../getting-started/set-up-your-pc.md#docker)); [Docker's guide](https://docs.docker.com/engine/install/ubuntu/) describes the alternative from Docker's repository. Add your user to the `docker` group and log in again:

    ```console
    sudo apt update
    sudo apt install docker.io docker-compose-v2 git
    sudo usermod -aG docker $USER
    ```

4. **The diffbot repository**, cloned into your home folder:

    ```console
    git clone https://github.com/ros-mobile-robots/diffbot.git
    cd diffbot
    ```

5. **Stable device names.** The kernel numbers serial devices in the order they appear, so the Teensy isn't always `/dev/ttyACM0`. The repository's [udev rules]({{ diffbot_repo_url }}/.docker/robot/99-diffbot-robot.rules) give the Teensy the name `/dev/teensy` and the lidar `/dev/rplidar`, by their USB vendor and product IDs:

    ```console
    sudo cp .docker/robot/99-diffbot-robot.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ls -l /dev/teensy /dev/rplidar
    ```

6. **A folder for ROS logs and bags**, which the container mounts as its `~/.ros`. Create it as your user: the first user on Ubuntu has UID 1000, like `ros` in the container, so the container can write to it.

    ```console
    mkdir -p ~/robot-data
    ```

## Run the bringup

**Settings** go into a file `.docker/robot/.env` in the `diffbot` folder, one `NAME=value` per line. Set the ROS network first if the PC runs the ROS master: start `roscore` in the PC's dev container, and set the two variables here and on the PC as on [ROS Network Setup](ros-network-setup.md).

| Setting | Default | What it's for |
|:--|:--|:--|
| `ROBOT_MODEL` | `remo` | `diffbot` runs DiffBot's bringup; the image has both models |
| `ROS_MASTER_URI`, `ROS_IP` | not set | The PC's ROS master and the robot's own IP address. Without them, roslaunch starts a master on the robot |
| `TEENSY_DEVICE`, `LIDAR_DEVICE` | `/dev/teensy`, `/dev/rplidar` | The devices on the host |
| `DIALOUT_GID` | `20` | The numeric ID of the host's group that owns the serial devices, `dialout` on Ubuntu |
| `ROBOT_DATA` | `~/robot-data` | The folder for logs and bags |

The container uses the host's network (`network_mode: host`), so ROS reaches the PC as a native installation would.

**Start the bringup** on the Raspberry Pi, in the `diffbot` folder. The first start builds the image on the Raspberry Pi, which compiles the packages and takes a while; later starts reuse it.

```console
docker compose -f .docker/robot/compose.yaml up --build
```

This runs Remo's bringup ([`compose.yaml`]({{ diffbot_repo_url }}/.docker/robot/compose.yaml)): the hardware interface, the controllers and rosserial to the Teensy, which drives the motors and reads the encoders. To start the lidar too, use this command instead; it adds the lidar's profile:

```console
docker compose -f .docker/robot/compose.yaml --profile lidar up
```

Leave this terminal open: it shows the bringup's output, and `Ctrl+C` there stops the containers. For everything else, open a second SSH session to the Raspberry Pi from your PC and go to the `diffbot` folder (`cd ~/diffbot`).

**ROS commands in the running container** go through its entry point, which loads ROS and the workspace. In the second session, for example, list the topics, or drive the robot with the keyboard:

```console
docker compose -f .docker/robot/compose.yaml exec bringup /entrypoint.sh rostopic list
docker compose -f .docker/robot/compose.yaml exec bringup /entrypoint.sh \
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/diffbot/mobile_base_controller/cmd_vel
```

The bringup's output is also available later, with `docker compose -f .docker/robot/compose.yaml logs -f bringup`, and ROS's log files are in `~/robot-data/log`. `docker compose -f .docker/robot/compose.yaml down` removes the stopped containers.

**After the Teensy was reset, reflashed or unplugged,** stop the containers and start them again (`down`, then `up`). A container gets its devices when it starts, and a reconnected Teensy is a new device for the kernel. A restart also starts a new odometry, so a measurement run ends there.

## How it's tested

The workflow [`robot_container.yml`]({{ diffbot_repo_url }}/.github/workflows/robot_container.yml) builds the image on GitHub's arm64 and amd64 runners whenever the image or its packages change, and runs [`smoke_test.sh`]({{ diffbot_repo_url }}/.docker/robot/smoke_test.sh) in it twice: directly in the image, and through `compose.yaml` with its mounts and groups. The test checks that:

- the packages and the programs that talk to the hardware are there;
- both robot models expand with xacro;
- the launch files parse;
- Remo's bringup starts without hardware: the robot model loads and both controllers run. Without a Teensy, rosserial can't open its port and the hardware interface gets no wheel feedback, so the test can't show that the robot drives.

That needs the real robot: teleop and lidar scans on a Raspberry Pi with Remo, the open test in [diffbot#109](https://github.com/ros-mobile-robots/diffbot/issues/109). [Testing and CI](../development/ci.md) lists all workflows.

## Not yet

- **The camera**, see [What's in the image](#whats-in-the-image).
- **Starting at boot.** Start the bringup by hand for now; an automatic start needs the behavior at a reset or a missing device checked first.
- **A smaller image**, without the desktop packages that the robot model packages still declare.
