# Remo Description

ROS URDF description package of REMO robot (Research Education Mobile/Modular robot) a highly modifiable and extendable
autonomous mobile robot based on [Nvidia's Jetbot](https://github.com/NVIDIA-AI-IOT/jetbot).
This ROS package is found in the [`remo_description` repository]({{ remo_repo_url }}) contains the stl files to 3D print Remo robot.

![https://media.githubusercontent.com/media/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/remo/remo-rviz-spin.gif](https://media.githubusercontent.com/media/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/remo/remo-rviz-spin.gif)

## Usage

This is a ROS package which should be cloned in a catkin workspace.
To use `remo_description` inside a Gazebo simulation or on a real 3D printed Remo robot, you can directly make use of the ROS packages in the
[ros-mobile-robots/diffbot]({{ diffbot_repo_url }}) repository.
Most of the launch files you find in the `diffbot` repository
accept a `model` argument. Just append `model:=remo` to the end of a `roslaunch` command to make use of this `remo_description` package.

## Camera Types

The [`remo.urdf.xacro`]({{ remo_repo_url }}/urdf/remo.urdf.xacro) accepts a `camera_type`
[xacro arg](http://wiki.ros.org/xacro#Rospack_commands) which lets you choose between the following different camera types

| Raspicam v2 with IMX219 | OAK-1 | OAK-D |
|:-----------------------:|:-----:|:-----:|
| [<img src="{{ asset_dir }}/remo/camera_types/raspi-cam.png" width="700">]({{ asset_dir }}/remo/camera_types/raspi-cam.png) | [<img src="{{ asset_dir }}/remo/camera_types/oak-1.png" width="700">]({{ asset_dir }}/remo/camera_types/oak-1.png) | [<img src="{{ asset_dir }}/remo/camera_types/oak-d.png" width="700">]({{ asset_dir }}/remo/camera_types/oak-d.png) |

## Single Board Computer Types

Another xacro argument is the `sbc_type` wher you can select between `jetson` and `rpi`.

| Jetson Nano | Raspberry Pi 4 B |
|:-----------------------:|:-----:|:-----:|
| [<img src="{{ asset_dir }}/remo/sbc_types/jetson-nano.png" width="700">]({{ asset_dir }}/remo/sbc_types/jetson-nano.png) | [<img src="{{ asset_dir }}/remo/sbc_types/raspi.png" width="700">]({{ asset_dir }}/remo/sbc_types/raspi.png) |


## :handshake: Acknowledgment

- [Louis Morandy-Rapiné](https://louisrapine.com/) for his great work on REMO robot and designing it in [Fusion 360](https://www.autodesk.com/products/fusion-360/overview).

## References

- [Nvidia Jetbot](https://github.com/NVIDIA-AI-IOT/jetbot)