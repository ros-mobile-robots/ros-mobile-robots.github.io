# Remo Description

ROS URDF description package of REMO robot (Research Education Mobile/Modular robot) a highly modifiable and extendable
autonomous mobile robot based on [Nvidia's Jetbot](https://github.com/NVIDIA-AI-IOT/jetbot).
This ROS package is found in the [`remo_description` repository]({{ remo_repo_url }}) contains the stl files to 3D print Remo robot.

![https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/remo/remo-rviz-spin.gif](https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/remo/remo-rviz-spin.gif)

You can explore the model in more detail through the following Fusion 360 viewer:

<iframe src="https://myhub.autodesk360.com/ue2da69dd/g/shares/SH56a43QTfd62c1cd96877645745238409cb?mode=embed" width="640" height="480" allowfullscreen="true" webkitallowfullscreen="true" frameborder="0"></iframe>

## Usage

This is a ROS package which should be cloned in a catkin workspace.
To use `remo_description` inside a Gazebo simulation or on a real 3D printed Remo robot, you can directly make use of the ROS packages in the
[ros-mobile-robots/diffbot]({{ diffbot_repo_url }}) repository.
Most of the launch files you find in the `diffbot` repository
accept a `model` argument. Just append `model:=remo` to the end of a `roslaunch` command to make use of this `remo_description` package.

### Git LFS and Bandwith Quota

The binary stl files are hosted on GitHub using [Git Large File Storage (git lfs)](https://git-lfs.github.com/) 
to avoid increasing the total size of the repository because of possible stl file changes.
For open source repositories, GitHub has a bandwith limit of 1 GB (up to 1.5 GB) per month. 
Depending on how many users clone/pull the stl files using git lfs per month, this bandwith can be exhausted after a few days. 
If you are not able to clone/pull the stl files and only get the pointer files, you have to wait until the bandwith quota resets back to zero. 
In case you need the stl files immediately, and to support this work you can get [immediate access to the stl files](https://gumroad.com/l/GnMpU?wanted=true):

<a class="gumroad-button" href="https://gumroad.com/l/GnMpU?wanted=true" data-gumroad-single-product="true">Access Remo STL files</a>

Also if you find this work useful please consider the funding options to support the development and design of this robot.
However, you will always be able to clone/pull and use the Remo stl files once the bandwith quota resets.

### Assembly

For assembly instructions please watch the video below:

[![remo fusion animation](https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/remo/remo_fusion_animation.gif)](https://youtu.be/6aAEbtfVbAk)

## Camera Types

The [`remo.urdf.xacro`]({{ remo_repo_url }}/urdf/remo.urdf.xacro) accepts a `camera_type`
[xacro arg](http://wiki.ros.org/xacro#Rospack_commands) which lets you choose between the following different camera types

| Raspicam v2 with IMX219 | OAK-1 | OAK-D |
|:-----------------------:|:-----:|:-----:|
| [<img src="{{ asset_dir }}/remo/camera_types/raspi-cam.png" width="700">]({{ asset_dir }}/remo/camera_types/raspi-cam.png) | [<img src="{{ asset_dir }}/remo/camera_types/oak-1.png" width="700">]({{ asset_dir }}/remo/camera_types/oak-1.png) | [<img src="{{ asset_dir }}/remo/camera_types/oak-d.png" width="700">]({{ asset_dir }}/remo/camera_types/oak-d.png) |

## Single Board Computer Types

Another xacro argument is the `sbc_type` wher you can select between `jetson` and `rpi`.

| Jetson Nano | Raspberry Pi 4 B |
|:-----------------------:|:-----:|
| [<img src="{{ asset_dir }}/remo/sbc_types/jetson-nano.png" width="700">]({{ asset_dir }}/remo/sbc_types/jetson-nano.png) | [<img src="{{ asset_dir }}/remo/sbc_types/raspi.png" width="700">]({{ asset_dir }}/remo/sbc_types/raspi.png) |


## :handshake: Acknowledgment

- [Louis Morandy-Rapiné](https://louisrapine.com/) for his great work on REMO robot and designing it in [Fusion 360](https://www.autodesk.com/products/fusion-360/overview).

## References

- [Nvidia Jetbot](https://github.com/NVIDIA-AI-IOT/jetbot)
