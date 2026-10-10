# Remo Description

ROS URDF description package of REMO robot (Research Education Mobile/Modular robot) a highly modifiable and extendable
autonomous mobile robot based on [Nvidia's Jetbot](https://github.com/NVIDIA-AI-IOT/jetbot).
This ROS package is in the [`remo_description` repository]({{ remo_repo_url }}). It describes Remo for RViz and Gazebo; the STL files to 3D print it are described [below](#stl-files).

![Remo spinning in RViz]({{ asset_dir }}/remo/remo-rviz-spin.gif)

You can explore the model in more detail through the following Fusion 360 viewer:

<iframe data-consent-src="https://myhub.autodesk360.com/ue2da69dd/g/shares/SH56a43QTfd62c1cd96877645745238409cb?mode=embed" width="800" height="600" allowfullscreen="true" webkitallowfullscreen="true" frameborder="0"></iframe>

## Usage

This is a ROS package which should be cloned in a catkin workspace.
To use `remo_description` inside a Gazebo simulation or on a real 3D printed Remo robot, you can directly make use of the ROS packages in the
[ros-mobile-robots/diffbot]({{ diffbot_repo_url }}) repository.
Most of the launch files you find in the `diffbot` repository
accept a `model` argument. Just append `model:=remo` to the end of a `roslaunch` command to make use of this `remo_description` package.

### STL files

The public `remo_description` repository has empty placeholder STL files, so Remo has no meshes in RViz and Gazebo, and there is nothing to print. There are two ways to get the real files:

- **Gumroad download:** buy the STL files, then download them with the configuration file as described in the repository's [README](https://github.com/ros-mobile-robots/remo_description#stl-mesh-files).

    <a class="gumroad-button" href="https://gumroad.com/l/GnMpU?wanted=true" data-gumroad-single-product="true">Access Remo STL files</a>

- **Remo Insiders:** a private version of this repository that contains the STL files, stored with Git LFS. See [Remo STL files](../insiders/index.md#remo-stl-files).

Which parts to print and how: [3D Printing](../hardware_setup/3D_print.md).

### Assembly

For assembly instructions please watch the video below:

[![remo fusion animation]({{ asset_dir }}/remo/remo_fusion_animation.gif)](https://youtu.be/6aAEbtfVbAk)

## Camera Types

The [`remo.urdf.xacro`]({{ remo_repo_url }}/urdf/remo.urdf.xacro) accepts a `camera_type`
[xacro arg](http://wiki.ros.org/xacro#Rospack_commands) which lets you choose between the following different camera types

| Raspicam v2 with IMX219 | OAK-1 | OAK-D |
|:-----------------------:|:-----:|:-----:|
| [<img src="{{ asset_dir }}/remo/camera_types/raspi-cam.png" width="700">]({{ asset_dir }}/remo/camera_types/raspi-cam.png) | [<img src="{{ asset_dir }}/remo/camera_types/oak-1.png" width="700">]({{ asset_dir }}/remo/camera_types/oak-1.png) | [<img src="{{ asset_dir }}/remo/camera_types/oak-d.png" width="700">]({{ asset_dir }}/remo/camera_types/oak-d.png) |

## Single Board Computer Types

Another xacro argument is the `sbc_type` where you can select between `jetson` and `rpi`.

| Jetson Nano | Raspberry Pi 4 B |
|:-----------------------:|:-----:|
| [<img src="{{ asset_dir }}/remo/sbc_types/jetson-nano.png" width="700">]({{ asset_dir }}/remo/sbc_types/jetson-nano.png) | [<img src="{{ asset_dir }}/remo/sbc_types/raspi.png" width="700">]({{ asset_dir }}/remo/sbc_types/raspi.png) |


## :handshake: Acknowledgment

- [Louis Morandy-Rapiné](https://louisrapine.com/) for his great work on REMO robot and designing it in [Fusion 360](https://www.autodesk.com/products/fusion-360/overview).

## References

- [Nvidia Jetbot](https://github.com/NVIDIA-AI-IOT/jetbot)
