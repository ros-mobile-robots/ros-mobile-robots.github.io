# Raspberry Pi Camera

The diffbot robot uses the [`UbiquityRobotics/raspicam_node`](https://github.com/UbiquityRobotics/raspicam_node)
to interface the [Raspberry Pi Camera v2](https://www.raspberrypi.org/products/camera-module-v2/).

![RPi Camera v2]({{ asset_dir }}/components/rpi-camera.jpg)


## Setup

Currently there exists  no binary of the `raspicam_node` for ROS noetic (only for kinetic there is the `ros-kinetic-raspicam-node`).
To work with the `raspicam_node` for ROS Noetic, you have to build it from source with steps 
outlined in the build intstructions of the [readme](https://github.com/UbiquityRobotics/raspicam_node#build-intructions).
For completeness, these steps are listed here:

First go to your catkin_ws cd `~/ros_ws/src` and download the source for this node by running

```console
git clone https://github.com/UbiquityRobotics/raspicam_node.git
```

There are some dependencies that are not recognized by ROS, so you need to create the file `/etc/ros/rosdep/sources.list.d/30-ubiquity.list` and add 
the following to it:

```
yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml
```

This will add ROS dependency sources, which are relevant for the Raspberry Pi. 

For completeness you can list the content of this yaml file here:

<details>
<summary>Show content</summary>

```yaml
libraspberrypi0:
  debian:
    apt:
      packages: [libraspberrypi0]
  ubuntu:
    apt:
      packages: [libraspberrypi0]
libraspberrypi-dev:
  debian:
    apt:
      packages: [libraspberrypi-dev]
  ubuntu:
    apt:
      packages: [libraspberrypi-dev]
libpigpio:
  debian:
    apt:
      packages: [libpigpio-dev]
  ubuntu:
    apt:
      packages: [libpigpio-dev]
libpigpiod-if:
  debian:
    apt:
      packages: [libpigpiod-if-dev]
  ubuntu:
    apt:
      packages: [libpigpiod-if-dev]
```
</details>


To make use of these sources run 

```console
rosdep update
```

Next, we parse the `package.xml` file for required dependencies and also install them with the `rosdep` command:

```console
cd ~/ros_ws
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
```

Finally, you can compile the code with the `catkin build` command from [`catkin-tools`](https://catkin-tools.readthedocs.io/en/latest/) or 
use the legacy [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make) command.

## Running the node

To publish the camera image on the `/raw/image/compressed` topic run the following on the SBC of the robot:

```console
roslaunch raspicam_node camerav2_410x308_30fps.launch
```

Running `rostopic list` should show the topics of raspicam node:

```console
/raspicam_node/camera_info
/raspicam_node/image/compressed
/raspicam_node/parameter_descriptions
/raspicam_node/parameter_updates
```


To view the compressed image run the following command on your development PC:

```
rosrun image_view image_view image:=/raspicam_node/image _image_transport:=compressed
```

This will open up a window where you can see the camera image. If it is dark, check if you removed the lense protection ;-)

!!! note
    There is also a [`rpicamera.launch`](https://github.com/ros-mobile-robots/diffbot/blob/noetic-devel/diffbot_bringup/launch/rpicamera.launch)
    in the `diffbot_bringup` package which you can make use of:
    
    ```console
    roslaunch diffbot_bringup rpicamera.launch
    ```
