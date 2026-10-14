# Raspberry Pi Camera

The diffbot robot uses the [`UbiquityRobotics/raspicam_node`](https://github.com/UbiquityRobotics/raspicam_node)
to interface the [Raspberry Pi Camera v2](https://www.raspberrypi.org/products/camera-module-v2/).

<figure>
    <a href="{{ asset_dir }}/components/rpi-camera.jpg"><img src="{{ asset_dir }}/components/rpi-camera.jpg"></a>
    <figcaption>RPi Camera v2.</figcaption>
</figure>


## Setup

Currently there exists  no binary of the `raspicam_node` for ROS noetic (only for kinetic there is the `ros-kinetic-raspicam-node`).
To work with the `raspicam_node` for ROS Noetic, you have to build it from source with steps 
outlined in the build instructions of the [readme](https://github.com/UbiquityRobotics/raspicam_node#build-intructions).
For completeness, these steps are listed here:

First go to your catkin_ws cd `~/catkin_ws/src` and download the source for this node by running

```console
git clone https://github.com/UbiquityRobotics/raspicam_node.git
```

rosdep doesn't know two of its dependencies: the Raspberry Pi's camera libraries `libraspberrypi0` and `libraspberrypi-dev`. Ubuntu 20.04 for the Raspberry Pi has them as packages (in `focal-updates`, universe). The rosdep rules for them that Ubiquity Robotics published (`UbiquityRobotics/rosdep`, `raspberry-pi.yaml`) are no longer online, so add the rules as a local file, and tell rosdep where it is. The first command removes the entry for the old rules that earlier versions of this page added, if it's there: with it, `rosdep update` fails.

```console
sudo rm -f /etc/ros/rosdep/sources.list.d/30-ubiquity.list
sudo tee /etc/ros/rosdep/raspberry-pi.yaml > /dev/null << 'EOF'
libraspberrypi0:
  ubuntu:
    apt:
      packages: [libraspberrypi0]
libraspberrypi-dev:
  ubuntu:
    apt:
      packages: [libraspberrypi-dev]
EOF
echo "yaml file:///etc/ros/rosdep/raspberry-pi.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/30-raspberry-pi.list
```

Then update rosdep's index:

```console
rosdep update --rosdistro noetic
```

Next, we parse the `package.xml` file for required dependencies and also install them with the `rosdep` command:

```console
cd ~/catkin_ws
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
