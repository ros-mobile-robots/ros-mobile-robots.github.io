## Raspberry Pi Camera

The diffbot robot uses the [`UbiquityRobotics/raspicam_node`](https://github.com/UbiquityRobotics/raspicam_node)
to interface the [Raspberry Pi Camera v2](https://www.raspberrypi.org/products/camera-module-v2/).

To publish the camera image on the /raw/image/compressed topic run the following on the robot:

```console
roslaunch raspicam_node camerav2_410x308_30fps.launch
```

Running `rostopic list` should show the topics of raspicam node::

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