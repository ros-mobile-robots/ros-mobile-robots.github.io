# Processing Units

Our robot consists of components which are connected as shown in the block diagram below:

[![Block Diagram Remo]][Block Diagram Remo]

  [Block Diagram Remo]: {{ asset_dir }}/processing_units/block-diagram-remo.svg

The Teensy 3.2 microcontroller board is connected to the encoder and optional IMU
sensor as well as the motor driver actuator. It communicates to the Raspberry Pi 4 B via
USB over the `rosserial` protocol. The motor driver and the optional IMU exchange
data over I2C with the microcontroller. The RPLIDAR has a serial-to-USB converter and
is therefore connected to one of the USB ports of the SBC. The motor encoder sensors
are interfaced through the GPIO pins of the microcontroller. The following shows the
connection diagram of the components:


[![Remo Fritzing]][Remo Fritzing]

  [Remo Fritzing]: /fritzing/remo_architecture.svg


## Overview of ROS nodes and topics for the Remo robot

The following launch file will bring up the hardware nodes, load the robot description
onto the parameter server, start `diff_drive_controller`, and begin to publish the
transformations using `tf`. Run this launch file on the robot's SBC:

```console
roslaunch diffbot_bringup bringup.launch model:=remo
```

On the development PC, you can use the teleop node to steer the robot. To do this, run
the following:

```console
roslaunch diffbot_bringup keyboard_teleop.launch
```

Issuing the rosnode list command shows the following list of started nodes:

```console
/diffbot/controller_spawner
/diffbot/diffbot_base
/diffbot/robot_state_publisher
/diffbot/rosserial_base_controller
/diffbot_teleop_keyboard
/rosout
```

[![ROS nodes and topics of Remo]][ROS nodes and topics of Remo]

  [ROS nodes and topics of Remo]: {{ asset_dir }}/processing_units/ros-nodes-remo.svg