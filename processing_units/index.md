# Processing Units

Our robot consists of components which are connected as shown in the block diagram below:

![Block Diagram Remo]({{ asset_dir }}/processing_units/block-diagram-remo.svg)

The Teensy 3.2 microcontroller board is connected to the encoder and optional IMU
sensor as well as the motor driver actuator. It communicates to the Raspberry Pi 4 B via
USB over the `rosserial` protocol. The motor driver and the optional IMU exchange
data over I2C with the microcontroller. The RPLIDAR has a serial-to-USB converter and
is therefore connected to one of the USB ports of the SBC. The motor encoder sensors
are interfaced through the GPIO pins of the microcontroller. The following shows the
connection diagram of the components:


[![Remo Fritzing]][Remo Fritzing]

  [Remo Fritzing]: /fritzing/remo_architecture.svg