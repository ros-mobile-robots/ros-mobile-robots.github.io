## Grove - I2C Motor Driver V1.3

The package for the [Grove I2C Motor Driver V1.3](http://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/) from Seeed Studio is created with [`catkin create pkg PKG_NAME [--catkin-deps [DEP [DEP ...]]]`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html#catkin-create-pkg):

```console
fjp@ubuntu:~/git/diffbot/ros/src$ catkin create pkg grove_motor_driver --catkin-deps rospy roscpp geometry_msgs
Creating package "grove_motor_driver" in "/home/fjp/git/diffbot/ros/src"...
Created file grove_motor_driver/CMakeLists.txt
Created file grove_motor_driver/package.xml
Created folder grove_motor_driver/include/grove_motor_driver
Created folder grove_motor_driver/src
Successfully created package files in /home/fjp/git/diffbot/ros/src/grove_motor_driver.
```

The package depends on the two ROS [client libraries](http://wiki.ros.org/Client%20Libraries) [`rospy`](http://wiki.ros.org/rospy) and [`roscpp`](http://wiki.ros.org/roscpp). 
To control the two motors the package will use the [`geometry_msgs/Twist`](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) message.
The interface to the motor driver is done with the [Python library from DexterInd](https://github.com/DexterInd/GrovePi/blob/master/Software/Python/grove_i2c_motor_driver) 
which is a rewerite of the [Seeed Studio Arduino library](https://github.com/Seeed-Studio/Grove_I2C_Motor_Driver_v1_3/blob/master).


This library requires the following two python libraries

- [RPi.GPIO](https://pypi.org/project/RPi.GPIO/)
- [smbus](https://pypi.org/project/smbus/) SMBus (System Management Bus) is a subset from the I2C protocol

These libraries should be installed with `pip3`, Python's package manager:

```console
pip3 install RPi.GPIO
pip3 install smbus
```

!!! note
    Note that this will install these packages system wide. This is ok because they are installed on the Raspberry Pi which is dedicated to
    operate for this purpose. For a development environment it is best practice to use a python virtual environment like
    [`venv`](https://docs.python.org/3/library/venv.html) and install the packages inside it.

### Connection

DiffBot's Raspberry Pi 4 B sends the motor commands on its I2C bus 1, through a passive [Grove I2C hub](https://wiki.seeedstudio.com/Grove-I2C_Hub/).
Select DiffBot on the [Electronics](hardware_setup/electronics.md) page to trace the cable through the Grove hub.
At the Raspberry Pi header, the drawing shows SDA on pin 4 (5 V) and SCL on pin 6 (ground).
Use the table below for the header connections.
Connect SDA to physical pin 3 (GPIO2) and SCL to physical pin 5 (GPIO3).
Shut down the Raspberry Pi, unplug its power bank and take out the motor cells before you change connections.

| Signal | Raspberry Pi header to the hub's port J1 | Hub's port J4 to the driver's Grove I2C connector |
| --- | --- | --- |
| SDA, data | Pin 3, GPIO2 → SDA | SDA → SDA |
| SCL, clock | Pin 5, GPIO3 → SCL | SCL → SCL |
| 5 V supply | Pin 2 → breadboard's positive rail → VCC | VCC → VCC |
| Ground | Pin 9 → breadboard's ground rail → GND | GND → GND |

The Raspberry Pi's power bank and the motor battery share their ground through the hub and the driver.
Both boards use this ground as the reference for SDA and SCL.
The Raspberry Pi's 5 V rail also supplies both wheel encoders.

On the motor driver, the jumper J4 connects the output of the board's regulator to the VCC pin of the Grove connector.
Seeed asks to remove this jumper when the battery powers the screw terminal and the Raspberry Pi supplies the connector, as in DiffBot's wiring ([Seeed's hardware overview](https://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/#hardware-overview)).
With the jumper removed, the regulator supplies the driver's logic, the Raspberry Pi supplies the connector's VCC pin, and the grounds stay connected.

DiffBot connects SDA and SCL through the hub without a level shifter.
I2C devices pull these lines low, and pull-up resistors bring them high when the devices release them.
GPIO2 and GPIO3 each have a fixed 1.8 kΩ pull-up to 3.3 V on the Raspberry Pi.
Its GPIO inputs are designed for 3.3 V signals and can be damaged by 5 V ([Raspberry Pi's documentation of the 40-pin GPIO header](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)).
Seeed lists 5 V for the driver's I2C interface ([specifications](https://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/#specifications)).
The idle voltages on DiffBot's connected signal lines haven't been measured.

With the motor cells removed and the Raspberry Pi powered, SDA and SCL can feed the unpowered microcontroller of the driver, an ATmega8, through the protection diodes of its inputs ([ATmega8 datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2486-8-bit-AVR-microcontroller-ATmega8_L_datasheet.pdf), I/O Ports).
So shut down and unplug the Raspberry Pi before you take out the motor cells, and put the cells back before you power the Raspberry Pi.

Set the driver's DIP switches to the address `0x0f` to match the Python library ([`grove_i2c_motor_driver.py`](https://github.com/ros-mobile-robots/diffbot/blob/f98446cfa658ab520c61faa58146299a4697c736/ros/src/grove_motor_driver/src/grove_i2c_motor_driver.py#L35-L38)).
[Seeed's address switch diagram](https://files.seeedstudio.com/wiki/Grove-I2C_Motor_Driver_V1.3/img/I2CMotorDriver-9.jpg) shows how the switches encode the address.

To check that the driver responds to motor commands, follow [Test Motor Driver](#test-motor-driver).
The GrovePi motor example advises against scanning this controller with `i2cdetect`, because the driver can stop responding and hold the clock line low ([`motor_example.py`](https://github.com/ros-mobile-robots/diffbot/blob/f98446cfa658ab520c61faa58146299a4697c736/ros/src/grove_motor_driver/src/motor_example.py#L9-L13)).
If this happens after a scan, try the driver's `RESET` button.
The following output records a scan on DiffBot's Raspberry Pi. A device answered at the configured address, `0x0f`, on bus 1.

```console
$ i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- 0f 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --
```

### Test Motor Driver

Support DiffBot with both drive wheels clear of the ground.
On its Raspberry Pi, run `python3 motor_example.py` from the `grove_motor_driver/src` directory.
Watch the wheels turn forward, reverse and stop.

```console
fjp@ubuntu:~/git/diffbot/ros/src/grove_motor_driver/src$ python3 motor_example.py 
Forward
Back
Stop
Speed: 0
Speed: 1
Speed: 2
Speed: 3
Speed: 4
Speed: 5
Speed: 6
Speed: 7
Speed: 8
Speed: 9
Speed: 10
Speed: 11
Speed: 12
...
Speed: 25
Speed: 26
Speed: 27
Speed: 28
Speed: 29
Speed: 30
Speed: 31
Speed: 32
Speed: 33
...
Speed: 55
Speed: 56
Speed: 57
Speed: 58
Speed: 59
Speed: 60
Speed: 61
Speed: 62
Speed: 63
Speed: 64
Speed: 65
Speed: 66
...
Speed: 75
Speed: 76
Speed: 77
Speed: 78
Speed: 79
Speed: 80
Speed: 81
Speed: 82
Speed: 83
Speed: 84
Speed: 85
Speed: 86
Speed: 87
Speed: 88
...
Speed: 97
Speed: 98
Speed: 99
Stop
```

#### Troubleshooting

If you get errors like the following, make sure the I2C cables from the motor driver to the 
Raspberry Pi are connected (see [Hardware Interfaces](processing_units/hardware-interfaces.md#single-board-computer-i2c-connection) for more infos) 
and use the `RESET` button on the motor driver.

```console
fjp@ubuntu:~/git/2wd-robot/ros/src/control$ sudo python grove_i2c_motor_driver.py 
Traceback (most recent call last):
  File "grove_i2c_motor_driver.py", line 68, in <module>
    m.MotorSpeedSetAB(100,100)
  File "grove_i2c_motor_driver.py", line 57, in MotorSpeedSetAB
    bus.write_i2c_block_data(self.I2CMotorDriverAdd, self.MotorSpeedSet, [MotorSpeedA,MotorSpeedB])
IOError: [Errno 110] Connection timed out
fjp@ubuntu:~/git/2wd-robot/ros/src/control$ sudo python grove_i2c_motor_driver.py 
Traceback (most recent call last):
  File "grove_i2c_motor_driver.py", line 68, in <module>
    m.MotorSpeedSetAB(100,100)
  File "grove_i2c_motor_driver.py", line 57, in MotorSpeedSetAB
    bus.write_i2c_block_data(self.I2CMotorDriverAdd, self.MotorSpeedSet, [MotorSpeedA,MotorSpeedB])
IOError: [Errno 121] Remote I/O error
```

If the driver remains unresponsive, press and release its `RESET` button, then retry the motor example.
The following terminal output records a scan of bus 1 on DiffBot.

```console
fjp@ubuntu:~/git/2wd-robot/ros/src/control/src$ sudo i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- 0f 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --
```

The first capture shows a response at `0x0f`.
In the capture below, the scan reports a response at every probed address.
The output alone doesn't identify the cause.

```console
i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 
10: 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 
20: 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 
30: 30 31 32 33 34 35 36 37 38 39 3a 3b 3c 3d 3e 3f 
40: 40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f 
50: 50 51 52 53 54 55 56 57 58 59 5a 5b 5c 5d 5e 5f 
60: 60 61 62 63 64 65 66 67 68 69 6a 6b 6c 6d 6e 6f 
70: 70 71 72 73 74 75 76 77
```

To check the wiring, shut down and unplug the Raspberry Pi, then take out the motor cells.
Check SDA at the physical pin 3, SCL at the physical pin 5 and the common ground against the [Connection table](#connection).
Put the motor cells back before you power the Raspberry Pi, then retry the motor example.


### ROS Node for Motor Driver

To use the available library of the Grove I2C motor driver in ROS we need to create a wrapper node, called [`motor_driver`](https://github.com/fjp/2wd-robot/blob/master/ros/src/grove_motor_driver/src/motor_driver.py).
The node subscribes to the topic `/2wd_robot/cmd_vel` which is of type [Twist message](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) 
from the [geometry_msgs](https://docs.ros.org/api/geometry_msgs/html/index-msg.html) header. 
To send commands to the motor the . These topics can be published with nodes from the navigation stack or with [`rostopic pub`](http://wiki.ros.org/rostopic#rostopic_pub) for test purposes.

https://en.wikipedia.org/wiki/Differential_wheeled_robot

http://wiki.ros.org/differential_drive


After the node has been implemented, we need to build the workspace with [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html):

```console
fjp@ubuntu:~/git/2wd-robot/ros$ catkin build
----------------------------------------------------------------
Profile:                     default
Extending:          [cached] /opt/ros/melodic
Workspace:                   /home/fjp/git/2wd-robot/ros
----------------------------------------------------------------
Build Space:        [exists] /home/fjp/git/2wd-robot/ros/build
Devel Space:        [exists] /home/fjp/git/2wd-robot/ros/devel
Install Space:      [unused] /home/fjp/git/2wd-robot/ros/install
Log Space:          [exists] /home/fjp/git/2wd-robot/ros/logs
Source Space:       [exists] /home/fjp/git/2wd-robot/ros/src
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
[build] Found '2' packages in 0.0 seconds.                                                                          
[build] Updating package table.                                                                                     
Starting  >>> grove_motor_driver                                                                                    
Starting  >>> grove_ultrasonic_ranger                                                                                                                                                                              
Finished  <<< grove_motor_driver                     [ 1.0 seconds ]                                                
Finished  <<< grove_ultrasonic_ranger                [ 1.0 seconds ]                                                                                             
[build] Summary: All 2 packages succeeded!                                                                          
[build]   Ignored:   None.                                                                                          
[build]   Warnings:  None.                                                                                          
[build]   Abandoned: None.                                                                                          
[build]   Failed:    None.                                                                                          
[build] Runtime: 2.0 seconds total. 
```


As the final note of the build output suggests, we have to `source` the `setup.bash` files in the `devel` space.

```console
fjp@ubuntu:~/git/2wd-robot/ros$ source devel/setup.bash
```

To make the `ranger` node executable we have to modify the `ranger.py` file:

```console
fjp@ubuntu:~/git/2wd-robot/ros/src/grove_ultrasonic_ranger/src$ sudo chmod a+x ranger.py
```

