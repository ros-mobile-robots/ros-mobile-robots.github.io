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

Connect the I2C port of the motor driver to the I2C port 1 of the Raspberry Pi 4 B.
Two things matter for this connection: the voltage of the two signals and a common ground.

- **Signal voltage:** The motor driver's I2C bus works with 5 V ([Seeed's specifications](https://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/#specifications)). The Raspberry Pi's GPIO pins work with 3.3 V and aren't 5 V tolerant ([Raspberry Pi's documentation of the 40-pin GPIO header](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)). So the data line SDA and the clock line SCL go through a bidirectional I2C level shifter, for example [Adafruit's BSS138 level converter](https://www.adafruit.com/product/757). The level shifter connects the 3.3 V signals on its low side with the 5 V signals on its high side.
- **Common ground:** Each board measures the voltages on SDA and SCL against its own ground. On DiffBot, the two boards have separate supplies: a power bank for the Raspberry Pi and eight NiMH AA cells for the motor driver ([Power Supply](power-supply.md)). So the grounds of the Raspberry Pi, the level shifter and the motor driver have to be connected.

!!! warning "Not tested on a DiffBot yet"
    The wiring below follows the specifications of the two boards. It isn't tested on a DiffBot yet.

Switch off both supplies before you connect the wires: shut down the Raspberry Pi and unplug its power bank, and take the batteries out of the motor driver's holder.
With Adafruit's converter as the example, each row is one wire. The pins of the Raspberry Pi are those of its 40-pin header ([I2C pinout](https://pinout.xyz/pinout/i2c)), and the level shifter's pins are named as printed on its board:

| From | To |
| --- | --- |
| Raspberry Pi pin 1, 3.3 V | Level shifter `LV` |
| Motor driver's I2C connector, VCC | Level shifter `HV` |
| Raspberry Pi pin 6, GND | Level shifter `GND`, on the `LV` side |
| Level shifter `GND`, on the `HV` side | Motor driver's I2C connector, GND |
| Raspberry Pi pin 3, SDA | Level shifter `A1` |
| Level shifter `B1` | Motor driver's I2C connector, SDA |
| Raspberry Pi pin 5, SCL | Level shifter `A2` |
| Level shifter `B2` | Motor driver's I2C connector, SCL |

The level shifter connects `A1` with `B1` and `A2` with `B2` itself. Don't connect these pins with a wire.

Each side of the level shifter gets its reference voltage from the board on that side: 3.3 V from the Raspberry Pi, and the logic supply of the motor driver, which Seeed gives as 5 V.
The motor driver makes this supply from its batteries, with its own 5 V regulator.
At a board input of 9.6 V, the regulator has enough voltage to supply the 5 V logic rail. The [Power Supply](power-supply.md#diffbot-eight-nimh-cells-and-the-grove-i2c-motor-driver) page shows the calculation.
Leave the jumper J4 on the motor driver board in place: it connects the regulator to the VCC pin of the I2C connector, and Seeed asks for the jumper when 5 V should be supplied to the I2C bus ([Seeed's hardware overview](https://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/#hardware-overview)).
Connect this VCC pin only to the level shifter's `HV`, never to the Raspberry Pi, which has its own supply.
The GND pin of the I2C connector is connected to the board's ground, with or without the jumper.

Keep the motor driver powered whenever the Raspberry Pi has power.
Without its batteries, the motor driver would get current from the Raspberry Pi's 3.3 V side through the level shifter and the SDA and SCL wires.
These wires end at the board's microcontroller, an ATmega8 ([Seeed's schematic](https://files.seeedstudio.com/wiki/Grove-I2C_Motor_Driver_V1.3/res/Grove%20-%20I2C%20Motor%20Driver%20%20v1.3b.pdf)), and each of its pins has a protection diode to its supply line ([ATmega8 datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2486-8-bit-AVR-microcontroller-ATmega8_L_datasheet.pdf), I/O Ports).
The level shifter keeps the two sides apart only when the Raspberry Pi's side is off ([Philips application note AN97055](https://cdn-shop.adafruit.com/datasheets/an97055.pdf), section 2.3.4).
So recharge the cells in this order:

1. Stop the motors and shut down the Raspberry Pi.
2. Unplug the Raspberry Pi's power bank. After a shutdown, the Raspberry Pi keeps power on its header while the power bank is plugged in ([Raspberry Pi's bootloader documentation, `POWER_OFF_ON_HALT`](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#POWER_OFF_ON_HALT)).
3. Take out the eight cells, recharge them as [Charging the NiMH cells](power-supply.md#charging-the-nimh-cells) on the Power Supply page describes, and put them back.
4. Plug the power bank in again, which starts the Raspberry Pi.

Make sure to set the address with the dip switches on the motor driver to `0x0f` because this is the default address used
in the library files.

To test the physical I2C connection use `i2cdetect` described in [Hardware Interfaces](processing_units/hardware-interfaces.md#single-board-computer-i2c-connection):
The output should list `0f` in the address table:

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

Test the motor driver by running one of the python files:

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

Try pressing the `RESET` button and release it right before executing one of the scripts.
You should also be able to detect the motor driver with `i2cdetect -y 1`:

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

As you can see the address of the motor driver is detected at `0x0f`.

In case of the following output, where every address of the I2C bus seems to be taken,
it is most likely that the SDA (data) and SCL (clock) signal cables are switched:

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

To solve this make sure the SDA and SCL cables are connected to the right pins on the Raspberry Pi.
See [Hardware Interfaces](processing_units/hardware-interfaces.md#single-board-computer-i2c-connection) for more infos.

Another solution is to restart the Raspberry Pi while making sure that the motor driver is powerd on by connecting it to the battery pack.


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

