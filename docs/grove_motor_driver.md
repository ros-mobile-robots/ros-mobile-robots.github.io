### Grove - I2C Motor Driver V1.3

The package for the [Grove I2C Motor Driver V1.3](http://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/) from Seeed Studio is created with [`catkin create pkg PKG_NAME [--catkin-deps [DEP [DEP ...]]]`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html#catkin-create-pkg):

```bash
fjp@ubuntu:~/git/2wd-robot/ros/src$ catkin create pkg grove_motor_driver --catkin-deps rospy roscpp geometry_msgs
Creating package "grove_motor_driver" in "/home/fjp/git/2wd-robot/ros/src"...
Created file grove_motor_driver/CMakeLists.txt
Created file grove_motor_driver/package.xml
Created folder grove_motor_driver/include/grove_motor_driver
Created folder grove_motor_driver/src
Successfully created package files in /home/fjp/git/2wd-robot/ros/src/grove_motor_driver.
```

The package depends on the two ROS [client libraries](http://wiki.ros.org/Client%20Libraries) [`rospy`](http://wiki.ros.org/rospy) and [`roscpp`](http://wiki.ros.org/roscpp). 
To control the two motors the package will use the [`geometry_msgs/Twist`](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) message.
The interface to the motor driver is done with the [Python library from DexterInd](https://github.com/DexterInd/GrovePi/blob/master/Software/Python/grove_i2c_motor_driver) 
which is a rewerite of the [Seeed Studio Arduino library](https://github.com/Seeed-Studio/Grove_I2C_Motor_Driver_v1_3/blob/master).


This library requires the following two python libraries

- [RPi.GPIO](https://pypi.org/project/RPi.GPIO/)
- [smbus](https://pypi.org/project/smbus/) SMBus (System Management Bus) is a subset from the I2C protocol

These libraries should be installed with `pip`, Python's package manager:

```bash
pip install RPi.GPIO
pip install smbus
``` 

### Connection

Afterwards the I2C port of the motor should be connected to the I2C port 1 of the Raspberry Pi 4 B. 
Don't forget to remove the jumper on the motor driver board which would provide power to the Pi.
However, it is also not required to connect VCC and GND of the I2C connector. 
Only the SDA (data) and SCL (clock) wires are required.

Make sure to set the address with the dip switches on the motor driver to `0x0f` because this is the default address used
in the library files.

### Test Motor Driver

Test the motor driver by running one of the python files:

```bash
sudo python ...
``` 

If you get errors like the following use the `RESET` button on the motor driver.

```bash
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

```bash
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


### ROS Wrapper

To use the available library of the Grove I2C motor driver in ROS we need to create a wrapper node, called `motor_driver`.
The topic `/2wd_robot/cmd_vel` uses the Twist message to send commands to the motor. These commands can be published with
nodes from the navigation stack.

https://en.wikipedia.org/wiki/Differential_wheeled_robot