## ROS Driver for Adafruit IMU Bosch BNO055

The [Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055](https://www.adafruit.com/product/2472) is based on
the [BNO055 from Bosch](https://www.bosch-sensortec.com/products/smart-sensors/bno055/). This is system in package solution which features a MEMS accelerometer,
magnetometer and gyroscope and putting them on a single die with a high speed ARM Cortex-M0 based processor to digest all the sensor data,
abstract the sensor fusion and real time requirements away, and spit out data you can use in quaternions, Euler angles or vectors.

<figure markdown>
  ![Adafruit Bosch BNO055 IMU]({{ asset_dir }}/components/bno055.jpg)
  <figcaption>Adafruit Bosch BNO055 IMU</figcaption>
</figure>

### High level Driver (I2C)

When connecting the IMU directly via i2c to the single board computer (SBC), which is refered to as the high level approach, we can make use of
[`ros-imu-bno055`](https://github.com/dheera/ros-imu-bno055) ROS package.


### Low Level Driver

In case we connect the Bosch IMU to the low level microcontroller board (e.g. Teensy or Arduino) there is
[Adafruit BNO055](https://github.com/adafruit/Adafruit_BNO055) library which can be used.
