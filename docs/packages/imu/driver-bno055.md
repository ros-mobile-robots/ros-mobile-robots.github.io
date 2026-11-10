# IMU: Bosch BNO055

An inertial measurement unit (IMU) measures how the robot turns and accelerates: its gyroscope measures the **angular velocity** about three axes, its accelerometer the **specific force**, the acceleration minus gravity, which at rest reads about +9.8 m/s² upwards. DiffBot and Remo use the [Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout](https://www.adafruit.com/product/2472) with Bosch's [BNO055](https://www.bosch-sensortec.com/en/products/smart-sensor-systems/bno055): an accelerometer, a gyroscope and a magnetometer in one package, with a processor that fuses them. Both robots use the same board, which is moved from one robot to the other.

??? info "Why the accelerometer reads +9.8 m/s² upward at rest"
    The accelerometer contains a tiny suspended mass. When the robot rests on the floor, the floor supports the robot, and the sensor's suspension supports that mass against gravity. It's like the upward force you feel from the floor on your feet.

    The reading is the **specific force**, also called **proper acceleration**: the acceleration relative to free fall. With z pointing upward, gravity is −9.81 m/s². So the resting sensor reads 0 − (−9.81) = +9.81 m/s², although the robot's velocity doesn't change. In ideal free fall, the sensor and its mass fall together, and the reading is zero.

    [Analog Devices](https://www.analog.com/en/resources/technical-articles/accelerometer-and-gyroscopes-sensors-operation-sensing-and-applications.html) explains the mass on its springs and how its movement is measured. [REP 145](https://github.com/ros-infrastructure/rep/blob/master/rep-0145.rst#data-sources), the ROS convention for IMU data, specifies +g on an upward axis at rest.

<figure markdown>
  ![Adafruit Bosch BNO055 IMU]({{ asset_dir }}/components/bno055.jpg)
  <figcaption>Adafruit Bosch BNO055 IMU</figcaption>
</figure>

The robot uses one value from it: the **yaw rate**, the angular velocity about the upward axis. The [EKF](robot_localization.md) on the robot's computer fuses it with the wheel odometry, which, on its own, estimates turns less well. The BNO055 sits on the Teensy's I2C bus, and the Teensy's firmware publishes its measurements over rosserial.

!!! info "Checked on Remo"
    The firmware passed the [bench checks](#bench-checks) on Remo with the wheels off the ground, using both a Teensy 3.2 and Remo's own Teensy 4.0 ([Teensy 4.0 results](https://github.com/ros-mobile-robots/diffbot/issues/169#issuecomment-6600460527)). A tilt test established the board's orientation. Remo's model includes that orientation and the separately checked position on the breadboard. On DiffBot, the IMU's mounting still needs to be measured, and the firmware checked on DiffBot itself. The EKF still needs a test with the real IMU.

## Wiring to the Teensy

The BNO055 shares the Teensy's first I2C bus, `Wire`, with the motor driver ([Adafruit Motor FeatherWing](../../components.md)). On the Teensy 4.0 and 3.2, `Wire` uses pin 18 for SDA and pin 19 for SCL ([PJRC: Wire library](https://www.pjrc.com/teensy/td_libs_Wire.html)):

| BNO055 breakout | Teensy | Note |
|:--|:--|:--|
| VIN | 3.3V | The breakout takes 3.3 to 5 V. 3.3 V is the Teensy's logic level |
| GND | GND | |
| SDA | 18 (SDA) | The breakout has a 10 kΩ pull-up resistor on SDA and SCL |
| SCL | 19 (SCL) | |
| ADR | not connected | I2C address 0x28. Connected to 3.3 V: 0x29, then set `IMU_BNO055_ADDRESS` |

The other pins (3Vo, RST, INT, PS0, PS1) stay unconnected. 3Vo is the breakout's 3.3 V output. Adafruit's [pinouts page](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/pinouts) describes all of them. The motor driver's address is 0x60, so the two don't collide.

## What the firmware publishes

The firmware in [`diffbot_firmware`](https://github.com/ros-mobile-robots/diffbot/tree/noetic-devel/diffbot_firmware) reads the BNO055 in its IMU fusion mode (accelerometer and gyroscope, without the magnetometer, called IMUPLUS in Bosch's and Adafruit's libraries) and publishes `/diffbot/imu/data_raw`, a [`sensor_msgs/Imu`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html) message, at 50 Hz:

| Field | Content |
|:--|:--|
| `header.frame_id` | `imu_link`: the sensor's own axes. The robot's URDF places this frame where the board sits |
| `header.stamp` | ROS time before the read |
| `angular_velocity` | rad/s, from the gyroscope |
| `linear_acceleration` | m/s², the specific force, including gravity |
| `orientation` | not provided: `orientation_covariance[0]` is −1, which [REP 145](https://www.ros.org/reps/rep-0145.html) defines as "no data" for a field |
| covariances | diagonal starting values: 4 · 10⁻⁴ (rad/s)² and 10⁻² (m/s²)². At rest, the [bench checks](#bench-checks) measured much less, but driving adds vibration and drift |

The chip's own orientation estimate isn't read: the EKF uses only the angular velocity. The topic's name is REP 145's for accelerometer and gyroscope data without an orientation.

The settings are in [`diffbot_base_config.h`]({{ diffbot_repo_url }}/diffbot_firmware/lib/config/diffbot_base_config.h): `IMU_BNO055_ADDRESS` and `IMU_PUBLISHER_CONFIG`. The second is one value of the type `ImuPublisherConfig` from [`imu_config.h`]({{ diffbot_repo_url }}/diffbot_firmware/lib/imu/imu_config.h), with these fields:

| Field | Robot's value | What it's for |
|:--|:--|:--|
| `enabled` | `true` | `false` for a robot without an IMU: no topic, no I2C traffic |
| `period_ms` | 20 | The sampling period, 50 Hz |
| `retry_ms` | 10000 | Look for a missing or failed IMU again after 10 s |
| `read_budget_us` | 10000 | Drop a sample whose read took longer than 10 ms |
| `frame_id` | `"imu_link"` | `header.frame_id` |
| `angular_velocity_variance` | 4.0e-4 | (rad/s)², on the covariance's diagonal |
| `linear_acceleration_variance` | 1.0e-2 | (m/s²)², on the covariance's diagonal |

The compiler checks the field types. A build-time check (`static_assert`) rejects invalid values, such as a read budget as long as the sampling period. The unit tests pass settings of their own. The comment in the configuration header explains the covariances' starting values with the datasheet's noise and offsets.

## How the firmware reads it

[`lib/imu`]({{ diffbot_repo_url }}/diffbot_firmware/lib/imu/README.md) has a small driver of its own for the BNO055, instead of [Adafruit's library](https://github.com/adafruit/Adafruit_BNO055), so that every I2C transfer is checked and the start-up's waits don't hold up the firmware's loop:

- **The start-up doesn't block the motor control.** A reset of the chip takes about 650 ms, and each switch of its operating mode up to 19 ms (Bosch's [datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)). The driver goes through the steps as a state machine: configuration mode, reset, settings written and read back, IMU mode, until the chip reports that its fusion runs. Instead of sleeping through those times, each pass through the firmware's loop checks whether the next step is due and does it. Most steps take one or two transfers. The step that writes the settings and reads them back takes 14, about 4 ms, once per start-up. The transfers themselves are synchronous: the loop waits while one runs, about 0.3 ms for one register and about 3 ms for a whole sample (26 bytes) at I2C's standard clock of 100 kHz. The waits for the chip to finish resetting and start its fusion have deadlines. If either wait reaches its deadline, the driver fails the start-up on the next loop pass. The firmware tries again after `retry_ms`. On Remo, the control loop kept its 20 Hz while the IMU was unplugged ([bench checks](#bench-checks)).
- **A sample is published only if it's good:** the read succeeded and took at most `read_budget_us` (10 ms: a later result is dropped, but the budget can't cut a transfer short), the chip still is in IMU mode with the right units and its fusion runs. A stalled or unplugged IMU stops the topic instead of repeating old values with new stamps. After ten failed reads in a row, the firmware starts the chip again. A missing IMU is looked for again every `retry_ms` (10 s). The robot drives without an IMU, too.
- **Stamps are ROS time,** which rosserial takes from the robot's computer: the Teensy's clock is synchronized with it every few seconds. Until the first synchronization after a connection, the firmware publishes nothing, because a stamp would carry the Teensy's uptime instead of ROS time.
- **The 50 Hz rate counts on the Teensy's own clock** (`millis()`), so it doesn't change when ROS time jumps, for example when the Teensy connects to a computer whose clock is earlier.

**Unit tests on the PC:** `.docker/firmware/run.sh pio test -e native`, from the root folder of the diffbot repository, runs the tests in [`diffbot_firmware/test`](https://github.com/ros-mobile-robots/diffbot/tree/noetic-devel/diffbot_firmware/test) in the firmware container ([Running the checks locally](../../development/ci.md#running-the-checks-locally)). They run the driver, the bus code, the time-sync check and the publisher with a fake sensor, I2C bus, serial port and clock, and with rosserial's own message parser. CI runs them in the firmware workflow ([Testing and CI](../../development/ci.md)). They check the logic, not the hardware.

## Bench checks

With the bringup running on the robot and its wheels off the ground:

1. **Rate and stamps:** `rostopic hz /diffbot/imu/data_raw` shows about 50 Hz, and the stamps in `rostopic echo /diffbot/imu/data_raw` are the current ROS time.
2. **Axes and signs:** with the robot level and still, the accelerometer's upward axis reads about +9.8 m/s². Turning the robot counterclockwise by hand, seen from above, gives a positive angular velocity about that axis ([REP 103](https://www.ros.org/reps/rep-0103.html)). These checks identify the upward axis and its sign. To check the horizontal axes, lift the robot's front slightly and hold the robot still while reading the acceleration. Return the robot to level, then lift the left side and hold still again. For sensor axes aligned with the robot's directions, a horizontal axis pointing toward the raised side reads positive. An axis pointing the opposite way reads negative. On Remo, raising the front gives positive y and raising the left side gives negative x. These checks establish the orientation of `imu_link` in the URDF. The mounting position must be checked separately.
3. **Noise and bias:** a recording of 180 s at rest gives the gyroscope's remaining bias and the noise of each axis, as [Bias and noise at rest](#bias-and-noise-at-rest) below shows.
4. **Faults:** unplugging the IMU stops the topic and logs a warning. While it stays missing, each retry logs another warning. The wheels keep following commands. Plugged back in, it's found at the next try, at most `retry_ms` (10 s) later, and publishes again after its start-up, which takes about another second. The control loop keeps its rate during all this. The firmware's command timeout keeps working: after 1 s without a wheel command, it stops driving the motors and resets the wheels' PIDs ([`COMMAND_TIMEOUT_MS`](https://github.com/ros-mobile-robots/diffbot/blob/1a3b5476e1188565872b0bcc1a3d239d8b2ed529/diffbot_firmware/lib/config/diffbot_base_config.h#L35-L39), [`eStop()`](https://github.com/ros-mobile-robots/diffbot/blob/1a3b5476e1188565872b0bcc1a3d239d8b2ed529/diffbot_firmware/lib/base_controller/base_controller.h#L564-L575)).
5. **Timing:** the control loop keeps its rate with the IMU running, for example `rostopic hz /diffbot/measured_joint_states`.

Remo was checked on 2026-10-31 with firmware `1a3b547` built for the Teensy 3.2, and the ROS commands ran in the [robot container](../../processing_units/robot-container/usage.md). The BNO055 used SDA 18 and SCL 19, with VIN supplied by the Raspberry Pi's 3.3 V ([evidence](https://github.com/ros-mobile-robots/diffbot/issues/160#issuecomment-6550353204), checklist [diffbot#169](https://github.com/ros-mobile-robots/diffbot/issues/169)).

| Check | Result |
|:--|:--|
| Rate and stamps | 50.0 Hz. 180 s at rest: 8,986 messages, gaps between 14 and 27 ms. Stamps in current ROS time, also after rosserial reconnected |
| Axes and signs | At rest in this recording, z reads about +9.7 m/s². Turning left, `angular_velocity.z` is positive. How the board is turned about z came from a later tilt test, below |
| Noise and bias at rest | Gyroscope: bias below 0.0001 rad/s, standard deviation 0.0005 to 0.0008 rad/s, below the output step of 1/900 rad/s. The same with the Teensy 4.0 ([Bias and noise at rest](#bias-and-noise-at-rest)) |
| Faults | Unplugged while the wheels turned at 9 rad/s: a warning after ten failed reads (0.2 s), and the wheels kept their speed. Plugged back in: found again within 10 s. With SDA and SCL swapped, it stayed "not found", without blocking the firmware |
| Timing | `measured_joint_states` at 20.0 Hz with the IMU running |

The [evidence](https://github.com/ros-mobile-robots/diffbot/issues/160#issuecomment-6550353204) has the numbers and log excerpts, [diffbot#169](https://github.com/ros-mobile-robots/diffbot/issues/169) the full checklist.

A [tilt test](https://github.com/ros-mobile-robots/diffbot/issues/169#issuecomment-6684760031) on 2026-11-05 checked the BNO055's axes on Remo using the Teensy 4.0. With the front held raised, the average y acceleration was +3.21 m/s². With the left side held raised, the average x acceleration was −7.24 m/s². The z acceleration remained positive during both tilts and was close to +9.8 m/s² with the robot level. These readings show that the board's y axis points forward, its x axis points right, and its z axis points up when Remo is level. The corresponding URDF orientation is `imu_rpy="0 0 -1.5707963"`, a −90° rotation about `chassis_link`'s z axis. The chip's pin-1 dot sits at the board's front right corner, which with the datasheet's default axes, placement P1 ([BNO055 datasheet, page 27](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf#page=27)) gives the same. Remo's model has this orientation and the checked position on Remo's Raspberry Pi deck ([`remo.urdf.xacro`, lines 12–27](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/remo.urdf.xacro#L12-L27) in remo_description 2.5.0). The IMU's header pins sit in the breadboard, and `imu_link` lies about 7.6 cm behind the wheel axle and 2.34 cm left of the robot's center line.

### Bias and noise at rest

An IMU at rest should read an angular velocity of zero and an acceleration of gravity's size. A real one doesn't. Its readings have an offset, the **bias**, and they scatter around their average, the **noise**. A recording of the robot standing still estimates the gyroscope's remaining bias and the noise on each axis. The acceleration's average still contains gravity. The noise is a baseline for the variances in the IMU's messages, which tell the [EKF](robot_localization.md) how much to trust each reading.

**1. Record 180 s at rest.** Put the robot on a firm surface, with the bringup running, and don't touch the robot or the table while it records. On the Raspberry Pi, in `~/diffbot/.docker/robot`:

```console
docker compose exec -w /home/ros/.ros bringup /entrypoint.sh rosbag record -O imu_still.bag --duration=180 /diffbot/imu/data_raw
```

`rosbag record` saves the messages of the topic `/diffbot/imu/data_raw` in the file `imu_still.bag` (`-O`) and stops by itself after 180 s. `-w /home/ros/.ros` runs the command in this folder of the container, which is `~/robot-data` on the Raspberry Pi by default. Without it, the file would stay inside the container and be gone after `docker compose down`. The rest of the line is explained under "Running a command in a container" on [Start, Check and Stop](../../processing_units/robot-container/usage.md).

With a native ROS installation ([ROS Setup](../../processing_units/ros-setup.md)) instead of the robot container, run the same `rosbag record` command without the part from `docker compose exec` to `/entrypoint.sh`. The file is then saved in the folder where the command runs. The same holds for the tool in step 2.

**2. Run the tool** [`imu_noise_at_rest.py`]({{ diffbot_repo_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py) on the recording:

```console
docker compose exec -w /home/ros/.ros bringup /entrypoint.sh rosrun diffbot_bringup imu_noise_at_rest.py imu_still.bag
```

The tool only reads the file, so it doesn't need the robot or a ROS master. To run it on the PC instead, copy the file there (`scp <user>@<robot-ip>:robot-data/imu_still.bag .`) and run `rosrun diffbot_bringup imu_noise_at_rest.py imu_still.bag` in the dev container. `--topic` picks another `sensor_msgs/Imu` topic. With Remo's Teensy 4.0, it printed:

```
/diffbot/imu/data_raw: 8990 messages in 179.8 s, 50.00 Hz
Gaps between the stamps: shortest 13.3 ms, longest 25.8 ms, 0 longer than 30 ms

angular_velocity  mean (bias)     variance  std dev  distinct
                      [rad/s]  [(rad/s)^2]  [rad/s]    values
x                    -0.00003    3.194e-07  0.00057        10
y                    -0.00001    6.572e-07  0.00081        11
z                    +0.00005    5.317e-07  0.00073         9

linear_acceleration  mean (gravity + bias)     variance  std dev  distinct
                                   [m/s^2]  [(m/s^2)^2]  [m/s^2]    values
x                                 -0.23153    1.773e-04  0.01331        11
y                                 +0.07258    7.015e-04  0.02649        19
z                                 +9.78867    1.265e-03  0.03557        21

Gravity, the length of the mean acceleration: 9.792 m/s^2
```

**3. Read the numbers.**

| Number | What it means |
|:--|:--|
| Messages and Hz | 180 s at 50 Hz give about 9,000 messages. The rate counts the gaps between the stamps per second. |
| Gaps between the stamps | The firmware sends a message every 20 ms. A gap longer than 30 ms means that a message was lost or came late. |
| Mean of `angular_velocity` | The gyroscope's **bias**: what it reads while the robot doesn't turn. In the IMU mode that the firmware selects, the BNO055's fusion estimates the gyroscope's offset in the background and subtracts it from the output ([datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf), sections 3.6.5.1 and 3.6.5.3). So this is the bias that remains in the corrected output, not the bias of the raw sensor, which the firmware never reads. |
| Mean of `linear_acceleration` | Gravity's share on each axis, plus the accelerometer's remaining error. One recording at rest can't separate the two. The shares depend on how the robot stands, so they change from one recording to the next. |
| Gravity | The length of the mean acceleration: gravity as the IMU measures it, whichever way the robot is tilted. It should be close to 9.81 m/s². |
| Variance | The **noise**: how far the readings scatter around their mean, in (rad/s)² or (m/s²)², the unit of the covariances in the message. It's the noise at 50 Hz with the robot at rest, a baseline for the diagonal of the message's covariances. Driving adds vibration from the motors and the floor, so the variance at rest can underestimate the noise while driving. |
| Std dev | The standard deviation, the square root of the variance. It's in the readings' own unit, so it's easier to compare with them. |
| Distinct values | How many different values an axis read. The BNO055 gives its readings in steps of 1/900 rad/s (0.0011 rad/s) and 1/100 m/s² (0.01 m/s²). The gyroscope's noise is smaller than one step, so its readings take only a handful of values around zero. |

**Results on Remo:** two recordings of 180 s, with a Teensy 3.2 on 2026-10-31 ([evidence](https://github.com/ros-mobile-robots/diffbot/issues/160#issuecomment-6550353204)) and with Remo's own Teensy 4.0 on 2026-11-03 ([results](https://github.com/ros-mobile-robots/diffbot/issues/169#issuecomment-6600460527)).

Both boards give the same picture, as expected: the Teensy only reads the sensor's registers. The gyroscope's bias stays below 0.0001 rad/s, less than a tenth of one output step. Its variance, 2 to 7 · 10⁻⁷ (rad/s)², is about a thousand times smaller than the firmware's starting value of 4 · 10⁻⁴ (rad/s)². The starting values stay until a check while driving shows how much the vibration adds. That check has to tell real motion apart from measurement noise: the readings of a robot that turns or speeds up change because it moves.

??? info "The numbers of both recordings"
    | | Teensy 3.2 | Teensy 4.0 |
    |:--|:--|:--|
    | Messages and rate | 8,986 at 50.00 Hz | 8,990 at 50.00 Hz |
    | Gaps between the stamps | 14.2 to 27.4 ms, none over 30 ms | 13.3 to 25.8 ms, none over 30 ms |
    | Gyroscope bias x, y, z (rad/s) | +0.00001, +0.00000, +0.00003 | −0.00003, −0.00001, +0.00005 |
    | Gyroscope std dev x, y, z (rad/s) | 0.00047, 0.00078, 0.00075 | 0.00057, 0.00081, 0.00073 |
    | Gyroscope distinct values x, y, z | 7, 8, 9 | 10, 11, 9 |
    | Acceleration std dev x, y, z (m/s²) | 0.020, 0.024, 0.028 | 0.013, 0.026, 0.036 |
    | Acceleration distinct values x, y, z | 11, 14, 17 | 11, 19, 21 |
    | Gravity | 9.742 m/s² | 9.792 m/s² |

To try the breakout separately with an Arduino, see [Adafruit's Arduino guide](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code) for wiring, library installation and example programs.

??? info "Trying the breakout on the Raspberry Pi"
    To check a breakout on its own, it can also be connected to the Raspberry Pi's I2C pins ([pinout](https://pinout.xyz/pinout/i2c)): VIN to 3.3 V, GND to GND, SDA to SDA, SCL to SCL. Adafruit's guide reads it with Python and its CircuitPython library: first [prepare the Pi for it](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi) (I2C on, the Blinka library), then [slow down the Pi's I2C clock](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching), which the BNO055 needs because it stretches the clock, and then run the [BNO055 example](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/python-circuitpython).

    [ros-imu-bno055](https://github.com/dheera/ros-imu-bno055) is a C++ driver for ROS 1 and ROS 2 that reads the BNO055 over I2C. The project's README says the driver should work on a Raspberry Pi 5, or on an older Pi using software I2C. The author has not tested the slower-clock workaround with this driver. DiffBot and Remo read the IMU on the Teensy. ros-imu-bno055 has not been tested with either robot.
