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

The robot uses one value from it: the **yaw rate** $\omega_z$, the angular velocity about the upward axis. The [EKF](robot_localization.md) on the robot's computer fuses it with the wheel odometry, which, on its own, estimates turns less well. The BNO055 sits on the Teensy's I2C bus, and the Teensy's firmware publishes its measurements over [rosserial](../../processing_units/teensy-mcu.md#other-boards-such-as-an-arduino).

To measure how far your robot's IMU is off at rest and how much its readings scatter, go to [Bias and noise at rest](#bias-and-noise-at-rest). Its figure shows what the bias and the noise are, and one command measures both on a recording of the IMU's messages.

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

- **The start-up doesn't block the motor control:** a reset of the chip takes about 650 ms, and each switch of its operating mode up to 19 ms (Bosch's [datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)). The driver goes through the steps as a state machine: configuration mode, reset, settings written and read back, IMU mode, until the chip reports that its fusion runs. Instead of sleeping through those times, each pass through the firmware's loop checks whether the next step is due and does it. Most steps take one or two transfers. The step that writes the settings and reads them back takes 14, about 4 ms, once per start-up. The transfers themselves are synchronous: the loop waits while one runs, about 0.3 ms for one register and about 3 ms for a whole sample (26 bytes) at I2C's standard clock of 100 kHz. The waits for the chip to finish resetting and start its fusion have deadlines. If either wait reaches its deadline, the driver fails the start-up on the next loop pass. The firmware tries again after [`retry_ms`](#what-the-firmware-publishes). On Remo, the control loop kept its 20 Hz while the IMU was unplugged ([bench checks](#bench-checks)).
- **A sample is published only if it's good:** the read succeeded and took at most `read_budget_us` (10 ms: a later result is dropped, but the budget can't cut a transfer short), the chip still is in IMU mode with the right units and its fusion runs. A stalled or unplugged IMU stops the topic instead of repeating old values with new stamps. After ten failed reads in a row, the firmware starts the chip again. A missing IMU is looked for again every `retry_ms` (10 s). The robot drives without an IMU, too.
- **Stamps are ROS time:** [rosserial](../../processing_units/teensy-mcu.md#other-boards-such-as-an-arduino) takes the time from the robot's computer and synchronizes the Teensy's clock with it every few seconds. Until the first synchronization after a connection, the firmware publishes nothing, because a stamp would carry the Teensy's uptime instead of ROS time.
- **The 50 Hz rate counts on the Teensy's own clock:** the firmware times it with [`millis()`](../diffbot_base/low-level.md#timing-with-the-mcus-clock), so the rate doesn't change when ROS time jumps, for example when the Teensy connects to a computer whose clock is earlier.

**Unit tests on the PC:** `.docker/firmware/run.sh pio test -e native -e native_googletest`, from the root folder of the diffbot repository, runs the tests in [`diffbot_firmware/test`]({{ firmware_tests_src_url }}/diffbot_firmware/test) in the firmware container ([Running the checks locally](../../development/ci.md#running-the-checks-locally)). The bus code's suite is written with [GoogleTest](../../development/firmware-unit-tests.md#which-framework-for-a-new-test) and the other three with Unity, each with an environment of its own in that command. They run the driver, the bus code, the time-sync check and the publisher with a fake sensor, I2C bus, serial port and clock, and with rosserial's own message parser. CI runs them in the firmware workflow ([Testing and CI](../../development/ci.md)). They check the logic, not the hardware.

## Bench checks

With the bringup running on the robot and its wheels off the ground:

1. **Rate and stamps:** `rostopic hz /diffbot/imu/data_raw` shows about 50 Hz, and the stamps in `rostopic echo /diffbot/imu/data_raw` are the current ROS time.
2. **Axes and signs:** with the robot level and still, the accelerometer's upward axis reads about +9.8 m/s². Turning the robot counterclockwise by hand, seen from above, gives a positive angular velocity about that axis ([REP 103](https://www.ros.org/reps/rep-0103.html)). These checks identify the upward axis and its sign. To check the horizontal axes, lift the robot's front slightly and hold the robot still while reading the acceleration. Return the robot to level, then lift the left side and hold still again. For sensor axes aligned with the robot's directions, a horizontal axis pointing toward the raised side reads positive. An axis pointing the opposite way reads negative. On Remo, raising the front gives positive y and raising the left side gives negative x. These checks establish the orientation of [`imu_link`](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/remo.urdf.xacro#L12-L27) in the URDF. The mounting position must be checked separately.
3. **Noise and bias:** a recording of 180 s at rest gives the gyroscope's remaining bias and the noise of each axis, as [Bias and noise at rest](#bias-and-noise-at-rest) shows.
4. **Faults:** unplugging the IMU stops the topic and logs a warning. While it stays missing, each retry logs another warning. The wheels keep following commands. Plugged back in, it's found at the next try, at most [`retry_ms`](#what-the-firmware-publishes) (10 s) later, and publishes again after its start-up, which takes about another second. The control loop keeps its rate during all this. The firmware's command timeout keeps working: after 1 s without a wheel command, it stops driving the motors and resets the wheels' PIDs ([`COMMAND_TIMEOUT_MS`](https://github.com/ros-mobile-robots/diffbot/blob/1a3b5476e1188565872b0bcc1a3d239d8b2ed529/diffbot_firmware/lib/config/diffbot_base_config.h#L35-L39), [`eStop()`](https://github.com/ros-mobile-robots/diffbot/blob/1a3b5476e1188565872b0bcc1a3d239d8b2ed529/diffbot_firmware/lib/base_controller/base_controller.h#L564-L575)).
5. **Timing:** the control loop keeps its rate with the IMU running, for example `rostopic hz /diffbot/measured_joint_states`.

Remo was checked on 2026-10-31 with firmware `1a3b547` built for the Teensy 3.2, and the ROS commands ran in the [robot container](../../processing_units/robot-container/usage.md). The BNO055 used SDA 18 and SCL 19, with VIN supplied by the Raspberry Pi's 3.3 V ([evidence](https://github.com/ros-mobile-robots/diffbot/issues/160#issuecomment-6550353204), checklist [diffbot#169](https://github.com/ros-mobile-robots/diffbot/issues/169)).

| Check | Result |
|:--|:--|
| Rate and stamps | 50.0 Hz. 180 s at rest: 8,986 messages, intervals between the header stamps of 14 to 27 ms. Stamps in current ROS time, also after [rosserial](#how-the-firmware-reads-it) reconnected |
| Axes and signs | At rest in this recording, z reads about +9.7 m/s². Turning left, `angular_velocity.z` is positive. How the board is turned about z came from a later tilt test, below |
| Noise and bias at rest | Gyroscope: bias below 0.0001 rad/s, standard deviation 0.0005 to 0.0008 rad/s, below the output step of 1/900 rad/s. The same with the Teensy 4.0 ([Results on Remo](#results-on-remo)) |
| Faults | Unplugged while the wheels turned at 9 rad/s: a warning after ten failed reads (0.2 s), and the wheels kept their speed. Plugged back in: found again within 10 s. With SDA and SCL swapped, it stayed "not found", without blocking the firmware |
| Timing | [`measured_joint_states`](../../development/simulation-test.md#on-the-real-robot) at 20.0 Hz with the IMU running |

The [evidence](https://github.com/ros-mobile-robots/diffbot/issues/160#issuecomment-6550353204) has the numbers and log excerpts, [diffbot#169](https://github.com/ros-mobile-robots/diffbot/issues/169) the full checklist.

A [tilt test](https://github.com/ros-mobile-robots/diffbot/issues/169#issuecomment-6684760031) on 2026-11-05 checked the BNO055's axes on Remo using the Teensy 4.0. With the front held raised, the average y acceleration was +3.21 m/s². With the left side held raised, the average x acceleration was −7.24 m/s². The z acceleration remained positive during both tilts and was close to +9.8 m/s² with the robot level. These readings show that the board's y axis points forward, its x axis points right, and its z axis points up when Remo is level. The corresponding URDF orientation is [`imu_rpy="0 0 -1.5707963"`](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/remo.urdf.xacro#L27), a −90° rotation about [`chassis_link`](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/remo.urdf.xacro#L212-L215)'s z axis. The chip's pin-1 dot sits at the board's front right corner, which with the datasheet's default axes, placement P1 ([BNO055 datasheet, page 27](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf#page=27)) gives the same. Remo's model has this orientation and the checked position on Remo's Raspberry Pi deck ([the mounting arguments in `remo.urdf.xacro`](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/remo.urdf.xacro#L12-L27) in [remo_description](../remo_description.md) 2.5.0). The IMU's header pins sit in the breadboard, and `imu_link` lies about 7.6 cm behind the wheel axle and 2.34 cm left of the robot's center line.

## Bias and noise at rest

An IMU at rest should read an angular velocity of zero and an acceleration of gravity's size. A real one doesn't: its readings have an offset, the **bias**, and they scatter around their average, the **noise**. The tool [`imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py) in the diffbot repository reads a recording of the IMU's messages, made while the robot stood still, and prints the gyroscope's remaining bias and the noise of each axis.

### Bias and noise in a picture

The figure shows what the two words mean. It plots the yaw rate $\omega_z$ that a gyroscope at rest reads, against time. A robot at rest doesn't turn, so every reading should lie on the zero line.

--8<-- "includes/imu-noise-trace.html"

- **The bias** is how far the mean of the readings lies from zero.
- **The noise** is how far the readings scatter around their mean. The band reaches one standard deviation $s$ above and below the mean.
- **The steps:** the readings lie on the dotted lines only, because the sensor reports whole steps. The BNO055's step is 1/900 rad/s, about 0.0011 rad/s.

The example's bias is about 30 times the bias that was measured on Remo, so that the figure can show it. With Remo's +0.00005 rad/s, which [Results on Remo](#results-on-remo) lists, the line of the mean would lie on the zero line.

### Run the tool on a recording

**You need:** the recording, a [rosbag](https://github.com/ros/ros_comm/tree/25d371664e34ec9d26ee331434de9a38c412c890/tools/rosbag) file with the topic [`/diffbot/imu/data_raw`](#what-the-firmware-publishes) that was recorded while the robot stood still, and a terminal in which the diffbot workspace is built and sourced. [Make a recording](#make-a-recording) shows how to get both.

In the folder that holds the recording, run the tool with the recording's file name:

```console
rosrun diffbot_bringup \
  imu_noise_at_rest.py \
  imu_still_2026-11-18-13-18-16.bag
```

`imu_still_2026-11-18-13-18-16.bag` is an example of a name. Use the name of your own recording. For Remo's 180 s at rest, recorded on 2026-11-03 with the Teensy 4.0, the tool's report has these results for the yaw rate $\omega_z$:

| Result | Remo at rest |
|:--|:--|
| Messages | 8,990 at 50.00 Hz |
| **Bias** of the yaw rate: the column `mean (bias)` | +0.00005 rad/s |
| **Noise** of the yaw rate: the column `std dev` | 0.00073 rad/s |
| The noise as a variance: the column `variance` | 5.317 · 10⁻⁷ (rad/s)² |

The bias is small: 0.00005 rad/s is less than a twentieth of the smallest step that the BNO055 can report. The noise is a baseline for the variances in the IMU's messages, which tell the [EKF](robot_localization.md) how much to trust each reading, as [From the report to the EKF](#from-the-report-to-the-ekf) shows.

The `\` at the end of a line continues the command on the next line, so the three lines above are one command. The tool only reads the file: it needs no robot and no ROS master. Its report starts with the messages and their time stamps. A table for the angular velocity and a table for the acceleration follow, each with one row for each axis. The yaw rate is the row `z` of `angular_velocity`, the value that the EKF uses. For the acceleration, the mean of an axis isn't a bias: it is gravity's share on the axis together with the sensor's error, as [Read the report](#read-the-report) explains.

The rest of this section explains the result:

1. [Make a recording](#make-a-recording): 180 s on the robot, without overwriting the recording before.
2. [Read the report](#read-the-report): what each number means.
3. [The mean and the variance of three readings](#the-mean-and-the-variance-of-three-readings): the calculation by hand.
4. [From the report to the EKF](#from-the-report-to-the-ekf): where one measured number belongs.
5. [Results on Remo](#results-on-remo) and [How the tool is tested](#how-the-tool-is-tested).

### Make a recording

The commands of the tabs "Robot container" below and the copy with `sftp` weren't run on a robot yet. The tool itself ran on recordings on a PC.

Put the robot on a firm surface, with the [bringup running](../../processing_units/robot-container/usage.md#start). Don't touch the robot or its table during the recording. The command records the IMU's topic for 180 s and then stops by itself.

=== "Robot container"

    On the Raspberry Pi, in `~/diffbot/.docker/robot`:

    ```console
    docker compose exec \
      -w /home/ros/.ros bringup \
      /entrypoint.sh rosbag record \
      -o imu_still --duration=180 \
      /diffbot/imu/data_raw
    ```

    `-w /home/ros/.ros` runs the command in this folder of the container. Compose mounts a folder of the Raspberry Pi there: `~/robot-data`, or the folder that the variable [`ROBOT_DATA`](../../processing_units/robot-container/setup.md#all-settings) names if you set it. Without `-w`, the file would be saved inside the container and be gone after `docker compose down`. The box "Running a command in a container" under [All commands](../../processing_units/robot-container/usage.md#all-commands) explains `docker compose exec` and `/entrypoint.sh`.

=== "Native ROS"

    On a robot with a native ROS installation ([ROS Setup](../../processing_units/ros-setup.md)), in the folder where the recording should be saved:

    ```console
    rosbag record \
      -o imu_still --duration=180 \
      /diffbot/imu/data_raw
    ```

The recorder prints the name of the file that it writes. Its second line ends with the name:

```
Recording to 'imu_still_2026-11-18-13-18-16.bag'.
```

The two options of [`rosbag record`](https://github.com/ros/ros_comm/blob/25d371664e34ec9d26ee331434de9a38c412c890/tools/rosbag/src/rosbag/rosbag_main.py#L91-L96) do this:

- **`-o imu_still`:** the recorder puts `imu_still` in front of the date and the time at which the recording starts, to the second. A second recording that starts at least a second later, by a clock that goes forward, gets another name. So a repeated check of 180 s keeps the recording before it.
- **`--duration=180`:** the recorder stops after 180 s.

!!! warning "Not the capital `-O`"
    `-O imu_still.bag` would write exactly this name and replace a file of that name without asking. A second run would destroy the first recording. In our test with ROS Noetic it did so even when it recorded no message at all, because no IMU was publishing.

Then run the tool where the recording is, with the name that the recorder printed:

=== "Robot container"

    On the Raspberry Pi, in `~/diffbot/.docker/robot`:

    ```console
    docker compose exec \
      -w /home/ros/.ros bringup \
      /entrypoint.sh rosrun \
      diffbot_bringup imu_noise_at_rest.py \
      imu_still_2026-11-18-13-18-16.bag
    ```

    The image holds the diffbot repository as it was when the image was built. In an image that is older than the tool, `rosrun` doesn't find `imu_noise_at_rest.py`. Then update the code and build the image again: `git pull`, then `docker compose up -d --build` ([All commands](../../processing_units/robot-container/usage.md#all-commands)).

=== "Native ROS"

    In the folder of the recording:

    ```console
    rosrun diffbot_bringup \
      imu_noise_at_rest.py \
      imu_still_2026-11-18-13-18-16.bag
    ```

=== "Dev container on the PC"

    The [dev container](../../development/dev-container.md) mounts your clone of the diffbot repository on the PC as `/home/ros/catkin_ws/src/diffbot`. A file in the clone is therefore also in the container, and a file anywhere else on the PC isn't.

    **1. Copy the recording into the clone.** In a terminal on the PC, in the folder of your clone:

    ```console
    BAG=imu_still_2026-11-18-13-18-16.bag
    FROM='robot@remo.local:robot-data'
    sftp "$FROM/$BAG"
    ```

    The first two lines hold what differs from one robot and one recording to the next, and the third line copies. Replace the values in the first two lines with yours:

    - **`BAG`:** the name of your recording.
    - **`FROM`:** the user and the name or address of the Raspberry Pi, as you use them for `ssh`, and after the colon the folder of the recording on the Raspberry Pi. `robot@remo.local` is the example of [Set Up the Raspberry Pi](../../processing_units/robot-container/setup.md#2-first-login-and-updates). `robot-data` is the robot container's folder. If you set `ROBOT_DATA`, use the folder that the variable names. If you recorded with a native ROS installation, use the folder in which you ran `rosbag record`.
    - **The quotes** keep a folder's name with a space in it in one piece.
    - **[`sftp`](https://manpages.ubuntu.com/manpages/jammy/man1/sftp.1.html)** is SSH's file transfer. Given the path of a file, it fetches that file. The file is saved in the current folder of your PC.

    **2. Run the tool in the dev container.** A terminal of the dev container starts in `/home/ros/catkin_ws/src/diffbot`, the same folder, with the built workspace sourced:

    ```console
    rosrun diffbot_bringup \
      imu_noise_at_rest.py \
      imu_still_2026-11-18-13-18-16.bag
    ```

    Git lists the copy as an untracked file of your clone. Delete the copy after the check, or leave it out of your commits.

### Read the report

After the checks of the input, the report has 18 lines when each of the two fields, the angular velocity and the acceleration, has at least two readings. The example is the whole report for a small bag: three messages 20 ms apart, with the yaw rates 0, 1/900 and 2/900 rad/s and a robot that stands level. [The mean and the variance of three readings](#the-mean-and-the-variance-of-three-readings) calculates its row `z` by hand.

??? example "The whole report for three messages"
    The tool's test [`test_tool_prints_the_report_of_three_readings_one_output_step_apart`](noise-at-rest-tests.md#the-whole-report-for-three-messages) writes this bag and compares every line of the report:

    ```
    /diffbot/imu/data_raw: 3 messages, frame_id "imu_link"
    Header stamps: 40.0 ms from the first to the last, 50.00 Hz
    Header intervals: shortest 20.0 ms, longest 20.0 ms, 0 longer than 30 ms
    Stamps that don't advance: 0 repeated, 0 earlier than the one before

    angular_velocity  mean (bias)     variance  std dev  distinct
    3 readings            [rad/s]  [(rad/s)^2]  [rad/s]    values
    x                    +0.00000    0.000e+00  0.00000         1
    y                    +0.00000    0.000e+00  0.00000         1
    z                    +0.00111    1.235e-06  0.00111         3

    linear_acceleration  mean (gravity + bias)     variance  std dev  distinct
    3 readings                         [m/s^2]  [(m/s^2)^2]  [m/s^2]    values
    x                                 +0.00000    0.000e+00  0.00000         1
    y                                 +0.00000    0.000e+00  0.00000         1
    z                                 +9.81000    0.000e+00  0.00000         1

    Gravity, the length of the mean acceleration: 9.810 m/s^2
    ```

The first four lines are about the messages and their header stamps. The firmware takes a message's stamp before it reads the IMU ([What the firmware publishes](#what-the-firmware-publishes)), and the tool reads the stamps in the order in which the bag stored the messages.

| Line | What it means |
|:--|:--|
| `3 messages, frame_id "imu_link"` | The number of messages $N$ on the topic, and the frame whose axes x, y and z the readings have. |
| `Header stamps` | The time from the first header stamp $t_1$ to the last $t_N$, and the rate: the number of intervals in that time, $(N - 1) / (t_N - t_1)$. Three messages have two intervals, so 2 / 0.040 s gives 50 Hz. |
| `Header intervals` | The shortest and the longest time between two consecutive header stamps, and how many are longer than 30 ms. The firmware takes a sample every 20 ms, and 30 ms is one and a half of these periods. A longer interval is worth a look. Two possible reasons are a sample that the firmware took late and a sample that is missing. The header stamps don't say when a message arrived, so the report can't show a message that was delivered late. A longest interval just above 30 ms that would read `30.0 ms` is given to the nanosecond, such as `30.040000 ms`. |
| `Stamps that don't advance` | How many stamps repeat the stamp before, and how many are earlier than the stamp before, for example because the clock was set back. Both counts should be 0. If one isn't, the line `Header stamps` says "no rate" instead of a rate in Hz. |

Each table has one row for each axis and these columns:

| Column | What it means |
|:--|:--|
| `3 readings` | Below the field's name: how many messages have a measurement of this field. A [`sensor_msgs/Imu`](#what-the-firmware-publishes) message marks a field that it didn't measure with −1 as the first element of the field's covariance ([`Imu.msg`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/sensor_msgs/msg/Imu.msg#L5-L13)). The tool leaves such a message out of the field's statistics. |
| `mean` of `angular_velocity` | The gyroscope's **bias**: what the gyroscope reads while the robot doesn't turn. In the IMU mode that the firmware selects, the BNO055's fusion estimates the gyroscope's offset in the background and subtracts the estimate from the output ([datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf), sections 3.6.5.1 and 3.6.5.3). So the mean is the bias that remains in the corrected output. It isn't the bias of the raw sensor, which the firmware never reads. |
| `mean` of `linear_acceleration` | Gravity's share on each axis, plus the accelerometer's remaining error. One recording at rest can't separate the two. The shares depend on how the robot stands, so they change from one recording to the next. |
| `variance` | The **noise** as a variance $s^2$: how far the readings scatter around their mean, as a squared distance in (rad/s)² or (m/s²)². [The mean and the variance of three readings](#the-mean-and-the-variance-of-three-readings) has the formula. The variances in the message's covariances have the same unit. |
| `std dev` | The standard deviation $s$, the square root of the variance. Its unit is the readings' own, so it is easier to compare with them. |
| `distinct values` | How many different values the axis read in this recording. The BNO055 reports in steps of 1/900 rad/s (0.0011 rad/s) and 1/100 m/s² (0.01 m/s²). When the noise is smaller than one step, the readings take only a handful of values around their mean. The count says how many steps the recording used. It doesn't say how large a step is. |

A report can be shorter. An IMU can leave a field out, and each field needs two readings for its table:

- **A field with fewer than two readings:** instead of the table, the report has the field's number of readings and the reason, for example `angular_velocity: 0 readings, the statistics need at least two`. If the other field has at least two readings, its table stays, and the tool ends with status 0, as for a full report.
- **Fewer than two readings of the acceleration:** the report has no line about gravity.
- **No field with two readings:** the tool prints one line with the reason and ends with status 1.

The last line is gravity as the IMU measures it: the length of the mean acceleration. A robot that stands tilted measures gravity on all three axes, and the length puts the three shares together again:

$$
\sqrt{\bar{a}_x^2 + \bar{a}_y^2 + \bar{a}_z^2}
$$

$\bar{a}_x$, $\bar{a}_y$ and $\bar{a}_z$ are the means of the three acceleration axes. Remo's means of −0.23153, +0.07258 and +9.78867 m/s² give $\sqrt{0.0536 + 0.0053 + 95.8181}$ m/s² = 9.792 m/s². The result should be close to 9.81 m/s², however the robot stands.

The tool refuses a recording that it can't interpret. It then prints one line that names the file and the reason, and ends with status 1 instead of a report: when the file isn't a readable [rosbag](https://github.com/ros/ros_comm/tree/25d371664e34ec9d26ee331434de9a38c412c890/tools/rosbag), when the topic's messages aren't ROS's `sensor_msgs/Imu`, when the topic has fewer than two messages, when a message has no `frame_id` or the messages have different ones, and when a reading is NaN, infinite or beyond 1,000,000 rad/s or m/s², the tool's limit for a plausible reading. For a recording that was cut off before the recorder finished, the line says that the file has no index and that `rosbag reindex` can repair it.

[`--topic`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L770-L771) selects another `sensor_msgs/Imu` topic than `/diffbot/imu/data_raw`. The 30 ms of the line `Header intervals` belong to the firmware's 50 Hz and don't adapt to a topic with another rate.

### The mean and the variance of three readings

The row `z` of the example report comes from three readings of the yaw rate. One step of the BNO055's gyroscope is $\Delta$ = 1/900 rad/s, about 0.00111 rad/s, and the three readings $\omega_{z,1}$, $\omega_{z,2}$ and $\omega_{z,3}$ are 0, $\Delta$ and $2\Delta$:

--8<-- "includes/imu-noise-three-readings.html"

**The mean** $\bar{\omega}_z$ is the sum of the $N$ = 3 readings, divided by $N$:

$$
\begin{aligned}
\bar{\omega}_z &= \frac{1}{N}\sum_{i=1}^{N} \omega_{z,i} \\
&= \frac{0 + \Delta + 2\Delta}{3} = \Delta \\
&= 0.00111\ \text{rad/s}
\end{aligned}
$$

**The sample variance** $s^2$ adds up the squared deviations of the readings from the mean, here $-\Delta$, 0 and $+\Delta$, and divides the sum by $N - 1$:

$$
\begin{aligned}
s^2 &= \frac{1}{N-1}\sum_{i=1}^{N} \left(\omega_{z,i} - \bar{\omega}_z\right)^2 \\
&= \frac{\Delta^2 + 0 + \Delta^2}{3 - 1} = \Delta^2 \\
&= 1.235 \cdot 10^{-6}\ (\text{rad/s})^2
\end{aligned}
$$

**The standard deviation** $s$ is the square root of the variance:

$$
s = \sqrt{s^2} = \Delta = 0.00111\ \text{rad/s}
$$

These are the three numbers of the row `z`: `+0.00111`, `1.235e-06` and `0.00111`. The last column of the row, `3`, counts the three different values.

The divisor is $N - 1$ and not $N$, because the deviations are measured from the readings' own mean. That mean lies closer to the readings than the true mean, which nobody knows, so the squared deviations come out a little too small. Dividing by $N - 1$ makes up for it ([Bessel's correction](https://en.wikipedia.org/wiki/Bessel%27s_correction)). With $N$, the example's variance would be two thirds of $\Delta^2$.

The excerpt shows the two functions of the tool that do this calculation, [lines 577–596 of `imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L577-L596). [`axis_statistics()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L592-L596) gets the readings of one axis as [`values`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L592). It takes their mean from Python's [`statistics.mean()`](https://docs.python.org/3.8/library/statistics.html#statistics.mean) and their sample variance from [`sample_variance()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L577-L589). That function passes the readings to Python's [`statistics.variance()`](https://docs.python.org/3.8/library/statistics.html#statistics.variance), which divides by $N - 1$, as fractions. A fraction holds a floating-point number exactly, so the mean, the deviations and their squares aren't rounded on the way:

```python title="imu_noise_at_rest.py: the sample variance and the statistics of one axis" linenums="577"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/scripts/imu_noise_at_rest.py:577:596"
```

Two readings are enough to calculate a variance, and the tool asks for no more. A recording is longer for a reason: 180 s at 50 Hz give about 9,000 readings, which describe that run far better than three. Three limits remain:

- **Only this run:** the numbers describe one pose on one day. A robot that stands tilted has other shares of gravity on its axes, so the acceleration's means differ from one pose to the next. A share that stays the same during a recording adds no spread to that recording. A tilt that changes during the recording, a table that shakes or a wheel that still turns adds to the variances like the sensor's own noise.
- **Readings that follow each other can depend on each other:** a bias that drifts slowly, for example with the temperature, moves many readings in the same direction. Then 9,000 readings say less than 9,000 independent ones would.
- **At rest only:** driving adds vibration from the motors and the floor, which a recording at rest doesn't contain.

### From the report to the EKF

One number of the report has a counterpart in the robot's configuration: the variance of the yaw rate. In the figure, first compare the two bands. They show, to scale on one axis, the standard deviation that Remo's report measured at rest and the standard deviation that the firmware's setting stands for. Then follow the solid arrows: from the setting to the covariance matrix in the IMU's message, and from the matrix's index 8 to the EKF.

--8<-- "includes/imu-noise-report-to-ekf.html"

1. **In the report:** Remo's row `z` of [`angular_velocity`](#what-the-firmware-publishes) has three numbers:
    - **The mean,** +0.00005 rad/s, is the average of all readings. A robot at rest doesn't turn, so a perfect gyroscope's mean would be 0. What's left is the bias.
    - **The standard deviation,** 0.00073 rad/s, says how far a single reading typically lies from the mean.
    - **The variance,** 5.317 · 10⁻⁷ (rad/s)², is the standard deviation squared. It holds the same information in another unit. A ROS message and the EKF take the variance, so this is the number to compare.
2. **In the message:** a `sensor_msgs/Imu` message carries the variances of its angular velocity in the field [`angular_velocity_covariance`]({{ imu_noise_src_url }}/diffbot_firmware/lib/imu/imu_publisher.h#L220-L224), a 3 × 3 matrix stored as nine numbers. The yaw rate's variance is the last of them, index 8. [The variances](robot_localization.md#the-variances) on the Localization page shows the matrix with the index of every element.
3. **In the firmware:** the firmware doesn't measure a variance. It writes one setting, [`angular_velocity_variance`]({{ imu_noise_src_url }}/diffbot_firmware/lib/config/diffbot_base_config.h#L75) = 4 · 10⁻⁴ (rad/s)², to the indices 0, 4 and 8 of every message ([What the firmware publishes](#what-the-firmware-publishes)). Its comment gives the value as (0.02 rad/s)²: the square of a standard deviation of 0.02 rad/s, about 1.1 °/s, chosen as the starting value. For scale, Bosch's datasheet gives the gyroscope a noise of typically 0.1 °/s (0.0017 rad/s) at a bandwidth of 47 Hz, and a zero-rate offset of typically ±1 °/s with the sensor's offset cancellation off ([BNO055 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf#page=15), the gyroscope's table).
4. **In the EKF:** of the IMU's values, the filter fuses only the yaw rate, so index 8 is the one that it uses. It doesn't fuse the IMU's acceleration ([What the filter fuses](robot_localization.md#what-the-filter-fuses)). The larger the variance, the less the filter trusts the IMU's yaw rate compared with the wheel odometry's.

So the measured spread at rest is a baseline for the setting. In standard deviations, the setting's 0.02 rad/s is about 27 times the measured 0.00073 rad/s. In variances, the two numbers that the EKF sees, 4 · 10⁻⁴ (rad/s)² is about 750 times 5.317 · 10⁻⁷ (rad/s)², because 27.4² ≈ 750. Three things follow:

- **The tool changes nothing:** it prints a report. A changed setting is an edit of [`diffbot_base_config.h`]({{ imu_noise_src_url }}/diffbot_firmware/lib/config/diffbot_base_config.h) and a new firmware on the Teensy.
- **The starting value stays for now:** the recording at rest doesn't contain the vibration of driving. A check while driving has to show how much the vibration adds. That check has to tell real motion apart from noise, because the readings of a robot that turns or speeds up change for a reason.
- **The bias isn't part of the variance:** the mean is in rad/s and the variance in (rad/s)². The bias doesn't go into `angular_velocity_variance`.

### Results on Remo

Remo was recorded twice for 180 s at rest: with a Teensy 3.2 on 2026-10-31 ([the bench checks' results in diffbot#160](https://github.com/ros-mobile-robots/diffbot/issues/160#issuecomment-6550353204)) and with Remo's own Teensy 4.0 on 2026-11-03 ([the Teensy 4.0 results in diffbot#169](https://github.com/ros-mobile-robots/diffbot/issues/169#issuecomment-6600460527)). The linked comments summarize the bench checks. The table gives the values from the reports that the first version of the tool printed. The recordings weren't run through the tool again.

| | Teensy 3.2 | Teensy 4.0 |
|:--|:--|:--|
| Messages and rate | 8,986 at 50.00 Hz | 8,990 at 50.00 Hz |
| Intervals between the header stamps | 14 to 27 ms, none over 30 ms | 13 to 26 ms, none over 30 ms |
| Gyroscope bias x, y, z (rad/s) | +0.00001, +0.00000, +0.00003 | −0.00003, −0.00001, +0.00005 |
| Gyroscope std dev x, y, z (rad/s) | 0.00047, 0.00078, 0.00075 | 0.00057, 0.00081, 0.00073 |
| Gyroscope variance x, y, z (10⁻⁷ (rad/s)²) | 2.246, 6.054, 5.614 | 3.194, 6.572, 5.317 |
| Gyroscope distinct values x, y, z | 7, 8, 9 | 10, 11, 9 |
| Acceleration std dev x, y, z (m/s²) | 0.020, 0.024, 0.028 | 0.013, 0.026, 0.036 |
| Acceleration distinct values x, y, z | 11, 14, 17 | 11, 19, 21 |
| Gravity | 9.742 m/s² | 9.792 m/s² |

Both boards give the same picture, as expected: the Teensy only reads the sensor's registers. The gyroscope's bias stays below 0.0001 rad/s, less than a tenth of one output step. Its standard deviation is smaller than one step too, so each axis read only 7 to 11 different values in 180 s. Its variance, 2 to 7 · 10⁻⁷ (rad/s)², is about a thousand times smaller than the firmware's starting value of 4 · 10⁻⁴ (rad/s)².

### How the tool is tested

The [tests of the noise tool](noise-at-rest-tests.md) show how to run all 82 tests. They check statistics against known results and run the command on prepared inputs. No test needs a robot or a ROS master.

## Trying the breakout on its own

To try the breakout separately with an Arduino, see [Adafruit's Arduino guide](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code) for wiring, library installation and example programs.

??? info "Trying the breakout on the Raspberry Pi"
    To check a breakout on its own, it can also be connected to the Raspberry Pi's I2C pins ([pinout](https://pinout.xyz/pinout/i2c)): VIN to 3.3 V, GND to GND, SDA to SDA, SCL to SCL. Adafruit's guide reads it with Python and its CircuitPython library: first [prepare the Pi for it](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi) (I2C on, the [Blinka](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi) library), then [slow down the Pi's I2C clock](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching), which the BNO055 needs because it stretches the clock, and then run the [BNO055 example](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/python-circuitpython).

    [ros-imu-bno055](https://github.com/dheera/ros-imu-bno055) is a C++ driver for ROS 1 and ROS 2 that reads the BNO055 over I2C. The project's README says the driver should work on a Raspberry Pi 5, or on an older Pi using software I2C. The author has not tested the slower-clock workaround with this driver. DiffBot and Remo read the IMU on the Teensy. ros-imu-bno055 has not been tested with either robot.
