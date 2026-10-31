---
title: Base Package - Low Level Approach
description: "ROS Base Package for ROS Noetic running on a Raspberry Pi 4 for an autonomous 2WD Robot to act in an environment according to sensor information.
This describes the implementation of the diffbot_base package following the low-level approach."
categories: [robotics]
tags: [2wd, differential drive, robot, ros, noetic, raspberry, pi, autonomous, ubuntu, focal, package, gazebo, simulation, hardware_interfacem, hardware, interface, ros-control, control, controllers, diff_drive_controller]
---

# DiffBot Base Package - Low Level Approach

Having explained the two components of the base
controller in [DiffBot Base](index.md), the low-level firmware is implemented first, followed by the high-level hardware, detailed in the next section.

## Implementing the low-level base controller for Remo

!!! info "Firmware excerpts"
    The firmware excerpts on this page, the code blocks with a file name and
    line numbers, show the firmware at the diffbot commit that this page pins.
    The link before each excerpt opens the same lines on GitHub.

The low-level base controller is implemented on the Teensy microcontroller using
[PlatformIO](https://platformio.org/). The programming language in PlatformIO is
the same as Arduino (based on Wiring) and it is available as a plugin for the
Visual Studio Code editor. On the development PC, we can flash the robot
firmware to the board with this plugin. We will get the firmware code from the
[`diffbot_firmware`](https://github.com/ros-mobile-robots/diffbot/tree/noetic-devel/diffbot_firmware)
folder of the diffbot repository. Opening this folder in Visual Studio Code will recognize it as a
PlatformIO workspace because it contains the `platformio.ini` file. This file
defines the required dependencies and makes it straightforward to flash the
firmware to the Teensy board after compilation. Inside this file, the used
libraries are listed, here for Remo's Teensy 4.0, [lines 30–37]({{ low_level_src_url }}/diffbot_firmware/platformio.ini#L30-L37):

```ini title="platformio.ini: Remo's Teensy 4.0" linenums="30"
--8<-- "{{ low_level_src_raw }}/diffbot_firmware/platformio.ini:30:37"
```

As you can see, the firmware depends on `rosserial`, the `Adafruit Motor Shield
V2` library, and `Wire`, an I2C library. PlatformIO allows using custom
libraries defined in the local `./lib` folder, which are developed in this
section.

The firmware is used to read from encoders and ultrasonic and IMU sensors, and
receive wheel velocity commands from the high-level
`hardware_interface::RobotHW` class, discussed in the next section. The
following code snippets are part of the low-level base controller’s `main.cpp`
file and show the used libraries, found in
`diffbot_firmware`, in the `lib` and `src` folders. `src`
contains `main.cpp` consisting of the `setup()` and `loop()` functions, common
to every Arduino sketch and starts off by including the following headers, [lines 1–4]({{ low_level_src_url }}/diffbot_firmware/src/main.cpp#L1-L4):

```cpp title="main.cpp: ROS, Wire and Remo's configuration" linenums="1"
--8<-- "{{ low_level_src_raw }}/diffbot_firmware/src/main.cpp:1:4"
```

Besides the ROS header file and [`Wire`](https://www.pjrc.com/teensy/td_libs_Wire.html), the I2C
library for the motor driver and the IMU, it includes definitions specific to Remo, which are
defined in the `diffbot_base_config.h` header. It contains constant parameter
values such as the following:

- Encoder pins: Defines to which pins on the Teensy microcontroller the Hall
  effect sensors are connected.
- Motor I2C address and pins: The Adafruit motor driver can drive four DC
  motors. Due to cable management, motor terminals `M4` and `M3` are used for
  the left and right motors, respectively.
- PID: The tuned constants for both PID controllers of `base_controller`.
- PWM_MAX and PWM_MIN: The minimum and maximum possible PWM values that can be
  sent to the motor driver.
- Update rates: Defines how often functions of `base_controller` are executed.
  For example, the control portion of the low-level base controller code reads
  encoder values and writes motor commands at a specific rate.
- Command timeout: `COMMAND_TIMEOUT_MS`, how long the firmware drives the motors
  without a new wheel command, explained in [The command timeout](#the-command-timeout).
- IMU: the BNO055's address (`IMU_BNO055_ADDRESS`) and the IMU publisher's
  settings (`IMU_PUBLISHER_CONFIG`), for example `enabled` false for a robot
  without an IMU, explained on [IMU (BNO055)](../imu/driver-bno055.md).

After including Remo-specific definitions, next follows the custom libraries in
the `lib` folder, [lines 5–12]({{ low_level_src_url }}/diffbot_firmware/src/main.cpp#L5-L12):

```cpp title="main.cpp: the libraries from lib" linenums="5"
--8<-- "{{ low_level_src_raw }}/diffbot_firmware/src/main.cpp:5:12"
```

These include directives and the libraries that get included with them are
introduced next:

- `base_controller`: Defines the `BaseController` template class, defined in the
  `base_controller.h` header, and acts as the main class to manage the two
  motors, including each motor’s encoder, and communicate with the high-level
  hardware interface.
- `motor_controller_intf`: This library is indirectly included with
  `adafruit_feather_wing.h` and defines an abstract base class, named
  `MotorControllerIntf`. It is a generic interface used to operate a single
  motor using arbitrary motor drivers. It is meant to be implemented by other
  specific motor controller subclasses and therefore avoids changing code in
  classes that know the `MotorControllerIntf` interface and call its
  `setSpeed(int value)` method, such as done by `BaseController`. The only
  requirement for this to work is for a subclass to inherit from this
  `MotorControllerIntf` interface and implement the `setSpeed(int value)` class
  method.
- `adafruit_feather_wing`: This library, in the `motor_controllers` folder,
  implements the `MotorControllerIntf` abstract interface class and defines a
  concrete motor controller. For Remo, the motor controller is defined in the
  `AdafruitMotorController` class. This class has access to the motor driver
  board and serves to operate the speed of a single motor, which is why two
  instances are created in the `main.cpp` file.
- `encoder`: This library is used in the `BaseController` class and is based on
  `Encoder.h` from https://www.pjrc.com/teensy/td_libs_Encoder.html that allows
  reading encoder tick counts from quadrature encoders, like the DG01D-E motors
  consist of. The encoder library also provides a method `jointState()` to
  directly obtain the joint state, which is returned by this method in the
  `JointState` struct, that consists of the measured angular position (rad) and
  angular velocity (rad/s) of the wheel joints, [lines 14–39]({{ low_level_src_url }}/diffbot_firmware/lib/encoder/encoder_diffbot.cpp#L14-L39):

    <a id="the-encoder-excerpt"></a>

    ```cpp title="encoder_diffbot.cpp: angle and speed" linenums="14" hl_lines="8 11"
    --8<-- "{{ low_level_src_raw }}/diffbot_firmware/lib/encoder/encoder_diffbot.cpp:14:39"
    ```

    The time between two calls comes from the Teensy's own clock,
    [`micros()`](https://docs.arduino.cc/language-reference/en/functions/time/micros/),
    in microseconds since the board started, for the reason explained in the
    [`loop()` section](#timing-with-the-teensys-clock) below.

- `pid`: Defines a PID controller to compute PWM signals based on the velocity
  error between measured and commanded angular wheel joint velocities. For more
  infos about PIDs and the tuning sections refer to [PID Controllers](pid.md)
- `command_timeout`: Decides when the wheel commands from the robot's computer
  are too old, explained in [The command timeout](#the-command-timeout).
- `drive_step`: Holds `loop()`'s command timeout and control step in one
  function, `driveStep()`, so that unit tests can check their order with a fake
  controller.
- `imu`: The BNO055's driver (`bno055.h`) and publisher (`imu_publisher.h`),
  with the I2C bus (`wire_bus.h`) and the node handle's clock synchronization
  (`synced_node_handle.h`), explained on [IMU (BNO055)](../imu/driver-bno055.md).


### The firmware's parts

The diagrams show the firmware's parts and how they connect, first as they are, then as diffbot#148 plans them. In them, a solid arrow points from a part to the part it uses. A dashed arrow points from an adapter to the interface it implements. A line without an arrow is a message connection, and the messages can go either way or both ways.

=== "Today"

    <div class="wide-diagram" markdown>

    ```mermaid
    graph TB
      subgraph PI [Robot's computer]
        HW[diffbot_base<br/>hardware interface]
        ROS[Other ROS nodes]
        SN[rosserial_python's<br/>serial_node.py]
      end
      subgraph TEENSY [Teensy firmware]
        NH[SyncedNodeHandleT:<br/>rosserial's node handle]
        LOOP[loop]
        DS[driveStep and<br/>CommandTimeout]
        BC[BaseController:<br/>encoders, PIDs,<br/>motor controllers]
        IP[ImuPublisher]
        BNO[Bno055 driver<br/>on an I2cBus]
      end
      FW[Motor driver<br/>FeatherWing]
      IMU[BNO055 sensor]
      HW ---|wheel topics| SN
      ROS ---|imu/data_raw| SN
      SN ---|USB| NH
      NH ---|wheel topics| BC
      NH ---|imu/data_raw| IP
      LOOP --> DS
      DS --> BC
      LOOP --> IP
      IP --> BNO
      BC -->|I2C| FW
      BNO -->|I2C| IMU
    ```

    </div>

    `BaseController` holds most of the wheel control: the encoders, the two PIDs, the motor controllers and the rosserial topics. `driveStep()` and `CommandTimeout` decide when it drives. The IMU's parts are separate: the driver reaches the sensor only through `I2cBus`, so the unit tests replace the bus with a fake one.

=== "Planned"

    <div class="wide-diagram" markdown>

    ```mermaid
    graph TB
      subgraph PI [Robot's computer]
        HWI[Hardware interface:<br/>joints bound<br/>to channels]
      end
      subgraph TEENSY [Teensy firmware]
        W[Board wiring:<br/>one file<br/>per board]
        CORE[Portable core:<br/>states, wheel control,<br/>PID, command timeout,<br/>IMU sampler,<br/>typed settings]
        PORTS[Device interfaces:<br/>link, motor output,<br/>encoder, I2C bus,<br/>clock]
        ADAPT[Teensy adapters:<br/>serial protocol,<br/>motor drivers,<br/>encoders, Wire,<br/>millis]
      end
      HWI ----|USB| ADAPT
      W --> CORE
      W --> ADAPT
      CORE --> PORTS
      ADAPT -.->|implement| PORTS
    ```

    </div>

    Over USB, a versioned serial protocol replaces rosserial. Messages carry one record per channel. Each record includes the channel ID. The hardware interface maps each joint name to a channel ID. The core sees the hardware only through the device interfaces, which the Teensy adapters implement. So the unit tests on the PC replace the adapters with stand-ins. The board wiring creates the adapters and hands them to the core. [diffbot#148](https://github.com/ros-mobile-robots/diffbot/issues/148) plans this in steps, with [diffbot#165](https://github.com/ros-mobile-robots/diffbot/issues/165) for the IMU sampler and [diffbot#166](https://github.com/ros-mobile-robots/diffbot/issues/166) for the settings.

With these libraries, we look at the `main.cpp` file. Inside it exists only a
few global variables to keep the code organized and make it possible to test the
individual components that get included. The main code is explained next:

1. First, we define the global ROS node handle, which is referenced in other
   classes, such as BaseController, where it is needed to publish and
   subscribe. Its `ros::NodeHandle::now()` provides ROS time for time stamps.
   The time between updates comes from the Teensy's own clock, as the
   [`loop()` section](#timing-with-the-teensys-clock) below explains,
   [lines 15–17]({{ low_level_src_url }}/diffbot_firmware/src/main.cpp#L15-L17):

    ```cpp title="main.cpp: the node handle" linenums="15"
    --8<-- "{{ low_level_src_raw }}/diffbot_firmware/src/main.cpp:15:17"
    ```

    `SyncedNodeHandleT`, in [`synced_node_handle.h`]({{ low_level_src_url }}/diffbot_firmware/lib/imu/synced_node_handle.h),
    is rosserial's node handle with one addition: it tells when its clock is
    synchronized with the robot's computer. The IMU publisher waits for that
    before it stamps its messages.

2. For convenience and to keep the code organized, we declare that we want to
   use the `diffbot` namespace, where the libraries of the base controller are
   declared, [line 19]({{ low_level_src_url }}/diffbot_firmware/src/main.cpp#L19-L19):

    ```cpp title="main.cpp: the namespace" linenums="19"
    --8<-- "{{ low_level_src_raw }}/diffbot_firmware/src/main.cpp:19:19"
    ```

3. Next, we define two concrete motor controllers of type
   `AdafruitMotorController` found in the `motor_controllers` library,
   [lines 21–22]({{ low_level_src_url }}/diffbot_firmware/src/main.cpp#L21-L22):

    ```cpp title="main.cpp: the motor controllers" linenums="21"
    --8<-- "{{ low_level_src_raw }}/diffbot_firmware/src/main.cpp:21:22"
    ```

    This class inherits from the abstract base class `MotorControllerIntf`,
    explained above. It knows how to connect to the Adafruit motor driver using its
    open source `Adafruit_MotorShield` library
    (https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/library-reference)
    and how to get a C++ pointer to one of its DC motors (`getMotor(motor_num)`).
    Depending on the integer input value to `AdafruitMotorController::setSpeed(int
    value)`, the DC motor is commanded to rotate in a certain direction and at a
    specified speed. For Remo, the range is between –255 and 255, specified by the
    `PWM_MAX` and `PWM_MIN` identifiers.

4. The next class that is defined globally inside main is `BaseController`,
   which incorporates most of the main logic of this low-level base controller,
   [line 24]({{ low_level_src_url }}/diffbot_firmware/src/main.cpp#L24-L24):

    ```cpp title="main.cpp: the base controller" linenums="24"
    --8<-- "{{ low_level_src_raw }}/diffbot_firmware/src/main.cpp:24:24"
    ```

    As you can see, it is a template class that accepts different kinds of motor
    controllers (`TMotorController`, which equals `AdafruitMotorController` in the
    case of Remo) that operate on different motor drivers (`TMotorDriver`, which
    equals `Adafruit_MotorShield`), using the `MotorControllerIntf` interface as
    explained previously. The `BaseController` constructor takes a reference to the
    globally defined ROS node handle and the two motor controllers to let it set the
    commanded speeds computed through two separate PID controllers, one for each
    wheel.

    In addition to setting up pointers to the motor controllers, the
    `BaseController` class initializes two instances of type `diffbot::Encoder`. Its
    measured joint state, returned from `diffbot::Encoder::jointState()`, is used
    together with the commanded wheel joint velocities in the `diffbot::PID`
    controllers to compute the velocity error and output an appropriate PWM signal
    for the motors.

5. Last, the command timeout, and the IMU, which shares the motor driver's I2C
   bus, [lines 26–45]({{ low_level_src_url }}/diffbot_firmware/src/main.cpp#L26-L45):

    ```cpp title="main.cpp: the command timeout and the IMU" linenums="26"
    --8<-- "{{ low_level_src_raw }}/diffbot_firmware/src/main.cpp:26:45"
    ```

    `command_timeout` gets `COMMAND_TIMEOUT_MS`, see
    [The command timeout](#the-command-timeout). `ArduinoClock` gives the IMU
    publisher the Teensy's clock, which the unit tests replace with a fake one.
    `wire_bus`, `imu` and `imu_publisher` read the BNO055 and publish
    `imu/data_raw`, explained on [IMU (BNO055)](../imu/driver-bno055.md). The
    [anonymous namespace](https://en.cppreference.com/cpp/language/namespace#Unnamed_namespaces)
    makes these objects private to `main.cpp`.

After defining the global instances, the firmware’s `setup()` function is
discussed next. The low-level BaseController class communicates with the
high-level interface `DiffBotHWInterface` using ROS publishers and subscribers.
These are set up in the `Basecontroller::setup()` method, which is called in the
`setup()` function of `main.cpp`. In addition to that, the
`BaseController::init()` method is here to read parameters stored on the ROS
parameter server, such as the wheel radius and distance between the wheels.
Beside initializing `BaseController`, the communication frequency of the motor
driver is configured, [lines 48–60]({{ low_level_src_url }}/diffbot_firmware/src/main.cpp#L48-L60):

```cpp title="main.cpp: setup()" linenums="48" hl_lines="4 5"
--8<-- "{{ low_level_src_raw }}/diffbot_firmware/src/main.cpp:48:60"
```

With the IMU enabled, `imu_publisher.advertise()` registers the `imu/data_raw`
publisher before `base_controller.setup()` connects, so that rosserial announces
it during the connection. `Wire.begin()` starts the I2C bus for the motor driver
and the IMU.

The `begin(uint16_t freq)` method of the motor controllers has to be called
explicitly in the main `setup()` function because `MotorControllerIntf` doesn't
provide a `begin()` or `setup()` method in its interface. This is a design
choice that, when added, would make the `MotorControllerIntf` less generic.

After the `setup()` function follows the `loop()` function, to read from sensors
and write to actuators, which happens at specific rates, defined in the
`diffbot_base_config.h` header. The bookkeeping of when these read/write
functionalities occurred is kept in the `BaseController` class inside its
`LastUpdateTime` struct, which `lastUpdateTime()` returns. `loop()` reads the
Teensy's clock once per pass and hands the command timeout and the control step
to `driveStep()`, [lines 63–87]({{ low_level_src_url }}/diffbot_firmware/src/main.cpp#L63-L87):

<a id="the-loop-excerpt"></a>

```cpp title="main.cpp: loop()" linenums="63" hl_lines="4 8 12"
--8<-- "{{ low_level_src_raw }}/diffbot_firmware/src/main.cpp:63:87"
```

`driveStep()`, in [`drive_step.h`]({{ low_level_src_url }}/diffbot_firmware/lib/drive_step/drive_step.h), checks the command
timeout first, then runs the control step, [lines 17–37]({{ low_level_src_url }}/diffbot_firmware/lib/drive_step/drive_step.h#L17-L37):

```cpp title="drive_step.h: the command timeout, then the control step" linenums="17" hl_lines="5 8 15"
--8<-- "{{ low_level_src_raw }}/diffbot_firmware/lib/drive_step/drive_step.h:17:37"
```

The following steps of the control step happen continuously at the control rate:

1. Encoder sensor values are read through the `BaseController::read()` method
   and the data is published inside this method for the high-level
   `DiffbotHWInterface` class, on the `measured_joint_states` topic of message
   type `sensor_msgs::JointState`.
2. The `BaseController` class subscribes to `DiffBotHWInterface` from which it
   receives the commanded wheel joint velocities (topic `wheel_cmd_velocities`,
   type `diffbot_msgs::WheelsCmdStamped`) inside the
   `BaseController::commandCallback(const diffbot_msgs::WheelsCmdStamped&)`
   callback method. In `BaseController::write()`, the PID is called to compute
   the motor PWM signals from the velocity error and the motor speeds are set
   with the two motor controllers. `driveStep()` calls `write()` only while the
   wheel commands are new.
3. To keep calling this method at the desired control rate, the
   `lastUpdateTime().control` variable is updated with the current time.

Before the control step, `driveStep()` checks the command timeout, explained in
[The command timeout](#the-command-timeout) below.

After `driveStep()`, `imu_publisher.update()` reads the BNO055 and publishes
`imu/data_raw` at 50 Hz, unless `enabled` is false in `IMU_PUBLISHER_CONFIG`
([IMU (BNO055)](../imu/driver-bno055.md)). The debug block prints the encoder
readings when debugging is on. Finally, all the callbacks waiting in the ROS
callback queue are processed with a call to `nh.spinOnce()`.

### The command timeout

When the robot's computer stops sending wheel commands, for example because the
bringup stopped or the USB connection broke, the firmware stops the motors. The
hardware interface on the robot's computer sends a command 10 times a second,
also while the robot stands. `COMMAND_TIMEOUT_MS` is 1000 ms in
[`diffbot_base_config.h`]({{ low_level_src_url }}/diffbot_firmware/lib/config/diffbot_base_config.h#L36-L39),
so the firmware stops after about ten missed commands.

`CommandTimeout` decides when the commands are too old, [lines 27–49]({{ low_level_src_url }}/diffbot_firmware/lib/command_timeout/command_timeout.h#L27-L49):

```cpp title="command_timeout.h: update() and timedOut()" linenums="27" hl_lines="7 13"
--8<-- "{{ low_level_src_raw }}/diffbot_firmware/lib/command_timeout/command_timeout.h:27:49"
```

- **`update()` returns true once,** on the pass when the last command becomes too
  old. So the motors are stopped and the timeout is logged once, not on every
  pass.
- **Only a new command ends the timeout.** The command callback counts the
  commands in `command_count_`, and `CommandTimeout` notices a new one by the
  changed count, not by its time, because an old time can look new again once
  `millis()` has gone around ([When the counters wrap](#when-the-counters-wrap)).
- **It starts timed out,** so the motors aren't driven before the first command.

When `update()` returns true, `driveStep()` calls `eStop()`, [lines 564–575]({{ low_level_src_url }}/diffbot_firmware/lib/base_controller/base_controller.h#L564-L575):

```cpp title="base_controller.h: eStop()" linenums="564"
--8<-- "{{ low_level_src_raw }}/diffbot_firmware/lib/base_controller/base_controller.h:564:575"
```

`eStop()` sets both target wheel velocities to zero, resets both PIDs, so that no
stored error drives the motors later, and sets both motors to zero. On the
Adafruit FeatherWing, zero releases a motor ([setSpeed()]({{ low_level_src_url }}/diffbot_firmware/lib/motor_controllers/adafruit_feather_wing/adafruit_feather_wing.cpp#L16-L36)), so the
wheels coast without braking.
Only then the firmware logs the timeout, because sending the log message can
wait for the USB connection. The control step keeps reading and publishing the
wheel states, but doesn't drive the motors until the next command arrives.

So the timeout is when the firmware stops driving, not a stopping distance.
Unit tests on the PC check the timeout, the order in `driveStep()` and the PID's
reset, in the [firmware container](../../development/firmware-container.md):
`.docker/firmware/run.sh pio test -e native`.

### Timing with the Teensy's clock

The firmware measures three kinds of elapsed time: whether 50 ms have passed
since the last control step, whether 1 s has passed since the last wheel
command, and how long the encoders took since their last reading. It takes all
three from the Teensy's own counters.
[`millis()`](https://docs.arduino.cc/language-reference/en/functions/time/millis/)
counts the milliseconds since the Teensy started, for the control step and the
command's age.
[`micros()`](https://docs.arduino.cc/language-reference/en/functions/time/micros/)
counts microseconds, for the encoders' speed. A correction of the Raspberry Pi's
clock doesn't change these counters.

#### The time stamps

`BaseController` keeps one `millis()` reading for each event in
`LastUpdateTime`, [lines 175–196]({{ low_level_src_url }}/diffbot_firmware/lib/base_controller/base_controller.h#L175-L196):

```cpp title="base_controller.h: the time stamps" linenums="175"
--8<-- "{{ low_level_src_raw }}/diffbot_firmware/lib/base_controller/base_controller.h:175:196"
```

`command_received`, `control` and `debug` are the times of the last wheel
command, control step and debug message. The `imu` field gets its start value
but isn't used: the IMU publisher keeps its own sampling time.
When the `BaseController` is created, all four get the same reading
([line 402]({{ low_level_src_url }}/diffbot_firmware/lib/base_controller/base_controller.h#L402-L402)).

The command callback updates `command_received` and counts the command in
`command_count_`, each time a wheel command arrives on `wheel_cmd_velocities`, [lines 473–485]({{ low_level_src_url }}/diffbot_firmware/lib/base_controller/base_controller.h#L473-L485):

```cpp title="base_controller.h: a wheel command arrives" linenums="473" hl_lines="11 12"
--8<-- "{{ low_level_src_raw }}/diffbot_firmware/lib/base_controller/base_controller.h:473:485"
```

#### The control step and the command timeout

The [`loop()` excerpt](#the-loop-excerpt) above reads the counter once per pass, as `now_ms`, and
`driveStep()` compares it with these time stamps:

- **The control step** runs when at least `control_period_ms` milliseconds have
  passed since the last one. `loop()` passes `1000.0 / control_`. With
  `UPDATE_RATE_CONTROL` set to 20 (Hz) in
  [`diffbot_base_config.h`]({{ low_level_src_url }}/diffbot_firmware/lib/config/diffbot_base_config.h#L30-L30),
  that's 50 ms. The step records its time in `control`. So the firmware runs
  at most 20 steps per second. Work that blocks the loop, for example a slow
  serial transfer, makes a step late.
- **The command timeout:** `CommandTimeout::update()` compares the age of the
  last wheel command with `COMMAND_TIMEOUT_MS`, 1000 ms.

#### The encoders' speed

Each control step reads both encoders. `Encoder::jointState()`, in the
[encoder excerpt](#the-encoder-excerpt) above, divides the angle that the wheel
turned by the time since its last reading, from `micros()`. `dts` is that time
in seconds, and `angular_velocity_` is the wheel's speed in
radians per second, although a comment in the code mentions RPM.

#### When the counters wrap

`millis()` and `micros()` return 32-bit unsigned integers. After 2³² counts they
return to zero: 2³² ms (4,294,967,296 ms) is about 49.7 days, and 2³² µs about
71.6 minutes. Arduino's reference pages for `millis()` and `micros()` round these
to about 50 days and 70 minutes. The differences are computed with
`uint32_t`, so `now_ms - control` still gives the right number of milliseconds
across that return to zero, as long as the two readings are less than one
counter cycle apart and come from the same run of the firmware. That's also why
a new command, not its time, ends the command timeout.

??? warning "Pitfalls when measuring time on a microcontroller"
    - **Don't measure intervals with ROS time.** On the Teensy, `nh.now()`
      returns ROS time, which rosserial synchronizes with the robot's computer.
      When that computer's clock is corrected, ROS time can jump. A jump backward
      makes a control step or the command timeout wait too long. A jump forward
      makes a recent command look too old. Clock corrections on the robot's
      computer don't change the Teensy's counters.
    - **Check the unit and type of every time value.** Storing 0.05 s in an
      integer field for whole seconds truncates it to 0, and a check against 0
      passes on every pass of the loop. Keep intervals in milliseconds or
      microseconds.
    - **The step rate is part of the controller.** The PID controller adds up
      the error once per step. At another rate, the same gains make the wheels
      respond differently. Tune the gains at the rate that the firmware runs.

This describes the low-level base controller. For more details and the complete
library code, please refer to the [`diffbot_firmware`](https://github.com/ros-mobile-robots/diffbot/tree/noetic-devel/diffbot_firmware)
folder.

In the following section, the `diffbot::DiffBotHWInterface` class is described.


## ROS Control high-level hardware interface for a differential drive robot

The [ros_control](http://wiki.ros.org/ros_control) meta package contains the
hardware interface class `hardware_interface::RobotHW`, which needs to be
implemented to leverage many available controllers from the `ros_controllers`
meta package. First, we’ll look at the `diffbot_base` node that instantiates and
uses the hardware interface:

1. The `diffbot_base` node includes the `diffbot_hw_interface.h` header, as well
   as the `controller_manager`, defined in `controller_manager.h`, to create the
   control loop (read, update, write):

    ```cpp
    #include <ros/ros.h>
    #include <diffbot_base/diffbot_hw_interface.h>
    #include <controller_manager/controller_manager.h>
    ```

2. Inside the main function of this `diffbot_base` node, we define the ROS node
   handle, the hardware interface (`diffbot_base::DiffBotHWInterface`), and pass
   it to the `controller_manager`, so that it has access to its resources:

    ```cpp
    ros::NodeHandle nh;
    diffbot_base::DiffBotHWInterface diffBot(nh);
    controller_manager::ControllerManager cm(&diffBot);
    ```

3. Next, set up a separate thread that will be used to service ROS callbacks.
   This runs the ROS loop in a separate thread as service callbacks can block
   the control loop:

    ```cpp
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ```

4. Then define at which rate the control loop of the high-level hardware
   interface should run. For Remo, we choose 10 Hz:

    ```cpp
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0); rate.sleep(); // 10 Hz rate
    ```

5. Inside the blocking while loop of the `diffbot_base` node, we do basic
   bookkeeping to get the system time to compute the control period:

    ```cpp
    while (ros::ok()) {
      const ros::Time time = ros::Time::now();
      const ros::Duration period = time - prev_time;
      prev_time = time;
      ...
    ```

6. Next, we execute the control loop steps: read, update, and write. The
   `read()` method is here to get sensor values, while `write()` writes commands
   that were computed by `diff_drive_controller` during the `update()` step:

    ```cpp
      ...
      diffBot.read(time, period);
      cm.update(time, period);
      diffBot.write(time, period);
      ...
    ```

7. These steps keep getting repeated with the specified rate using
   `rate.sleep()`.

After having defined the code that runs the main control loop of the
`diffbot_base` node, we’ll take a look at the implementation of
`diffbot::DiffBotHWInterface`, which is a child class of
`hardware_interface::RobotHW`. With it, we register the hardware and implement
the `read()` and `write()` methods, used above in the control loop.

The constructor of the `diffbot::DiffBotHWInterface` class is used to get
parameters from the parameter server, such as the `diff_drive_controller`
configuration from the `diffbot_control` package. Inside the constructor, the
wheel command publisher and measured joint state subscriber are initialized.
Another publisher is `pub_reset_encoders_`, which is used in the
`isReceivingMeasuredJointStates` method to reset the encoder ticks to zero after
receiving measured joint states from the low-level base controller.

After constructing `DiffBotHWInterface`, we create instances of
`JointStateHandles` classes (used only for reading) and `JointHandle` classes
(used for read, and write access) for each controllable joint and register them
with the `JointStateInterface` and `VelocityJointInterface` interfaces,
respectively. This enables the `controller_manager` to manage access for joint
resources of multiple controllers. Remo uses `DiffDriveController` and
`JointStateController`:

```cpp
for (unsigned int i = 0; i < num_joints_; i++) {
    hardware_interface::JointStateHandle joint_state_handle(joint_names_[i], &joint_positions_[i], &joint_velocities_[i], &joint_efforts_[i]);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle, &joint_velocity_commands_[i]);
    velocity_joint_interface_.registerHandle(joint_handle); 
}
```

The last step that is needed to initialize the hardware resources is to register
the `JointStateInterface` and the `VelocityJointInterface` interfaces with the
robot hardware interface itself, thereby grouping the interfaces together to
represent Remo robot in software:

```cpp
registerInterface(&joint_state_interface_);
registerInterface(&velocity_joint_interface_);
```

Now that the hardware joint resources are registered and the controller manager
knows about them, it’s possible to call the `read()` and `write()` methods of
the hardware interface. The controller manager update happens in between the
read and write steps. Remo subscribes to the `measured_joint_states` topic,
published by the low-level base controller. The received messages on this topic
are stored in the `measured_joint_states_ array` of type
`diffbot_base::JointState` using the `measuredJointStateCallback` method, and
are relevant in the `read()` method:

1. The `read()` method is here to update the measured joint values with the current sensor readings from the encoders – angular positions (rad) and velocities (rad/s):

    ```cpp
    void DiffBotHWInterface::read() {
        for (std::size_t i = 0; i < num_joints_; ++i) {
        joint_positions[i] = measured_joint_states[i].angular_position;
        joint_velocity[i] = measured_joint_states[i].angular_velocity;
    }
    ```

2. The final step of the control loop is to call the `write()` method of the `DiffBotHWInterface` class to publish the angular wheel velocity commands of each joint, computed by `diff_drive_controller`:

    ```cpp
    void DiffBotHWInterface::write() {
        diffbot_msgs::WheelsCmdStamped wheel_cmd_msg;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            wheel_cmd_msg.wheels_cmd.angular_velocities.joint.push_back(joint_velocity_commands_[i]); 
        }
        pub_wheel_cmd_velocities_.publish(wheel_cmd_msg); 
    }
    ```

In this method, it would be possible to correct for steering offsets due to
model imperfections and slight differences in the wheel radii. See [gain / trim
model](pid.md#gain-trim-model).

This concludes the important parts of the `DiffBotHWInterface` class and enables
Remo to satisfy the requirements to work with the ROS Navigation Stack. In the
next section, we’ll look at how to bring up the robot hardware and how the
started nodes interact with each other.
