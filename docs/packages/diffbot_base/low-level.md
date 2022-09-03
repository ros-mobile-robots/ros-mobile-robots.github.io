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

The low-level base controller is implemented on the Teensy microcontroller using
[PlatformIO](https://platformio.org/). The programming language in PlatformIO is
the same as Arduino (based on Wiring) and it is available as a plugin for the
Visual Studio Code editor. On the development PC, we can flash the robot
firmware to the board with this plugin. We will get the firmware code from the
`diffbot_base` ROS package, located in the
[`scripts/base_controller`](https://github.com/ros-mobile-robots/diffbot/tree/noetic-devel/diffbot_base/scripts/base_controller)
subfolder. Opening this folder in Visual Studio Code will recognize it as a
PlatformIO workspace because it contains the `platformio.ini` file. This file
defines the required dependencies and makes it straightforward to flash the
firmware to the Teensy board after compilation. Inside this file, the used
libraries are listed:

```console
lib_deps = frankjoshua/Rosserial Arduino Library@^0.9.1
           adafruit/Adafruit Motor Shield V2 Library@^1.0.11
           Wire
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
`diffbot_base/scripts/base_controller`, in the `lib` and `src` folders. `src`
contains `main.cpp` consisting of the `setup()` and `loop()` functions, common
to every Arduino sketch and starts off by including the following headers:

```cpp
#include <ros.h>
#include "diffbot_base_config.h"
```

Besides the ros header file, it includes definitions specific to Remo, which are
defined in the `diffbot_base_config.h` header. It contains constant parameter
values such as the following:

- Encoder pins: Defines to which pins on the Teensy microcontroller the Hall
  effect sensors are connected.
- Motor I2C address and pins: The Adafruit motor driver can drive four DC
  motors. Due to cable management, motor terminals `M3` and `M4` are used for
  the left and right motors, respectively.
- PID: The tuned constants for both PID controllers of `base_controller`.
- PWM_MAX and PWM_MIN: The minimum and maximum possible PWM values that can be
  sent to the motor driver.
- Update rates: Defines how often functions of `base_controller` are executed.
  For example, the control portion of the low-level base controller code reads
  encoder values and writes motor commands at a specific rate.

After including Remo-specific definitions, next follows the custom libraries in
the `lib` folder:

    ```cpp
    #include "base_controller.h"
    #include "adafruit_feather_wing/adafruit_feather_wing.h"
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
  angular velocity (rad/s) of the wheel joints:

    ```cpp
    diffbot::JointState diffbot::Encoder::jointState() {
        long encoder_ticks = encoder.read();
        ros::Time current_time = nh_.now();
        ros::Duration dt = current_time - prev_update_time_;
        double dts = dt.toSec();
        double delta_ticks = encoder_ticks - prev_encoder_ticks_;
        double delta_angle = ticksToAngle(delta_ticks);
        joint_state_.angular_position_ += delta_angle;
        joint_state_.angular_velocity_ = delta_angle / dts;
        prev_update_time_ = current_time;
        prev_encoder_ticks_ = encoder_ticks;
        return joint_state_;
    }
    ```

- `pid`: Defines a PID controller to compute PWM signals based on the velocity
  error between measured and commanded angular wheel joint velocities. For more
  infos about PIDs and the tuning sections refer to [PID Controllers](pid.md)


With these libraries, we look at the `main.cpp` file. Inside it exists only a
few global variables to keep the code organized and make it possible to test the
individual components that get included. The main code is explained next:

1. First, we define the global ROS node handle, which is referenced in other
   classes, such as BaseController, where it is needed to publish, subscribe, or
   get the current time, using `ros::NodeHandle::now()`, to keep track of the
   update rates:

    ```cpp
    ros::NodeHandle nh;
    ```

2. For convenience and to keep the code organized, we declare that we want to
   use the `diffbot` namespace, where the libraries of the base controller are
   declared:

    ```cpp
    using namespace diffbot;
    ```

3. Next, we define two concrete motor controllers of type
   `AdafruitMotorController` found in the `motor_controllers` library:

    ```cpp
    AdafruitMotorController motor_controller_right = AdafruitMotorController(3);
    AdafruitMotorController motor_controller_left = AdafruitMotorController(4);
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
   which incorporates most of the main logic of this low-level base controller:

    ```cpp
    BaseController<AdafruitMotorController, Adafruit_MotorShield> base_controller(nh, &motor_controller_left, &motor_controller_right);
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

After defining the global instances, the firmware’s `setup()` function is
discussed next. The low-level BaseController class communicates with the
high-level interface `DiffBotHWInterface` using ROS publishers and subscribers.
These are set up in the `Basecontroller::setup()` method, which is called in the
`setup()` function of `main.cpp`. In addition to that, the
`BaseController::init()` method is here to read parameters stored on the ROS
parameter server, such as the wheel radius and distance between the wheels.
Beside initializing `BaseController`, the communication frequency of the motor
driver is configured:

```cpp
void setup() {
  base_controller.setup();
  base_controller.init();
  motor_controller_left.begin();
  motor_controller_right.begin();
}
```

The `begin(uint16_t freq)` method of the motor controllers has to be called
explicitly in the main `setup()` function because `MotorControllerIntf` doesn't
provide a `begin()` or `setup()` method in its interface. This is a design
choice that, when added, would make the `MotorControllerIntf` less generic.

After the `setup()` function follows the `loop()` function, to read from sensors
and write to actuators, which happens at specific rates, defined in the
`diffbot_base_config.h` header. The bookkeeping of when these read/write
functionalities occurred is kept in the `BaseController` class inside its
`lastUpdateRates` struct. Reading from the encoders and writing motor commands
happens in the same code block as the control rate:

```cpp
void loop() {
ros::Duration command_dt = nh.now() - base_controller.lastUpdateTime().control;
if (command_dt.toSec() >= ros::Duration(1.0 / base_controller.publishRate().control_, 0).toSec()) {
  base_controller.read();
  base_controller.write();
  base_controller.lastUpdateTime().control = nh.now();
}
```

The following steps in this code block happen continuously at the control rate:

1. Encoder sensor values are read through the `BaseController::read()` method
   and the data is published inside this method for the high-level
   `DiffbotHWInterface` class, on the `measured_joint_states` topic of message
   type `sensor_msgs::JointState`.
2. The `BaseController` class subscribes to `DiffBotHWInterface` from which it
   receives the commanded wheel joint velocities (topic `wheel_cmd_velocities`,
   type `diffbot_msgs::WheelsCmdStamped`) inside the
   `BaseController::commandCallback(const diffbot_msgs::WheelsCmdStamped&)`
   callback method. In `BaseController::read()`, the PID is called to compute
   the motor PWM signals from the velocity error and the motor speeds are set
   with the two motor controllers.
3. To keep calling this method at the desired control rate, the
   `lastUpdateTime().control` variable is updated with the current time.

After the control loop update block, if an IMU is used, its data could be read
at the imu rate and published for a node that fuses the data with the encoder
odometry to obtain more precise odometry. Finally, in the main `loop()`, all the
callbacks waiting in the ROS callback queue are processed with a call to
`nh.spinOnce()`.

MThis describes the low-level base controller. For more details and the complete
library code, please refer to the `diffbot_base/scripts/base_controller`
package. 

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
model](pid.md#gain--trim-model).

This concludes the important parts of the `DiffBotHWInterface` class and enables
Remo to satisfy the requirements to work with the ROS Navigation Stack. In the
next section, we’ll look at how to bring up the robot hardware and how the
started nodes interact with each other.
