# ROS Software Packages

After having verified that the hardware requirements for the Navigation Stack
are met, an overview of Remo's software follows.

## Software requirements for the ROS Navigation Stack

The [`diffbot`](https://github.com/ros-mobile-robots/diffbot/) and
[`remo_description`](https://github.com/ros-mobile-robots/remo_description)
repositories contain the following ROS packages:

- `diffbot_base`: This package contains the platform-specific code for the base
controller component required by the ROS Navigation Stack. It consists of the
firmware based on rosserial for the Teensy MCU and the C++ node running on the
SBC that instantiates the ROS Control hardware interface including the
`controller_manager` control loop for the real robot. The low-level
`base_controller` component reads the encoder ticks from the hardware,
calculates angular joint positions and velocities, and publishes them to the ROS
Control hardware interface. Using this interface makes it possible to use the
`diff_drive_controller` package from [ROS
Control](http://wiki.ros.org/diff_drive_controller). It provides a controller
(`DiffDriveController`) for a differential drive mobile base that computes
target joint velocities from commands received by either a teleop node or the
ROS Navigation Stack. The computed target joint velocities are forwarded to the
low-level base controller, where they are compared to the measured velocities to
compute suitable motor PWM signals using two separate PID controllers, one for
each motor.
- `diffbot_bringup`: Launch files to bring up the hardware driver nodes (camera,
lidar, microcontroller, and so on) as well as the C++ nodes from the
`diffbot_base` package for the real robot.
- `diffbot_control`: Configurations for `DiffDriveController` and
`JointStateController` of ROS Control used in the Gazebo simulation and the real
robot. The parameter configurations are loaded onto the parameter server with
the help of the launch files inside this package.
- `remo_description`: This package contains the URDF description of Remo
including its sensors. It allows you to pass arguments to visualize different
camera and SBC types. It also defines the `gazebo_ros_control` plugin. Remo's
description is based on the description at
https://github.com/ros-mobile-robots/mobile_robot_description, which provides a
modular URDF structure that makes it easier to model your own differential drive
robot.
- `diffbot_gazebo`: Simulation-specific launch and configuration files for Remo
and Diffbot, to be used in the Gazebo simulator.
- `diffbot_msgs`: Message definitions specific to Remo/Diffbot, for example, the
message for encoder data is defined in this package.
- `diffbot_navigation`: This package contains all the required configuration and
launch files for the ROS Navigation Stack to work.
- `diffbot_slam`: Configurations for simultaneous localization and mapping using
implementations such as gmapping to create a map of the environment.

After this overview of the ROS packages of a differential robot that fulfill the
requirements of the Navigation Stack, the next pages explain those packages in
more detail.