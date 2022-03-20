# ROS Software Packages

After having verified that the hardware requirements for the Navigation Stack are met, an
overview of Remo's software follows.

## Software requirements for the ROS Navigation Stack

The [`diffbot`](https://github.com/ros-mobile-robots/diffbot/) and
[`remo_description`](https://github.com/ros-mobile-robots/remo_description) repositories 
contain the following ROS packages:

- `diffbot_base`: This package contains the platform-specific code for the base
controller component required by the ROS Navigation Stack. It consists of the
firmware based on rosserial for the Teensy MCU and the C++ node running
on the SBC that instantiates the ROS Control hardware interface including the
`controller_manager` control loop for the real robot. The low-level `base_controller`
component reads the encoder ticks from the hardware, calculates
angular joint positions and velocities, and publishes them to the ROS Control
hardware interface. Using this interface makes it possible to use the `diff_drive_controller`
package from [ROS Control](http://wiki.ros.org/diff_drive_controller). 
It provides a controller (`DiffDriveController`) for a
differential drive mobile base that computes target joint velocities from commands
received by either a teleop node or the ROS Navigation Stack. The computed
target joint velocities are forwarded to the low-level base controller, where they are
compared to the measured velocities to compute suitable motor PWM signals using
two separate PID controllers, one for each motor.
- `diffbot_bringup`: Launch files to bring up the hardware driver nodes (camera,
lidar, microcontroller, and so on) as well as the C++ nodes from the `diffbot_base` 
package for the real robot.
- `diffbot_control`: Configurations for `DiffDriveController` and
`JointStateController` of ROS Control used in the Gazebo simulation and the
real robot. The parameter configurations are loaded onto the parameter server with
the help of the launch files inside this package.
- `remo_description`: This package contains the URDF description of Remo
including its sensors. It allows you to pass arguments to visualize different
camera and SBC types. It also defines the `gazebo_ros_control` plugin.
Remo's description is based on the description at https://github.com/ros-mobile-robots/mobile_robot_description,
which provides a modular URDF structure that makes it easier to model your own differential drive robot.
- `diffbot_gazebo`: Simulation-specific launch and configuration files for Remo
and Diffbot, to be used in the Gazebo simulator.
- `diffbot_msgs`: Message definitions specific to Remo/Diffbot, for example, the
message for encoder data is defined in this package.
- `diffbot_navigation`: This package contains all the required configuration and
launch files for the ROS Navigation Stack to work.
- `diffbot_slam`: Configurations for simultaneous localization and mapping using
implementations such as gmapping to create a map of the environment.

After this overview of the ROS packages of a differential robot that fulfill the requirements
of the Navigation Stack, the next section implements the base controller component.


## Developing a low-level controller and a highlevel ROS Control hardware interface for a differential drive robot

In the following two sections, the base controller, mentioned in the Navigation Stack, will be developed. 

![Navigation Stack]({{ asset_dir }}/navigation/navigation_stack.png)

For Remo, this platform-specific node is split into two software components.
The first component is the high-level `diffbot::DiffBotHWInterface` that
inherits from `hardware_interface::RobotHW`, acting as an interface between
robot hardware and the packages of ROS Control that communicate with the Navigation
Stack and provide [`diff_drive_controller`](http://wiki.ros.org/diff_drive_controller) – 
one of many available controllers from ROS Control. With the
`gazebo_ros_control` plugin, the same controller including its configuration can be
used in the simulation and the real robot. An overview of ROS Control in a simulation
and the real world is given in the following figure (http://gazebosim.org/tutorials/?tut=ros_control):


<figure markdown>
  ![ROS Control simulation and reality]({{ asset_dir }}/packages/ros-control-simulation-and-reality.svg)
  <figcaption>ROS Control in simulation and reality</figcaption>
</figure>

The second component is the low-level base controller that measures angular wheel
joint positions and velocities and applies the commands from the high-level interface
to the wheel joints. The following figure shows the communication between the two
components:


<figure markdown>
  ![ROS Control Simulation and Reality]({{ asset_dir }}/packages/ros-control-simulation-and-reality.svg)
  <figcaption>ROS Control Simulation and Reality</figcaption>
</figure>

The low-level base controller uses two PID controllers to compute PWM signals for each
motor based on the error between measured and target wheel velocities.
`RobotHW` receives measured joint states (angular position (rad) and angular velocity
(rad/s)) from which it updates its joint values. With these measured velocities and the
desired command velocity (`geometry_msgs/Twist` message on the `cmd_vel`
topic), from the Navigation Stack, the `diff_drive_controller` computes the
target angular velocities for both wheel joints using the mathematical equations of a
differential drive robot. This controller works with continuous wheel joints through a
`VelocityJointInterface` class. The computed target commands are then published
within the high-level hardware interface inside the robot's `RobotHW::write` method.
Additionally, the controller computes and publishes the odometry on the odom topic
(`nav_msgs/Odometry`) and the transform from `odom` to `base_footprint`.
Having explained the two components of the base controller, the low-level firmware is
implemented first. The high-level hardware interface follows the next section.

!!! note "TODO"
    TODO Details about implementation will follow (see code for now)