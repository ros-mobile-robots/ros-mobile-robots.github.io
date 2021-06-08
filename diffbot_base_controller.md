## DiffBot Base Controller Package

This package contains the low level code for a microcontroller such as the Teensy 3.2 or 4.0.
It communicates mostly with the the diffbot's implementation of 
`hardware_interface::RobotHW` in the [`diffbot_base`](diffbot_base.md).

The subscriber, publisher and topics relations are as follows:

| Topic                      | `hardware_interface::RobotHW` |`base_controller` | Description     |
|:--------------------------:|:-----------------------------:|:----------------:|:---------------:|
| `wheel_command_velocities` | publish | subscribe  | target angular wheel joint velocities computed by the `diff_drive_controller`. |
| `encoder_ticks`            | subscribe | publish | tick count read from the encoders, which are connected to the microcontroller. |
| `reset`                    | publish | subscribe | Topic to publish and empty message to reset the encoder ticks to zero. |

- No dynamic reconfigure on rosserial supported. This is why we use custom messages for the low level PID controller.

## Custom Messages

- Encoders
- EncodersStamped
- PID
- WheelsCmd: Command message that contains the target angular wheel velocities $\dot{\phi}$.
- WheelsCmdStamped: Adds a `std_msgs/Header` to the `WheelsCmd` message. 
- AngularVelocities
- AngularVelocitiesStamped



!!! info
    rosserial is based on C and only uses raw pointers as arrays and
    those don't have any intrinsic `size` property that could be queried.
    When working with messages that contain arrays of undefined size and rosserial,
    it is required to reserve the apropriate amount of memory using [`malloc`](https://en.cppreference.com/w/c/memory/malloc).
    Additionally the *_length member of type int that is part of the generated message has to be set accordingly.

!!! quote
    Also be aware that the arrays in the messages are not defined as vector objects. Thus you have to predefine the array and then pass it as pointer to the message. To determine the end of the array each one has an auto-generated integer companion with the same name and the suffix _length (see also [rosserial/Overview/Limitations](http://wiki.ros.org/rosserial/Overview/Limitations#Arrays)). [^messages_overview] 


## Parameters

The `base_controller` relies on the following parameters stored on the paramter server,
which are configured in the `config/base.yaml` and also party in the `diffbot_control/config/diffbot_control.yaml`:

| Parameter | Description |
|:---------:|:-----------:|
| wheel_radius | Radius of both wheel (assumed to be the same) |
| wheel_separation | Baseline distance between the two wheels |
| encoder_resolution | Total number of tick counts per one full revolution of a wheel |
| gain               | Gain parameter of the gain trim model[^d] |



## PID Controller

Note the two PID controllers inside the hardware interface, where each PID is passed the error between velocity measured by the encoders 
and the target velocity computed by the `diff_drive_controller` for a specific wheel joint. 
The `diff_drive_controller` doesn't have a PID controller integrated, and doesn't take care if the wheels of the robot are actually turning.
As mentioned above, ROS Control expects that the commands sent by the controller are actually implemented on the real robot hardware and that the
joint states are always up to date. This means that the `diff_drive_controller` just uses the `twist_msg` on the `cmd_vel` topic for example from the `rqt_robot_steering` and converts it to a velocity command for the motors. It doesn't take the actual velocity of the motors into account. 
See [the code of `diff_drive_controller`](https://github.com/ros-controls/ros_controllers/blob/698f85b2c3467dfcc3ca5743d68deba03f3fcff2/diff_drive_controller/src/diff_drive_controller.cpp#L460) where the `joint_command_velocity` is calculated. 
{: .notice :}

This is why a PID controller is needed to avoid situations like the following where the robot moves not straigth although it is commanded to do so:

<iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/chUPeWXtim4" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

The PID used here inherits from the ROS Control [`control_toolbox::Pid`](http://wiki.ros.org/control_toolbox) that provides Dynamic Reconfigure out of the box to tune the proportional, integral and derivative gains. The behaviour when using only the P, I and D gains is that the output can overshoot and even change between positive and negative motor percent values because of a P gain that is too high. To avoid this, a feed forward gain can help to reach the setpoint faster.
To add this feed forward gain to the dynamic reconfigure parameters it is necessary to add a new parameter configuration file in this package inside a `cfg` folder. 

For more details on ROS dynamic reconfigure see [the official tutorials](http://wiki.ros.org/dynamic_reconfigure/Tutorials).

With the use of the PID controller the robot is able to drive straight:

<iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/fdn5Mu0Qhl8" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

In case of using inexpensive motors like the [DG01D-E](https://www.sparkfun.com/products/16413) of DiffBot,
you have to take inaccurate driving behaviour into account. The straight driving behaviour can be improved
with motors that start spinning at the same voltage levels. To find suitable motors do a voltage sweep test by slightly increasing the voltage and
note the voltage level where each motor starts to rotate. Such a test was done on DiffBot's motors. 

Using six [DG01D-E](https://www.sparkfun.com/products/16413) motors the following values were recorded (sorted by increasing voltage):

| Motor | Voltage (V) |
|:-----:|:-----------:|
| 01    |  2.5        |
| 02    |  2.8 - 3.0  | 
| 03    |  3.1        |
| 04    |  3.2        |
| 05    |  3.2        |
| 06    |  3.3        |


In the videos above, motors numbered 01 and 03 were used coincidencely and I wasn't aware of the remarkable differences in voltage levels.
Using the motors 04 and 05 improved the driving behaviour significantly. 
{: .notice } 

To deal with significant differences in the motors it would also help to tune the two PIDs individually,
which is not shown in the [video above](https://youtu.be/fdn5Mu0Qhl8).


Make also sure that the motor driver outputs the same voltage level on both channels when the robot is commanded to move straight.
The used Grove i2c motor driver was tested to do this.
Another problem of not driving straight can be weight distribution or the orientation of the caster wheel.
{: .notice }

A good test to check the accuracy is to fix two meters of adhesive tape on the floor in a straight line. 
Then, place the robot on one end oriented in the direction to the other end. Now command it to move straight along the line
and stop it when it reaches the end of the tape. Record the lateral displacement from the tape. 
Measuring a value below 10 cm is considered precise for these motors.

### Launch File

To run a single controller_manager, the one from the `diffbot_base` package defined inside `difbot_base.cpp` use the 
launch file from [`diffbot_base/launch/diffbot.launch`](https://github.com/fjp/diffbot/blob/master/ros/src/diffbot_base/launch/diffbot.launch):

```xml
<!-- https://github.com/ros-controls/ros_controllers/tree/kinetic-devel/diff_drive_controller/test -->
<launch>
    <!-- Load DiffBot model -->
    <param name="robot_description"
	   command="$(find xacro)/xacro '$(find diffbot_description)/urdf/diffbot.xacro'"/>

    <node name="diffbot_base" pkg="diffbot_base" type="diffbot_base"/>

    <!-- Load controller config to the parameter server -->
    <rosparam command="load" 
              file="$(find diffbot_control)/config/diffbot_control.yaml"/>

    <!-- Load base config to the parameter server -->
    <rosparam command="load" 
              file="$(find diffbot_base)/config/base.yaml"/>

    <!-- Load the controllers -->
    <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
        output="screen" ns="diffbot" args="joint_state_controller
                                            mobile_base_controller"/>
</launch>
```

This will load the DiffBot robot description onto the parameter server which is required for the hardware interface that gets created inside the next
node `diffbot_base`. It creates the hardware interface and instantiates a new controller manager in the `diffbot_base.cpp`.
Finally the `spawner` from the `controller_manager` package is used to initialize and start the controllers defined in the `diffbot_control/config/diffbot_control.yaml`. This step of the launch file is required to get the controllers initialized and started.
Another way would be to use `controller_manager::ControllerManager::loadControllers()` inside the `diffbot_base.cpp`.

Additionally the launch file loads additional parameters, stored in the `diffbot_base/config/base.yaml` on the parameter server.
These parameters are hardware related and used to tune the driving behaviour:

```
# Hardware related parameters
# will be loaded onto the parameter server
# See the diffbot.launch
diffbot:
  encoder_resolution: 542
  gain: 1.0
  trim: 0.0
  motor_constant: 27.0
  pwm_limit: 1.0
```


After launching this launch file on DiffBot's single board computer (e.g. Raspberry Pi or Jetson Nano) with 

```console
roslaunch diffbot_base diffbot.launch
```

the following parameters are stored on the parameter server:

```console
$ rosparam list
/diffbot/hardware_interface/joints
/diffbot/joint_state_controller/extra_joints
/diffbot/joint_state_controller/publish_rate
/diffbot/joint_state_controller/type
/diffbot/mobile_base_controller/base_frame_id
/diffbot/mobile_base_controller/left_wheel
/diffbot/mobile_base_controller/pose_covariance_diagonal
/diffbot/mobile_base_controller/publish_rate
/diffbot/mobile_base_controller/right_wheel
/diffbot/mobile_base_controller/twist_covariance_diagonal
/diffbot/mobile_base_controller/type
/diffbot/mobile_base_controller/wheel_radius
/diffbot/mobile_base_controller/wheel_separation
/robot_description
/rosdistro
/roslaunch/uris/host_tensorbook__46157
/roslaunch/uris/host_ubuntu__33729
/rosversion
/run_id
```

## Additional Requirements

Because the hardware interface subscribes to the encoders, that are connected to the Teensy MCU, and publishes to the motors via the motr driver node,
another launch will be required to run these additional nodes. See the `diffbot_bringup` package for this setup.

## Simulation


To have a simulation showing DiffBot, the second step is to use the [`diffbot_gazebo/launch/diffbot_base.launch`](https://github.com/fjp/diffbot/blob/master/ros/src/diffbot_gazebo/launch/diffbot_base.launch) on the work pc:

```console
$ roslaunch diffbot_gazebo diffbot_base.launch
```

This will launch the gazebo simulation, which can make use of the running controllers inside the controller manager too:

<figure>
    <a href="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/ros_control_gazebo.png"><img src="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/ros_control_gazebo.png"></a>
    <figcaption><a href="http://gazebosim.org/tutorials/?tut=ros_control" title="ROS Control with Gazebo">ROS Control with Gazebo</a> Overview.</figcaption>
</figure>


After launching the Gazebo simulation the controllers got uninitialized.
(It is assumed that the [`gazebo_ros_control`](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/kinetic-devel/gazebo_ros_control) plugin that gets launched). Because of this the controllers have to be initialized and started again. For this the [`diffbot_base/launch/controllers.launch`](https://github.com/fjp/diffbot/blob/master/ros/src/diffbot_base/launch/controllers.launch) should be used.
This launch file is just loading and starting all controllers again. Note that using the `spawner` from the `controller_manager` package, like in the `diffbot_base/launch/diffbot.launch` results in an error. (TODO this needs some more testing).


## References

[^d] [ETH Zürich Odometry Calibration: Gain Trim Model](https://ethz.ch/content/dam/ethz/special-interest/mavt/dynamic-systems-n-control/idsc-dam/Lectures/amod/Lecture_13/20191104%20-%20ETH%20-%2002%20-%20OdometryCalibration.pdf)

[^messages_overview] http://wiki.ros.org/rosserial/Overview/Messages
