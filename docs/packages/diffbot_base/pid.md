
# PID Controllers

DiffBot uses two PID controllers, one for each motor.
Depending on which software version you use the two PID controllers are located in
different locations in the source code. Either the two PIDs are in the

- hardware interface: high-level approach until version 0.0.2, or
- base controller firmware: low-level approach starting from version 1.0.0

The current approach (starting from version 1.0.0) operates the PIDs in the
low-level base controller firmware.

Regardless of the location (high or low-level), each PID is passed the error
between velocity measured by the encoders and the target velocity computed by
the `diff_drive_controller` for a specific wheel joint. The
`diff_drive_controller` doesn't have a PID controller integrated, and doesn't
take care if the wheels of the robot are actually turning. As mentioned above,
ROS Control expects that the commands sent by the controller are actually
applied on the real robot hardware and that the joint states are always up
to date. This means that the `diff_drive_controller` just uses the `twist_msg`
on the `cmd_vel` topic for example from the `rqt_robot_steering` and converts it
to a velocity command for the motors. It doesn't take the actual velocity of the
motors into account.

!!! note

    See [the code of `diff_drive_controller`](https://github.com/ros-controls/ros_controllers/blob/698f85b2c3467dfcc3ca5743d68deba03f3fcff2/diff_drive_controller/src/diff_drive_controller.cpp#L460) where the `joint_command_velocity` is calculated. 

For this reason a PID controller can help to avoid situations such as the following where the robot moves not straigth although it's commanded to do so:

<iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/chUPeWXtim4" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

!!! note

    The video runs version 0.0.2, operating the PIDs in the high-level hardware interface.

The PIDs used in the video inherits from the ROS Control
[`control_toolbox::Pid`](http://wiki.ros.org/control_toolbox) that provides
Dynamic Reconfigure out of the box to tune the proportional, integral and
derivative gains. The behaviour when using only the P, I and D gains is that the
output can overshoot and even change between positive and negative motor percent
values because of a P gain that is too high. To avoid this, a feed forward gain F
can help to reach the setpoint faster. In the high-level approach this feed forward gain is present as a dynamic reconfigure parameter, defined in the in the `pid.yml` configuration file in the `cfg` folder of this package.

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


!!! note

    In the videos above, motors numbered 01 and 03 were used coincidencely and I
    wasn't aware of the remarkable differences in voltage levels. Using the motors
    04 and 05 improved the driving behaviour significantly.

To deal with significant differences in the motors it would also help to tune
the two PIDs individually, which is not shown in the [video
above](https://youtu.be/fdn5Mu0Qhl8).


!!! note

    Make also sure that the motor driver outputs the same voltage level on both channels when the robot is commanded to move straight.
    The used Grove i2c motor driver was tested to do this.
    Another problem of not driving straight can be weight distribution or the orientation of the caster wheel.

A good test to check the accuracy is to fix two meters of adhesive tape on the
floor in a straight line. Then, place the robot on one end oriented in the
direction to the other end. Now command it to move straight along the line and
stop it when it reaches the end of the tape. Record the lateral displacement
from the tape. Measuring a value below 10 cm is considered precise for these
motors.

<iframe width="560" height="315" src="https://www.youtube.com/embed/37M-3k2FCsQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

The video shows the real DiffBot robot as well as running the
`gazebo_ros_control` plugin with `diff_drive_controller` from ROS Control. The
robot is commanded to drive straight on a 2 meter test track. Five runs show
different offsets after 2 meter, which can be cause by:

- Orientation of the start pose
- Inaccurate motors start to spin at different voltage levels
- Low encoder resolution (geared output resolution at wheel)
- Wheel slip or alignment of the left/right and caster wheel
- Weight distribution
- Manufacturing tolerances
- Deformable material of the wheels generating different surfaces of contact with the ground depending on their assembly

## Gain / Trim Model

In case the robot constantly drives towards one side (e.g., to the left), this
issue might be overcome by tuning each PID individually. An alternative solution
can be gain and trim parameters, as implemented in the [`write()` method of the
high-level hardware
interface](https://github.com/ros-mobile-robots/diffbot/blob/e7b6c8c5206235a338ef15e20084630793813a4f/diffbot_base/src/diffbot_hw_interface.cpp#L169).

```cpp
double motor_constant_right_inv = (gain_ + trim_) / motor_constant_;
double motor_constant_left_inv = (gain_ - trim_) / motor_constant_;

joint_velocity_commands_[0] = joint_velocity_commands_[0] * motor_constant_left_inv;
joint_velocity_commands_[1] = joint_velocity_commands_[1] * motor_constant_right_inv;
```

The idea is that the trim coefficient $t$ is a "quick and dirty" way to account for
various nuisances of the physical robot, including differences in motors,
wheels, and the asymmetry of the robot. It's simple and easily interpretable,
too.

The two PIDs work to ensure that the wheels spin at the desired speed, but not
that the robot drives the desired way though. There are other factors such as
slightly different wheel raddi $R$ which lead to different rotational velocities
$\dot{\phi}_{l/r}$ (translational velocity $v_{l/r} = \dot{\phi}_{l/r} \cdot R$) and therefore different traveled distances for each wheel (left and right) $d_{l/r}$:

$$
\begin{aligned}
d_{l/r} &= v_{l/r} \cdot \Delta t \\
d_{l/r} &= \frac{\dot{\phi}_{l/r}}{R} \cdot \Delta t
\end{aligned}
$$

These mechanical variations "can't be seen" by the encoders alone. For example
assume you want the robot to go straight, you would set the two desired
rotational wheel velocities $\dot{\phi}_{l/r}$'s to be equal - but still the
robot would go left if, e.g., $R_r > R_l$, because, for the same wheel rotation
(ticks), the right wheel has traveled a bigger distance.

So the question is: what is the control objective?

When the control objective is to follow a desired reference position (angular or
lateral), it is more meaningful to measure that as error driving the PID
controller. But nothing stops us from having several PID controllers in cascade
sequence. One for the, e.g., lateral position, that computes the desired $(v_0,
\omega)$ of the robot, and two more in parallel for the wheel velocities. Still,
there would be a trim coefficient to compensate for differences in traveled
distances of the two wheels.


$d_{l/r}$ are the distances traveled by the left and right wheels, respectively, in a $\Delta t$:

$$
d_{l/r} = R_{l/r} \cdot \Delta \phi_{l/r}
$$

where $\Delta \phi_{l/r}$ are the rotations of each wheel (more accurately, of each motor), and $R_{l/r}$ is the radius of each wheel. If the radii are exactly the same, then $R_l = R_r = R$ and the approach using only two PID controllers would work because controlling $\dot{\phi_{l/r}} \approx \frac{\Delta \phi_{l/r,k}}{\Delta t_k}$ is equivalent to controlling $\Delta \phi_{l/r,k}$, as the time interval we can assume known.

In practice, $R_l = R_r = R$ does not hold because of manufacturing tolerances, deformable material of the wheels generating different surfaces of contact with the ground depending on their assembly and robot mass distribution, etc.

The trim parameter $t$ we define as something that adjusts the relationship between the commands sent to the motors (imagine voltage $V$) and the angular speed of each wheel ($\dot{\phi}$):

$$
\begin{aligned}
V_l = k(1-t) \dot \phi_l \\
V_r = k(1+t) \dot \phi_r
\end{aligned}
$$


Note that it's the same value of $t$ for the two motors ($k$, just assume to be
a given constant). The way we tweak it is to find the value such that the robot
goes straight, that is, $d_l = d_r$ (over some distance), when we command it to go
straight, that is, we send the same voltages to the two wheels ($V_l = V_r$).

$$
\begin{aligned}
V_l &= V_r \\
k(1-t) \dot\phi_l &= k(1+t) \dot\phi_r
\end{aligned}
$$

From the two equations ($V_l$, $V_r$), by imposing the assumptions above and using the
definition of $d_{l/r}$, we can derive $t = \frac{R_r - R_l}{R_r + R_l}$, which basically shows how the trim is there to
(empirically) compensate for the difference in the wheel radii.

!!! info "source"

    The trim / gain approach was customized for [Duckiebots](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/wheel_calibration.html). The broader context is
    system identification: a good resource for further reading could be [Russ
    Tedrake's underactuated robotics
    course](http://underactuated.mit.edu/sysid.html).


After the robot can drive on the straight line without deviating from it too far the final error can be overcome with addtional sensors, such as IMUs, cameras and LiDARs, and then fuse those sensor data to follow a goal location more accurately by constantly adjusting wheel velocities. An even simpler example are IR sensors to follow the line and adjust the wheel velocities according to stay near the line. 

### PID Tuning

To tune the PID controller, the following video provides a helpful overview of
the process. It gives some insights which behavior each PID parameter influences
for a vehicle trying to follow a lane, thereby reducing the cross track error.
However, the same process can be followed to reduce the error $e(t)$ being the
difference between the desired (target) angular wheel velocity $\dot
\phi_{desired,l/r}$ and the one measured $\dot \phi_{measured,l/r}$ by the
encoders:

$$
e_{l/r} = \dot{\phi}_{desired,l/r} - \dot{\phi}_{measured,l/r}
$$

<iframe width="560" height="315" src="https://www.youtube.com/embed/4Y7zG48uHRo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

In case of oscillations (wheel spin direction keeps changing), might happen
because of a too high proportional gain $K_P$ or a too low derivative gain
$K_D$. Changing gains should be done one at a time because all are related by
influencing the overall calculated output command $u(t)$ via the following equation

$$
u(t) = K_\text{p} e(t) + K_\text{i} \int_0^t e(\tau) \,\mathrm{d}\tau + K_\text{d} \frac{\mathrm{d}e(t)}{\mathrm{d}t}
$$

One option to start the tuning process is by setting these values:

P = something small until the motors start to spin and no major oscillations are observable
I = 0
D = 0

To overcome oscillations increase the D gain first but usually it will be a value below P (might be one or more magnitudes lower). The derivative term tracks how fast the velocity changes over time. A higher D gain can dampen oscillations and therefore lead to more stable behavior. The D term best estimate of the future trend of the error $e(t)$.
If the derivative gain is too low then the system is called underdamped and will pull too quickly towards the desired velocity, which can easily result in overshooting the target value. On the other hand if the D gain is too high the system is known to be overdamped and it will take a long time to reach the desired velocity.
A properlly chosen D gain will allow to reach the desired velocity quickly with reduced overshoot and therefore a near zero error rate (known as critically damped).

Term I accounts for past values of the error and integrates them over time. This term should be adjust last and increased slightly to tune the behavior further.

In code the values to tune these PID gains are found in the [`base_controller_config.h`](https://github.com/ros-mobile-robots/diffbot/blob/e7b6c8c5206235a338ef15e20084630793813a4f/diffbot_base/scripts/base_controller/lib/config/diffbot_base_config.h#L18-L20):

```cpp
#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant
```

## Related Issues to PIDs

- https://github.com/ros-mobile-robots/diffbot/issues/13
- https://github.com/ros-mobile-robots/diffbot/issues/35
- [Duckietown EDX MOOC Forum question](https://courses.edx.org/courses/course-v1:ETHx+DT-01x+1T2021/discussion/forum/90b2fa2c07d84c70926270dd995f1d57d007f56a/threads/60874666e136290498b20825)