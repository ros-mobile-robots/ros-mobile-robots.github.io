
# PID Controllers

DiffBot uses two PID controllers, one for each motor.
Depending on which software version you use the two PID controllers are located in
different locations in the source code. Either the two PIDs are in the

- hardware interface: high-level approach until version 0.0.2, or
- base controller firmware: low-level approach starting from version 1.0.0

The current approach (starting from version 1.0.0) operates the PIDs in the
low-level base controller firmware.

!!! note "In the simulation"
    In Gazebo, there is no firmware: `gazebo_ros_control` controls each wheel's speed with its own PID, with each robot's gains in `diffbot_control/config/<model>/pid.yaml`. [Wheel Dynamics in Simulation](../../theory/modeling-control/wheel-dynamics-simulation.md#turning-a-wheel-speed-command-into-torque) explains that PID and why its gains depend on the wheel's inertia and the time step.

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

## The feedback loop of one wheel

A feedback loop measures the value that it controls and corrects the difference to the value that is wanted.
Here, the controlled value is a wheel's angular velocity, in rad/s.
Each wheel has its own loop. Both loops run in the firmware on the Teensy, 20 times per second (`UPDATE_RATE_CONTROL` in [`diffbot_base_config.h`]({{ diffbot_repo_url }}/diffbot_firmware/lib/config/diffbot_base_config.h)).
In each of these control steps, [`WheelControl::step()`]({{ diffbot_repo_url }}/diffbot_firmware/lib/wheel_control/wheel_control.h) calls two functions of [`BaseController`]({{ diffbot_repo_url }}/diffbot_firmware/lib/base_controller/base_controller.h):

1. `read()` measures both wheel speeds with the encoders.
2. `write()` computes a new motor command for each wheel with its PID and sends the command to the motor driver. After one second without a wheel command from the Raspberry Pi (`COMMAND_TIMEOUT_MS`), the firmware stops the motors and skips `write()` until the next command arrives.

The figure shows the loop of Remo's left wheel. The right wheel's loop is the same, with its own PID, motor and encoder.

--8<-- "includes/pid-feedback-loop.html"

Each signal has a unit and, where the firmware stores it, a name in the code:

| | Signal | Unit | In the firmware |
| --: | --- | --- | --- |
| 1 | Target wheel speed | rad/s | `wheel_cmd_velocity_left_` |
| 2 | Error | rad/s | `error` in `PID::compute()` |
| 3 | Motor command | a number from −255 to 255 | `motor_cmd_left_` |
| 4 | PWM voltage to the motor | percent of the time that the voltage is on | set by `setSpeed()` |
| 5 | Rotation of the motor and the wheel | rad | |
| 6 | Encoder ticks | ticks | `encoder_left_.read()` |
| 7 | Measured wheel speed | rad/s | `joint_state_left_.angular_velocity_` |

The blue values in the figure are one control step of Remo's left wheel.
In that run, Remo was commanded to drive at 0.2 m/s with its wheels off the ground ([bench record](https://github.com/ros-mobile-robots/diffbot/issues/169#issuecomment-6661807745)).
The firmware logged the target, the measured speed, the PID's three terms and the motor command of this step.
The PWM's on-time (4) and the ticks (6) are calculated from the logged values.

A control step measures first and commands second.
`read()` measures the wheel's speed over the time since the previous step.
`write()` then sets the motor command that acts until the next step.
So the measured 5.56 rad/s (7) is the result of the previous motor command, and the 96 (3) is the new one.
Follow the values around the loop:

1. **Target wheel speed.** On the Raspberry Pi, `diff_drive_controller` converts the commanded robot velocity into a target angular velocity for each wheel. With Remo's wheel radius of 0.033 m, 0.2 m/s is 0.2 / 0.033 = 6.06 rad/s. The hardware interface publishes both targets on the topic `wheel_cmd_velocities`. The firmware stores the left one in `wheel_cmd_velocity_left_`.

2. **Error.** [`PID::compute()`]({{ diffbot_repo_url }}/diffbot_firmware/lib/pid/pid.cpp) starts with the comparison. The error is the target minus the measured speed: 6.06 − 5.56 = 0.50 rad/s. The error is positive, so the wheel is too slow.

3. **Motor command.** The PID turns the error into the motor command $u$, a number from −255 to 255. The sign is the direction, and 255 is the largest amount that the motor driver takes. The PID adds three terms:

    $$
    u = K_P \cdot e + K_I \cdot \sum e + K_D \cdot (e - e_\text{previous})
    $$

    $e$ is the error, $\sum e$ is the sum of the errors of the control steps so far, and $e - e_\text{previous}$ is the change of the error since the previous step. The gains are $K_P = 0.6$, $K_I = 0.3$ and $K_D = 0.5$ (`K_P`, `K_I` and `K_D` in `diffbot_base_config.h`). In the example, the sum is 322.20 rad/s, and the error is the same as in the previous step:

    $$
    u = 0.6 \cdot 0.50 + 0.3 \cdot 322.20 + 0.5 \cdot 0.00 = 96.96
    $$

    The firmware keeps the whole number 96 as the motor command.

4. **PWM voltage to the motor.** [`AdafruitMotorController::setSpeed()`]({{ diffbot_repo_url }}/diffbot_firmware/lib/motor_controllers/adafruit_feather_wing/adafruit_feather_wing.cpp) sends the direction and the amount 96 to the motor driver, the Adafruit DC Motor + Stepper FeatherWing. The driver switches the motor's supply voltage on and off many times per second, which is called pulse-width modulation (PWM). For the amount 96, the voltage is on for 96 × 16 of the 4096 steps of each PWM period, or 37.5 % of the time ([`Adafruit_DCMotor::setSpeed()`](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library/blob/1.1.4/Adafruit_MotorShield.cpp#L254-L256)). The longer the voltage is on, the faster the motor turns.

5. **Rotation.** The motor turns the wheel through its gearbox.

6. **Encoder ticks.** The encoder in the motor has two Hall sensors. The Teensy counts the edges of their two signals as ticks, 542 for one turn of the wheel (`encoder_resolution` in [`base.yaml`]({{ diffbot_repo_url }}/diffbot_base/config/base.yaml)). The debug output doesn't show the ticks of a single step. With 50 ms between two steps, the measured speed corresponds to 24 ticks.

7. **Measured wheel speed.** [`Encoder::jointState()`]({{ diffbot_repo_url }}/diffbot_firmware/lib/encoder/encoder_diffbot.cpp) converts the ticks to an angle and divides the angle by the time since the previous step, which it measures. With 24 ticks in 50 ms: 24 · 2π / 542 = 0.278 rad, and 0.278 rad / 0.05 s = 5.56 rad/s. This value goes back to the comparison. `read()` also publishes it to the Raspberry Pi on the topic `measured_joint_states`.

??? example "The firmware's debug output of this control step"
    While the parameter `/diffbot/debug/base_controller` is true, the firmware logs the values of its latest control step five times per second:

    - `wheel_cmd_velocity_left_` is the target wheel speed (1), and `measured_ang_vel_left` is the measured wheel speed (7).
    - `motor_cmd_left_` is the motor command (3).
    - The three values after `(p, i, d)` are the error (2), the sum of the errors and the change of the error.
    - `ticks_left_` is the encoder's running count since its last reset, not the ticks of one control step (6).

    ```console
    Read:
    ticks_left_: 1389
    ticks_right_: 1442
    measured_ang_vel_left: 5.56
    measured_ang_vel_right: 5.56
    wheel_cmd_velocity_left_: 6.06
    wheel_cmd_velocity_right_: 6.06
    Write:
    motor_cmd_left_: 96
    motor_cmd_right_: 93
    pid_left_errors (p, i, d): 0.50 322.20 0.00
    pid_right_error (p, i, d): 0.50 309.91 0.23
    ```

The example shows two properties of this PID that matter for tuning:

- **The sum of the errors holds the motor command.** It contributes 96.66 of the 96.96. When a wheel has turned at its target speed for several control steps, the error and its change are zero, and so are the first and the third term. Only the sum then keeps the motor command at the value that the motor needs for this speed. When the target is zero and the wheel stands still, the firmware clears the sum.
- **The gains belong to the control rate.** The firmware adds the error to the sum once per control step, and it takes the change of the error from one step to the next. It doesn't multiply or divide by the 50 ms between two steps. With another `UPDATE_RATE_CONTROL`, the same $K_I$ and $K_D$ have another effect.

## Driving straight

`diff_drive_controller` doesn't use the measured wheel speeds to correct its wheel-speed commands.
For this reason a PID controller can help to avoid situations such as the following where the robot moves not straight although it's commanded to do so:

<iframe width="560" height="315" data-consent-src="https://www.youtube-nocookie.com/embed/chUPeWXtim4" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

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

<iframe width="560" height="315" data-consent-src="https://www.youtube-nocookie.com/embed/fdn5Mu0Qhl8" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

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

<iframe width="560" height="315" data-consent-src="https://www.youtube-nocookie.com/embed/37M-3k2FCsQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

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


After the robot can drive on the straight line without deviating from it too far the final error can be overcome with additional sensors, such as IMUs, cameras and LiDARs, and then fuse those sensor data to follow a goal location more accurately by constantly adjusting wheel velocities. An even simpler example are IR sensors to follow the line and adjust the wheel velocities according to stay near the line. 

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

<iframe width="560" height="315" data-consent-src="https://www.youtube-nocookie.com/embed/4Y7zG48uHRo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

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
A properly chosen D gain will allow to reach the desired velocity quickly with reduced overshoot and therefore a near zero error rate (known as critically damped).

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