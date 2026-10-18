{% set rc_sha = "c2348e85abf35cf33cf654a84d61db206abe9a73" %}
{% set rc_raw = "https://raw.githubusercontent.com/ros-controls/ros_controllers/" ~ rc_sha %}
{% set rc_url = "https://github.com/ros-controls/ros_controllers/blob/" ~ rc_sha %}
{% set nav_sha = "f44bb1fc2810399165115cc98b530fe4b9397c18" %}
{% set nav_raw = "https://raw.githubusercontent.com/ros-planning/navigation/" ~ nav_sha %}
{% set nav_url = "https://github.com/ros-planning/navigation/blob/" ~ nav_sha %}
# Robot Motion

Chapter 5 of *Probabilistic Robotics* by Thrun, Burgard and Fox, "Robot Motion", describes how the pose of a robot that drives on the ground changes when it moves. The pose is the position $x, y$ and the heading $\theta$, as on [Robot Representations](../modeling-control/robot-representations.md#robot-pose). The book has two motion models:

- The **velocity motion model** describes the motion with the velocities that the robot drives with.
- The **odometry motion model** describes the motion between two poses that the robot's odometry reports.

In the book, both models are probabilistic: they add noise that describes how uncertain the motion is, because a filter such as the [particle filter](nonparametric-filters.md) needs it. This page shows the core of each model without noise, and where DiffBot's ROS packages use it: diff_drive_controller computes the odometry with the velocity motion model, and amcl moves its particles with the odometry motion model. The code excerpts are from ROS Noetic's [ros_controllers]({{ rc_url }}) and [navigation]({{ nav_url }}) repositories, at the commits the links point to.

## Velocity motion model

A differential drive robot gets two velocities as its command: a linear velocity $v$ along its heading, and an angular velocity $\omega$, positive to the left. While $v$ and $\omega$ stay constant, the robot drives on a circular arc of radius $r = v/\omega$. After a time $\Delta t$, it ends at (section 5.3 of the book):

$$
\begin{aligned}
x' &= x - \frac{v}{\omega}\sin\theta + \frac{v}{\omega}\sin(\theta + \omega\,\Delta t) \\
y' &= y + \frac{v}{\omega}\cos\theta - \frac{v}{\omega}\cos(\theta + \omega\,\Delta t) \\
\theta' &= \theta + \omega\,\Delta t
\end{aligned}
$$

There are two special cases:

- **$\omega = 0$:** the robot drives straight, by $v\,\Delta t$ in the direction $\theta$. The equations above would divide by 0 here; in the limit they become $x' = x + v\,\Delta t\cos\theta$ and $y' = y + v\,\Delta t\sin\theta$.
- **$v = 0$:** the robot turns on the spot, by $\omega\,\Delta t$.

The [Simulation Test](../../development/simulation-test.md#the-checks) uses these two cases for its expected values: 0.2 m/s for 3 s is 0.6 m straight ahead, and 0.5 rad/s for 3 s is 1.5 rad on the spot.

The book adds noise to $v$ and $\omega$, and a small extra rotation at the end, so that a noisy end pose isn't confined to arcs.

### In diff_drive_controller

[diff_drive_controller](http://wiki.ros.org/diff_drive_controller) computes DiffBot's odometry (see [Simulation Test](../../development/simulation-test.md#what-runs-in-the-simulation) for where it runs). In each control cycle, `Odometry::update()` gets the wheel joint positions in rad, `left_pos` and `right_pos`. The first part of the method updates the pose, [`odometry.cpp`, lines 74–93]({{ rc_url }}/diff_drive_controller/src/odometry.cpp#L74-L93); the rest estimates the velocities:

```cpp title="odometry.cpp: the first part of Odometry::update()" linenums="74"
--8<-- "{{ rc_raw }}/diff_drive_controller/src/odometry.cpp:74:93"
```

- A wheel's angle times its radius $R$ is the distance it has rolled. The difference to the last cycle is the distance in this cycle, $d_l$ and $d_r$. Despite their names, `*_est_vel`, `linear` and `angular` are distances and an angle, not velocities.
- `linear` is the distance $\Delta s$ that the robot's center moved in this cycle, the mean of both wheels. `angular` is the angle $\Delta\theta$ it turned: the difference of the wheels' distances divided by the wheel separation $b$. These are the kinematics of a differential drive robot ([Odometry](../modeling-control/odometry.md#kinematic-model), where the wheel separation is $2L$):

$$
\Delta s = \frac{d_r + d_l}{2}, \qquad \Delta\theta = \frac{d_r - d_l}{b}
$$

`integrate_fun_` is `integrateExact()` by default, [lines 153–166]({{ rc_url }}/diff_drive_controller/src/odometry.cpp#L153-L166):

```cpp title="odometry.cpp: Odometry::integrateExact()" linenums="153"
--8<-- "{{ rc_raw }}/diff_drive_controller/src/odometry.cpp:153:166"
```

This is the velocity motion model for one control cycle. With $\Delta s = v\,\Delta t$ and $\Delta\theta = \omega\,\Delta t$, the quotient $v/\omega$ is the radius $r$, $\theta + \omega\,\Delta t$ is the new heading `heading_`, and the three lines are the equations above. When the robot drives almost straight, $r$ becomes huge, so the controller moves it straight instead, in the direction of the mean heading $\theta + \Delta\theta/2$ (`integrateRungeKutta2()`). The controller doesn't add noise. The [Odometry](../modeling-control/odometry.md) page derives a simpler update, a straight step in the direction $\theta$, which is close to the arc for short steps.

## Odometry motion model

Many robots report odometry, a pose that they compute from their wheel encoders, rather than the velocities they drove with. The odometry is usually more accurate than the commanded velocities, because it measures what the wheels did. The odometry motion model therefore uses two odometry poses as the robot's motion: the start pose $(x_0, y_0, \theta_0)$ and the end pose $(x_1, y_1, \theta_1)$. It describes the motion between them as a rotation $\delta_\text{rot1}$ towards the end position, a straight translation $\delta_\text{trans}$ to it, and a rotation $\delta_\text{rot2}$ to the end heading (section 5.4 of the book):

--8<-- "includes/odometry-motion-model.html"

$$
\begin{aligned}
\delta_\text{rot1} &= \operatorname{atan2}(y_1 - y_0,\; x_1 - x_0) - \theta_0 \\
\delta_\text{trans} &= \sqrt{(x_1 - x_0)^2 + (y_1 - y_0)^2} \\
\delta_\text{rot2} &= \theta_1 - \theta_0 - \delta_\text{rot1}
\end{aligned}
$$

- [atan2](https://en.wikipedia.org/wiki/Atan2)$(\Delta y, \Delta x)$ is the angle of the vector $(\Delta x, \Delta y)$ from the x axis, in $-\pi$ to $\pi$. Unlike $\arctan(\Delta y/\Delta x)$, it uses the signs of both values, so it tells forward from backward and works for $\Delta x = 0$. Subtracting $\theta_0$ makes $\delta_\text{rot1}$ relative to the direction the robot faced.
- $\delta_\text{trans}$ is the straight-line distance, the length of the hypotenuse of a right triangle with the sides $\Delta x$ and $\Delta y$.
- The angles are wrapped to $-\pi$ to $\pi$, because $\theta$ jumps from $\pi$ to $-\pi$ when the robot turns past 180°.

The three numbers don't say how the robot moved in between: it may have driven on an arc or turned on the spot, but it hardly ever turned, drove straight and turned again. They only describe where it ended up relative to where it started, and they are the same in every frame. So a filter can take them from the odometry frame and apply them to a pose in another frame, such as a particle's pose in the map.

### In amcl

[amcl](http://wiki.ros.org/amcl) localizes the robot in a map with a particle filter. DiffBot's [`amcl.launch`]({{ diffbot_repo_url }}/diffbot_navigation/launch/amcl.launch) sets `odom_model_type` to `diff-corrected`. When the odometry has changed enough, `AMCLOdom::UpdateAction()` moves every particle. Its `diff-corrected` case, [`amcl_odom.cpp`, lines 256–307]({{ nav_url }}/amcl/src/amcl/sensors/amcl_odom.cpp#L256-L307), gets the change of the odometry pose, `ndata->delta`, and the previous odometry pose, `old_pose`:

```cpp title="amcl_odom.cpp: the diff-corrected case of AMCLOdom::UpdateAction()" linenums="256"
--8<-- "{{ nav_raw }}/amcl/src/amcl/sensors/amcl_odom.cpp:256:307"
```

- **The three numbers:** `angle_diff()` subtracts two angles and wraps the result to $-\pi$ to $\pi$. When the robot turns on the spot, its position hardly changes, and the direction of such a short translation is noise. So below 1 cm, $\delta_\text{rot1}$ is 0, and $\delta_\text{rot2}$ is the whole turn.
- **Backward motion:** when the robot drives backward, $\delta_\text{rot1}$ is about $\pm\pi$, and $\delta_\text{rot2}$ about $\mp\pi$. The book's noise grows with the size of the rotations, so amcl measures them from 0 or $\pi$, whichever is closer, and treats backward like forward motion.
- **The noise:** for each particle, amcl draws the three numbers with Gaussian noise, scaled by the parameters `odom_alpha1` to `odom_alpha4`. `diff-corrected` differs from `diff` only here: it passes the standard deviation to `pf_ran_gaussian()`, as in the book, where `diff` passes the variance.
- **The particle update:** each particle moves by its noisy numbers, starting from its own heading.

## Where each model is used

| Where | Model | What for |
|:--|:--|:--|
| diff_drive_controller | Velocity motion model, without noise | Integrating the wheel motion of each control cycle into the odometry |
| amcl | Odometry motion model, with noise | Moving the particles by the change of the odometry |
| [Simulation Test](../../development/simulation-test.md#relative_pose) | Neither | Comparing two poses: it uses the end pose relative to the start pose, in the robot's frame at the start, which needs no special case for turning on the spot or driving backward |

## Resources

- *Probabilistic Robotics* by Sebastian Thrun, Wolfram Burgard and Dieter Fox, chapter 5, "Robot Motion" ([Amazon.com][amazon_book_probabilistic_robotics_us], [Amazon.de][amazon_book_probabilistic_robotics_ger], affiliate links)
- [`odometry.cpp`]({{ rc_url }}/diff_drive_controller/src/odometry.cpp) of diff_drive_controller and [`amcl_odom.cpp`]({{ nav_url }}/amcl/src/amcl/sensors/amcl_odom.cpp) of amcl (ROS Noetic)
