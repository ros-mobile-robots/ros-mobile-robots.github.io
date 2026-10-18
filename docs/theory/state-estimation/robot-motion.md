# Robot Motion

Chapter 5 of *Probabilistic Robotics* by Thrun, Burgard and Fox, "Robot Motion", describes how the pose of a robot that drives on the ground changes when it moves. The pose is the position x, y and the heading θ, as on [Robot Representations](../modeling-control/robot-representations.md#robot-pose). The book has two motion models:

- The **velocity motion model** describes the motion with the velocities that the robot drives with.
- The **odometry motion model** describes the motion between two poses that the robot's odometry reports.

In the book, both models are probabilistic: they add noise that describes how uncertain the motion is, because a filter such as the [particle filter](nonparametric-filters.md) needs it. This page shows the core of each model without noise, and where DiffBot's ROS packages use it: diff_drive_controller computes the odometry with the velocity motion model, and amcl moves its particles with the odometry motion model.

## Velocity motion model

A differential drive robot gets two velocities as its command: a linear velocity v along its heading, and an angular velocity ω, positive to the left. While v and ω stay constant, the robot drives on a circular arc of radius r = v / ω. After a time Δt, it ends at (section 5.3 of the book):

$$
\begin{aligned}
x' &= x - \frac{v}{\omega}\sin\theta + \frac{v}{\omega}\sin(\theta + \omega\,\Delta t) \\
y' &= y + \frac{v}{\omega}\cos\theta - \frac{v}{\omega}\cos(\theta + \omega\,\Delta t) \\
\theta' &= \theta + \omega\,\Delta t
\end{aligned}
$$

There are two special cases:

- **ω = 0:** the robot drives straight, by v Δt in the direction θ. The equations above would divide by 0 here; in the limit they become x' = x + v Δt cos θ and y' = y + v Δt sin θ.
- **v = 0:** the robot turns on the spot, by ω Δt.

The [Simulation Test](../../development/simulation-test.md) uses these two cases for its expected values: 0.2 m/s for 3 s is 0.6 m straight ahead, and 0.5 rad/s for 3 s is 1.5 rad on the spot.

The book adds noise to v and ω, and a small extra rotation at the end, so that a noisy end pose isn't confined to arcs.

### In diff_drive_controller

[diff_drive_controller](http://wiki.ros.org/diff_drive_controller) computes DiffBot's odometry (see [Simulation Test](../../development/simulation-test.md#2-sim_drivelaunch-starts-gazebo-and-the-controllers) for where it runs). In each control cycle, [`Odometry::update()`](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/diff_drive_controller/src/odometry.cpp) gets the wheel joint positions in rad, `left_pos` and `right_pos`:

```cpp
const double left_wheel_cur_pos  = left_pos  * left_wheel_radius_;
const double right_wheel_cur_pos = right_pos * right_wheel_radius_;

const double left_wheel_est_vel  = left_wheel_cur_pos  - left_wheel_old_pos_;
const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;

left_wheel_old_pos_  = left_wheel_cur_pos;
right_wheel_old_pos_ = right_wheel_cur_pos;

const double linear  = (right_wheel_est_vel + left_wheel_est_vel) * 0.5 ;
const double angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation_;

integrate_fun_(linear, angular);
```

- A wheel's angle times its radius R is the distance it has rolled. The difference to the last cycle is the distance in this cycle, d<sub>l</sub> and d<sub>r</sub>. Despite their names, `*_est_vel`, `linear` and `angular` are distances and an angle, not velocities.
- `linear` is the distance Δs that the robot's center moved in this cycle, the mean of both wheels. `angular` is the angle Δθ it turned: the difference of the wheels' distances divided by the wheel separation b. These are the kinematics of a differential drive robot ([Odometry](../modeling-control/odometry.md#kinematic-model), where the wheel separation is 2L):

$$
\Delta s = \frac{d_r + d_l}{2}, \qquad \Delta\theta = \frac{d_r - d_l}{b}
$$

`integrate_fun_` is `integrateExact()` by default:

```cpp
if (fabs(angular) < 1e-6)
  integrateRungeKutta2(linear, angular);
else
{
  const double heading_old = heading_;
  const double r = linear/angular;
  heading_ += angular;
  x_       +=  r * (sin(heading_) - sin(heading_old));
  y_       += -r * (cos(heading_) - cos(heading_old));
}
```

This is the velocity motion model for one control cycle. With Δs = v Δt and Δθ = ω Δt, v / ω is the radius r, θ + ω Δt is the new heading `heading_`, and the three lines are the equations above. When the robot drives almost straight, r becomes huge, so the controller moves it straight instead, in the direction of the mean heading θ + Δθ / 2 (`integrateRungeKutta2()`). The controller doesn't add noise. The [Odometry](../modeling-control/odometry.md) page derives a simpler update, a straight step in the direction θ, which is close to the arc for short steps.

## Odometry motion model

Many robots report odometry, a pose that they compute from their wheel encoders, rather than the velocities they drove with. The odometry is usually more accurate than the commanded velocities, because it measures what the wheels did. The odometry motion model therefore uses two odometry poses as the robot's motion: the start pose (x₀, y₀, θ₀) and the end pose (x₁, y₁, θ₁). It describes the motion between them as a rotation δ<sub>rot1</sub> towards the end position, a straight translation δ<sub>trans</sub> to it, and a rotation δ<sub>rot2</sub> to the end heading (section 5.4 of the book):

--8<-- "includes/odometry-motion-model.html"

$$
\begin{aligned}
\delta_\text{rot1} &= \operatorname{atan2}(y_1 - y_0,\; x_1 - x_0) - \theta_0 \\
\delta_\text{trans} &= \sqrt{(x_1 - x_0)^2 + (y_1 - y_0)^2} \\
\delta_\text{rot2} &= \theta_1 - \theta_0 - \delta_\text{rot1}
\end{aligned}
$$

- [atan2](https://en.wikipedia.org/wiki/Atan2)(Δy, Δx) is the angle of the vector (Δx, Δy) from the x axis, in −π to π. Unlike atan(Δy / Δx), it uses the signs of both values, so it tells forward from backward and works for Δx = 0. Subtracting θ₀ makes δ<sub>rot1</sub> relative to the direction the robot faced.
- δ<sub>trans</sub> is the straight-line distance, the length of the hypotenuse of a right triangle with the sides Δx and Δy.
- The angles are wrapped to −π to π, because θ jumps from π to −π when the robot turns past 180°.

The three numbers don't say how the robot moved in between: it may have driven on an arc or turned on the spot, but it hardly ever turned, drove straight and turned again. They only describe where it ended up relative to where it started, and they are the same in every frame. So a filter can take them from the odometry frame and apply them to a pose in another frame, such as a particle's pose in the map.

### In amcl

[amcl](http://wiki.ros.org/amcl) localizes the robot in a map with a particle filter. DiffBot's [`amcl.launch`]({{ diffbot_repo_url }}/diffbot_navigation/launch/amcl.launch) sets `odom_model_type` to `diff-corrected`. When the odometry has changed enough, [`AMCLOdom::UpdateAction()`](https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/src/amcl/sensors/amcl_odom.cpp) computes the three numbers from the change of the odometry pose, `ndata->delta`, and the previous odometry pose, `old_pose`:

```cpp
// Avoid computing a bearing from two poses that are extremely near each
// other (happens on in-place rotation).
if(sqrt(ndata->delta.v[1]*ndata->delta.v[1] + 
        ndata->delta.v[0]*ndata->delta.v[0]) < 0.01)
  delta_rot1 = 0.0;
else
  delta_rot1 = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
                          old_pose.v[2]);
delta_trans = sqrt(ndata->delta.v[0]*ndata->delta.v[0] +
                   ndata->delta.v[1]*ndata->delta.v[1]);
delta_rot2 = angle_diff(ndata->delta.v[2], delta_rot1);
```

- `angle_diff()` subtracts two angles and wraps the result to −π to π.
- When the robot turns on the spot, its position hardly changes, and the direction of such a short translation is noise. So below 1 cm, δ<sub>rot1</sub> is 0, and δ<sub>rot2</sub> is the whole turn.

Then amcl adds noise to the three numbers for each particle, scaled by the parameters `odom_alpha1` to `odom_alpha4`, and moves the particle by them, starting from the particle's own heading:

```cpp
sample->pose.v[0] += delta_trans_hat *
        cos(sample->pose.v[2] + delta_rot1_hat);
sample->pose.v[1] += delta_trans_hat *
        sin(sample->pose.v[2] + delta_rot1_hat);
sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
```

When the robot drives backward, δ<sub>rot1</sub> is about ±π, and δ<sub>rot2</sub> about ∓π. The book's noise grows with the size of the rotations, so amcl measures them from 0 or π, whichever is closer, and treats backward like forward motion. `diff-corrected` differs from `diff` only in the noise: it passes the standard deviation to the random number generator, as in the book, where `diff` passes the variance.

## Where each model is used

| Where | Model | What for |
|:--|:--|:--|
| diff_drive_controller | Velocity motion model, without noise | Integrating the wheel motion of each control cycle into the odometry |
| amcl | Odometry motion model, with noise | Moving the particles by the change of the odometry |
| [Simulation Test](../../development/simulation-test.md#7-relative_pose-compares-two-poses) | Neither | Comparing two poses: it uses the end pose relative to the start pose, in the robot's frame at the start, which needs no special case for turning on the spot or driving backward |

## Resources

- *Probabilistic Robotics* by Sebastian Thrun, Wolfram Burgard and Dieter Fox, chapter 5, "Robot Motion" ([Amazon.com][amazon_book_probabilistic_robotics_us], [Amazon.de][amazon_book_probabilistic_robotics_ger], affiliate links)
- [`odometry.cpp`](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/diff_drive_controller/src/odometry.cpp) of diff_drive_controller and [`amcl_odom.cpp`](https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/src/amcl/sensors/amcl_odom.cpp) of amcl (ROS Noetic)
