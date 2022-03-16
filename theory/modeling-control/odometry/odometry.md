# Odometry

As robots move in the world to reach an objective or avoid an obstacle,
it is important for them to know where they are.


Through odometry, robots can update their pose in time, as long
as they know where they started from.

Odometry comes from the Greek words ὁδός [odos] (route)
and μέτρον [metron] (measurement), which literally mean: "measurement of the route".

The odometry problem can be formulated as: 
given an initial pose $q$ of $t_0$ at some initial time,
find the pose at any future time $t_0 + \Delta t$.

Given:

$q(t_0) = q_{t_0} = q_0 = \begin{bmatrix}x_0 & y_0 & \theta_0 \end{bmatrix}^T$

Find:

$q_{t_0 + \Delta t}, \forall \Delta t > 0$


When $\Delta t$ is small enough to consider the angular speed of the wheels constant, 
the pose update can be approximated as a simple sum.

$q_{t} = \begin{bmatrix}x_t & y_t & \theta_t \end{bmatrix}^T$

$q_{t_{k+1}} = q_{t_k} + \dot{q}_{t_k}(t_{k+1} - t_k)$

The process can then be applied iteratively to find the pose at any time, 
and at each iteration using the previous estimate as an initial condition.


!!! todo "TODO"
    Add odometry update image in world frame


The essence of odometry is to use the measurements of the distance traveled
by each wheel in a certain time interval and use them to derive the linear and 
the angular displacement of the robot in time through a motion model.

!!! todo "TODO"
    Add odometry update image with $\Delta x$ and $\Delta y$ displacements


For example, if our robot starts at time 0 with pose (0, 0, 0), 
and drives straight for 1 second at the speed of 1 meter per second, 
the odometery will tell us that the final pose will be (1 meter, 0, 0).


Recall, the kinematics model, that maps the wheel velocities to the variation of the pose in time.

$$
q_{t_{k+1}} \approx q_{t_k} + {\color{orange}{\dot{q}_{t_k}}}(t_{k+1} - t_k)
$$

Kinematic model

{% raw %}
$$
{\color{orange}{\dot{q}_{t}}} = \begin{bmatrix}\dot{x}_t \\ \dot{y}_t \\ \dot{\theta}_t \end{bmatrix} = 
\frac{ \color{green}{R} }{2} \begin{bmatrix}cos(\theta_t) & 0 \\ sin(\theta_t) & 0 \\ 0 & 1\end{bmatrix}
\begin{bmatrix}1 & 1 \\ \frac{1}{ \color{green}{L} } & -\frac{1}{ \color{green}{L} } \end{bmatrix}
\begin{bmatrix} \color{red}{\dot{\phi}_{r,t}} \\ \color{red}{\dot{\phi}_{l,t}} \end{bmatrix}
$$
{% endraw %}

This model allows us to perform the pose update once we determine its parameters,
which are the wheel radii, which we assume identical, and the distance between the wheels, or the baseline.

**Parameters**

- ${\color{green}{R}}$: wheel radius
- $2 {\color{green}{L}}$: baseline (distance between wheels)

What's also needed is to measure the wheel angular velocities.

**Measurements**

- $\color{red}{\dot{\phi}_{l,t}}$: wheel angular speeds


## Wheel Encoders

To measure the wheel angular velocities, we can use wheel encoders.
Although there are various implementations, the operation principle of wheel encoders is simple.

The outer rim of the motor's rotor has some evenly spaced, fixed markers.
Optical encoders, for example, have a perforated disk where the empty space are the markers.

!!! todo "TODO"
    Add encoder wheel image robotc.net

- One pulse every ${\color{red}{\alpha}} = \frac{2\pi}{N_{tot}}$ radians

Every time any of these markers transitions through some reference position on the stator, 
the marker is sensed, and a signal is produced.

For example, in optical encoders, a light sensor will pick up the light from a source 
every time an empty space in the disk comes about.

Knowing how many markers there are in a whole circumference, 
we can derive how much each wheel rotated by just counting the pulses in each of the k-th time interval.

- Wheel rotation (in $\Delta t_k$): ${\color{orange}{\Delta \phi_k}} = N_k \cdot {\color{red}{\alpha}}$

By dividing the total rotation by delta t, we can then measure the average wheel angular speed in that time frame.

- Angular speed: $\dot{\phi}_{t_k} \approx \frac{ {\color{orange}{\Delta \phi_k}} }{\Delta t_k}$


Expanding the kinematics model expressions, we gain insight on the pose update process.

{% raw %}
$$
\begin{align}
&&{\color{orange}{\dot{q}_{t}}} &= \begin{bmatrix}\dot{x}_t \\ \dot{y}_t \\ \dot{\theta}_t \end{bmatrix} = 
\frac{ \color{green}{R} }{2} \begin{bmatrix}cos(\theta_t) & 0 \\ sin(\theta_t) & 0 \\ 0 & 1\end{bmatrix}
\begin{bmatrix}1 & 1 \\ \frac{1}{ \color{green}{L} } & -\frac{1}{ \color{green}{L} } \end{bmatrix}
\begin{bmatrix} \color{red}{\dot{\phi}_{r,t}} \\ \color{red}{\dot{\phi}_{l,t}} \end{bmatrix}
{\color{red}{\longleftarrow}}
{\color{red}{\dot{\phi}_{t_k}}} \approx \frac{ \Delta \phi_k }{\Delta t_k} \\
&&{\color{orange}{\downarrow}} \\
&& {\color{green}{q_{t_{k+1}}}} \approx q_{t_k} + &{\color{orange}{\dot{q}_{t_k}}}(t_{k+1} - t_k) \\
&&{\color{green}{\downarrow}} \\
&&{\color{green}{x_{t_{k+1}}}} \approx x_{t_k} + & \frac{R}{2 \cancel{\Delta t_k}} \left(\Delta \phi_{r,t} + \Delta \phi_{l,t} \right) \cos(\theta_{t_k}) \cancel{\Delta t_k} \\
&&{\color{green}{y_{t_{k+1}}}} \approx y_{t_k} + & \frac{R}{2} \left(\Delta \phi_{r,t} + \Delta \phi_{l,t} \right) \sin(\theta_{t_k}) \\
&&{\color{green}{\theta_{t_{k+1}}}} \approx \theta_{t_k} + & \frac{R}{2L} \left(\Delta \phi_{r,t} - \Delta \phi_{l,t} \right) \\
\end{align}
$$
{% endraw %}

Notice how the time intervals cancel out, so we don't need to actually compute the angular speed of each wheel, 
but just the total rotation.

The **first step** in solving the odometry is transforming the wheel rotations into traveled distances for each wheel.
We count the pulses from the encoders, derive the rotation of each wheel, 
and then multiply by the radius of each wheel.

{% raw %}
$$
\begin{align}
x_{t_{k+1}} &\approx x_{t_k} + \frac{ {\color{orange}{R}} }{2} \left({\color{orange}{\Delta\phi_{r,t}}} + {\color{orange}{\Delta\phi_{l,t}}} \right) \cos(\theta_{t_k}) \\
y_{t_{k+1}} &\approx y_{t_k} + \frac{ {\color{orange}{R}} }{2} \left({\color{orange}{\Delta\phi_{r,t}}} + {\color{orange}{\Delta\phi_{l,t}}} \right) \sin(\theta_{t_k}) \\
\theta_{t_{k+1}} &\approx \theta_{t_k} + \frac{ {\color{orange}{R}} }{2L} \left({\color{orange}{\Delta\phi_{r,t}}} - {\color{orange}{\Delta\phi_{l,t}}} \right) \\
\end{align}
$$
{% endraw %}

Wheel travelled distance: $\color{orange}{d_{l/r}} = R \cdot \Delta \phi_{r/l}$

!!! todo "TODO"
    Add robot image with $d_l$ $d_r$ travelled distance


The **second step** is to transform the wheel displacements into the linear and the angular displacements of the robot reference frame, 
as we have seen in the [modeling section](./modeling-differential-drive-robot.md).

{% raw %}
$$
\begin{align}
x_{t_{k+1}} &\approx x_{t_k} + {\color{orange}{ \frac{ d_{r,t_k} + d_{l,t_k} }{2} }} \cos(\theta_{t_k}) \\
y_{t_{k+1}} &\approx y_{t_k} + {\color{orange}{ \frac{ d_{r,t_k} + d_{l,t_k} }{2} }} \sin(\theta_{t_k}) \\
\theta_{t_{k+1}} &\approx \theta_{t_k} + {\color{red}{ \frac{ d_{r,t_k} - d_{l,t_k} }{2L} }} \\
\end{align}
$$
{% endraw %}

!!! todo "TODO"
    Add robot image with $d_l$ $d_r$ travelled distance and $\Delta \theta$


The **final step** is to represent the displacement in the world frame
and add the increments to the previous estimates.


{% raw %}
$$
\begin{align}
x_{t_{k+1}} &\approx x_{t_k} + {\color{orange}{ d_{A,t_k} \cos(\theta_{t_k}) }} \\
y_{t_{k+1}} &\approx y_{t_k} + {\color{orange}{ d_{A,t_k} \sin(\theta_{t_k}) }} \\
\theta_{t_{k+1}} &\approx \theta_{t_k} + \Delta \theta_{t_k} \\
\end{align}
$$
{% endraw %}


!!! todo "TODO"
    Add robot image showing increment update in world frame

## Summary of Odometry Equations


{% raw %}
$$
\begin{align}
x_{t_{k+1}} &\approx x_{t_k} + d_{A,t_k} \cos(\theta_{t_k}) \\
y_{t_{k+1}} &\approx y_{t_k} + d_{A,t_k} \sin(\theta_{t_k}) \\
\theta_{t_{k+1}} &\approx \theta_{t_k} + \Delta \theta_{t_k} \\
\end{align}
$$

$$
\begin{align}
d_{A,t_k} &= \frac{ d_{r,t_k} + d_{l,t_k} }{2} \\
\Delta \theta_{t_k} &= \frac{ d_{r,t_k} - d_{l,t_k} }{2L} \\
d_{r/l,t_k} &= 2\pi R \frac{N_k}{N_{tot}}
\end{align}
$$
{% endraw %}


## Challenges in Odometry

There are practical challenges in odometry.

### "Dead reconing"

The first practical challenge stems from using this dead reckoning approach, 
which is the official name of always adding an increment to a previous estimate in order to obtain a new one.

$$
\begin{align}
x_{t_{k+1}} &\approx x_{t_k} + \Delta x_{t_k} \\
y_{t_{k+1}} &\approx y_{t_k} +  \Delta y_{t_k} \\
\theta_{t_{k+1}} &\approx \theta_{t_k} + \Delta \theta_{t_k} \\
\end{align}
$$

!!! todo "TODO"
    Reuse odometry update image in world frame


While it might work well for short distances,
over time, errors like the discrete time approximation will accumulate, 
making the estimate drift from reality.


### Kinematic Model

Second, we're using a mathematical model, that of the kinematics of a differential drive robot,
to translate the actual measurements to the pose of the robot.

{% raw %}
$$
\dot{q}_{t} = 
\frac{R}{2} \begin{bmatrix}cos(\theta_t) & 0 \\ sin(\theta_t) & 0 \\ 0 & 1\end{bmatrix}
\begin{bmatrix}1 & 1 \\ \frac{1}{L} & -\frac{1}{L} \end{bmatrix}
\begin{bmatrix}\dot{\phi}_{r,t} \\ \dot{\phi}_{l,t} \end{bmatrix}
$$
{% endraw %}

You might recall that we previously said that all models are wrong, although some are useful.
This wisdom is ever more true when the assumptions of the model are not respected.

### Wheel Slip

In particular, we impose the condition of no slippage of the wheels.

!!! todo "TODO"
    Add wheel slip images/animation

When the wheels slip, it means that the motor will be spinning,
the encoders will be producing measurements, but the robot will not be moving the same distance as we are assuming.
This will induce errors in the odometry, and they will compound over time.


### Odometry Calibration

Finally, we need to use some actual numerical values for the parameters of the model:
the wheel radii - which, by the way, are assumed to be identical,
but will they really be - and the robot baseline.

{% raw %}
$$
\begin{align}
\Delta \theta_{t_k} &= \frac{ d_{r,t_k} - d_{l,t_k} }{2 {\color{red}{L}} } \\
d_{r/l,t_k} &= 2\pi {\color{red}{R}} \frac{N_k}{N_{tot}}
\end{align}
$$
{% endraw %}

Accurately measuring these parameters is very important.
Even small imperfections will induce systematic errors in the odometry that, again, will compound over time.

Note that although nominally identical, no two real-world physical robots
will ever be the same due to manufacturing, assembly, or handling
differences.

To find the values of the parameters of the model that best fit our robot,
we will need to perform an odometry calibration procedure.

## Summary

- Wheel encoders can be used to update the robot's pose in time:
  
    1. Measure the motor's angular displacements $\Delta \phi$ in $\Delta t$
    2. Use the kinematics mdoel to find the robot's $\Delta x$, $\Delta y$, $\Delta \theta$
    3. Update the pose by adding the calculated increments

- Subject to dfit in time due to accumulation of numerical, slipping/skidding and calibration impercision errors.

