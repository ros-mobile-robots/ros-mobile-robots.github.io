# IMU Simulation: From Measurement to Message

How one sample of the simulated IMU is made, from Gazebo's sensor to the message on `/diffbot/imu/data_raw`. This page belongs to [IMU: Simulation](index.md), which shows the finished message.

The sample passes three parts:

1. [Gazebo's IMU sensor](#the-gazebo-sensor-in-the-model), which the robot's model declares, [measures two vectors](#what-the-sensor-measures).
2. The robots' IMU plugin [decides whether to publish](#when-the-plugin-publishes), [adds noise and fills the covariances](#noise-and-covariance), and [sets the header](#the-header-and-the-stamp).
3. [The adapter](#the-adapter) marks the orientation unavailable.

## The Gazebo sensor in the model

Each robot's model declares the sensor in its macro `imu_sensor`, after the link and the joint that [Position and Axes](position-and-axes.md#the-link-and-the-joint-in-the-model) explains. DiffBot's macro is in [`common_sensors.xacro`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L204-L258) of `diffbot_description`, Remo's in [remo_description's file of the same name](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/include/common_sensors.xacro#L211-L268). The sensor stands in a block that only Gazebo reads. DiffBot's, [`common_sensors.xacro`, lines 242–257]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L242-L257):

```xml title="common_sensors.xacro: the Gazebo sensor and its plugin" linenums="242" hl_lines="2 5 10"
--8<-- "{{ imu_sim_src_raw }}/diffbot_description/urdf/include/common_sensors.xacro:242:257"
```

- **`<gazebo reference="imu_link">`** holds what only Gazebo reads. URDF tools ignore this block. `reference` attaches the sensor to `imu_link`, so the sensor measures at `imu_link`'s position and along its axes.
- **`type="imu"`** is Gazebo's IMU sensor. `update_rate` makes the sensor measure 50 times per second.
- **The plugin** turns the sensor's values into ROS messages. `filename` names its library, `libdiffbot_gazebo_ros_imu_sensor.so`, the robots' own copy of the IMU plugin that comes with ROS. [The Robots' Own IMU Plugin](plugin.md) explains why the robots have a copy, and how [Remo's model gets the library's name](plugin.md#the-plugins-library-in-remos-model).

The lines inside `<plugin>` are the plugin's parameters:

- `robotNamespace` and `topicName`, `/diffbot` and `imu/gazebo`, give the topic `/diffbot/imu/gazebo`.
- `frameName`, `imu_link`, becomes the message's `header.frame_id`.
- `updateRateHZ`, 50, is the plugin's own limit: at most 50 messages per second.
- `gaussianNoise`, 0.005, in the last highlighted line, is the standard deviation $\sigma$ of the [noise that the plugin adds](#noise-and-covariance).
- `xyzOffset` is read, but not used.
- `rpyOffset` and `initialOrientationAsReference` change only the orientation, which the adapter removes.

Where the sensor sits and how it's turned comes from the joint's origin, not from the offsets.

??? info "If you inspect the model that Gazebo makes from the URDF"

    Gazebo converts the URDF into its own format, SDF, before it simulates the robot. The converted model has no `imu_link`. Gazebo merges links that are joined by fixed joints into one link, and `imu_link` hangs on a fixed joint.

    The sensor is still there, at the same place. The sensor belongs to the link that remains, `base_footprint` for DiffBot, and the sensor's pose in that link is `imu_link`'s position and rotation. `gz sdf -p` prints the converted model when you [convert the model as on the Gazebo Package page](../../../diffbot_gazebo.md#troubleshooting).

    For ROS nothing changes. `imu_link` stays the IMU's frame, in the message's header and in the transforms.

## What the sensor measures

In every measurement, Gazebo's IMU sensor computes two vectors along `imu_link`'s axes: the angular velocity $\boldsymbol\omega$ and the specific force $\mathbf f$. Gazebo's lines, [`ImuSensor.cc` in Gazebo Classic 11.15.1, lines 344–376]({{ gazebo_src_url }}/gazebo/sensors/ImuSensor.cc#L344-L376), under the [Apache License 2.0]({{ gazebo_src_url }}/LICENSE):

```cpp title="ImuSensor.cc (Gazebo Classic 11.15.1): the two vectors" linenums="344" hl_lines="8 9 26 27 32 33"
--8<-- "{{ gazebo_src_raw }}/gazebo/sensors/ImuSensor.cc:344:376"
```

**The angular velocity**, lines 344–354. Gazebo takes the angular velocity of the link that carries the sensor, in the world's axes. The first highlighted lines rotate it into the sensor's axes, which are `imu_link`'s:

$$
\boldsymbol\omega = \begin{pmatrix} \omega_x \\ \omega_y \\ \omega_z \end{pmatrix}
$$

$\omega_x$ is the angular velocity about `imu_link`'s x axis in rad/s, $\omega_y$ about its y axis and $\omega_z$ about its z axis. [The angular velocity on the overview](index.md#the-angular-velocity) shows the three rotations in a figure.

**The linear acceleration**, lines 356–376. Gazebo computes it in two steps:

1. It takes the velocity of the point where the sensor sits, and divides the velocity's change since the last measurement by the time between the two measurements. The result is that point's acceleration $\mathbf a$, in the second highlighted lines.
2. It subtracts gravity $\mathbf g$, in the last highlighted lines.

Both steps rotate their vector into `imu_link`'s axes. The result is the **specific force**, in m/s²:

$$
\mathbf f = \mathbf a - \mathbf g =
\begin{pmatrix} a_x - g_x \\ a_y - g_y \\ a_z - g_z \end{pmatrix}
$$

At rest, $\mathbf a$ is zero. With `imu_link`'s z axis up, gravity is $\mathbf g = (0, 0, -9.8)$ m/s², so the sensor reads $\mathbf f = (0, 0, +9.8)$ m/s². [The linear acceleration on the overview](index.md#the-linear-acceleration) explains why an accelerometer reads this, with a figure of the robot at rest, speeding up and turning.

Gazebo's gravity is 9.8 m/s² in the robots' worlds. That is the default of SDF, Gazebo's format for worlds, as the [SDF specification of `<gravity>`](http://sdformat.org/spec?ver=1.6&elem=world#world_gravity) says.

## When the plugin publishes

`Load()` connects the plugin's `UpdateChild()` to Gazebo's world update event, in [lines 131–132 of `gazebo_ros_imu_sensor.cpp`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L131-L132). So Gazebo's physics thread, the part of Gazebo that computes the simulation's steps, calls `UpdateChild()` at the start of every step. The first lines of `UpdateChild()` decide whether a call publishes a message. [`gazebo_ros_imu_sensor.cpp`, lines 137–147]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L137-L147):

```cpp title="gazebo_ros_imu_sensor.cpp: the start of UpdateChild()" linenums="137" hl_lines="5 6"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp:137:147"
```

- **Two times.** `current_time` is the simulation time of the sensor's last measurement. `last_time` is the value that `current_time` had in the last call that got past the rate limit. The last line of `UpdateChild()`, [line 191]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L191), stores it.
- **The rate limit** is the second `if`. `update_rate` is the parameter `updateRateHZ`, 50. A call returns right away while less than one period, 1/50 s = 20 ms, has passed since `last_time`. So `UpdateChild()` goes on at most 50 times per second of simulation time.
- **A subscriber.** A call that gets past the rate limit fills and publishes a message only while the topic has a subscriber. In the simulation, the adapter always subscribes.
- **A clock that went back** is the first `if`, the highlighted lines. A reset of the simulation sets Gazebo's clock back to zero. Then the sensor's time is earlier than `last_time`, and the plugin sets `last_time` to the sensor's time. The rate limit counts from the new time, and the first message after a reset follows one period later. [Publishing again after a reset](plugin.md#publishing-again-after-a-reset) explains what would happen without these two lines.

## Noise and covariance

A call that publishes fills the IMU message with the sensor's values and adds noise. [`gazebo_ros_imu_sensor.cpp`, lines 160–178]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L160-L178):

```cpp title="gazebo_ros_imu_sensor.cpp: noise and covariance" linenums="160"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp:160:178"
```

**The noise.** The plugin adds a random number to each component of both vectors:

$$
\tilde{\boldsymbol\omega} = \boldsymbol\omega + \mathbf n_\omega, \qquad \tilde{\mathbf f} = \mathbf f + \mathbf n_f
$$

$\tilde{\boldsymbol\omega}$ and $\tilde{\mathbf f}$ are what the message carries. Each component $n$ of $\mathbf n_\omega$ and $\mathbf n_f$ is a new random number in every message, from a normal distribution with mean 0 and standard deviation $\sigma$:

$$
n \sim \mathcal N(0, \sigma^2), \qquad \sigma = 0.005
$$

- **$\sigma$ comes from the robot's model.** It's the plugin's parameter `gaussianNoise`, 0.005, in the [last highlighted line of the model's Gazebo block](#the-gazebo-sensor-in-the-model). The plugin reads the parameter into `gaussian_noise` when Gazebo loads the model, in [lines 264–266]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L264-L266).
- **`GuassianKernel(0, gaussian_noise)`** draws one random number with mean 0 and this standard deviation, in [lines 194–206]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L194-L206). The excerpt calls it once for each of the six values.
- **One setting serves both vectors.** $\sigma$ is 0.005 rad/s for the angular velocity and 0.005 m/s² for the specific force.
- **It's a simulation setting**, not the BNO055's measured noise. Gazebo's sensor adds no noise of its own, because the model gives it none.

**The covariance.** The plugin writes the variance $\sigma^2 = 2.5 \cdot 10^{-5}$ into the elements 0, 4 and 8 of each covariance, in (rad/s)² and (m/s²)². These three elements are the matrix's diagonal, as [how the message stores a covariance](index.md#how-the-message-stores-a-covariance) on the overview shows. Element 8 is the yaw rate's variance, which the EKF uses. The other elements stay 0. For the angular velocity:

$$
\Sigma_\omega = \begin{pmatrix} \sigma^2 & 0 & 0 \\ 0 & \sigma^2 & 0 \\ 0 & 0 & \sigma^2 \end{pmatrix} = \sigma^2 I_3
$$

$I_3$ is the 3 × 3 identity matrix. The specific force's covariance $\Sigma_f$ has the same numbers, in (m/s²)².

**Why the other elements are 0.** An element off the diagonal is the covariance of two axes' errors. The plugin draws a separate random number for each axis, so the noise on one axis says nothing about the noise on another. The two numbers are independent, and the covariance of two independent numbers is 0, as Wikipedia's article on [covariance](https://en.wikipedia.org/wiki/Covariance#Uncorrelatedness_and_independence) shows. On a real sensor, the errors of two axes can depend on each other. Then these elements aren't 0.

The plugin also fills the orientation: Gazebo's orientation of the sensor with noise, in the lines before the excerpt, and the orientation's covariance, in the excerpt's lines 170–172. The adapter removes both.

## The header and the stamp

The end of `UpdateChild()`, [`gazebo_ros_imu_sensor.cpp`, lines 180–188]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L180-L188):

```cpp title="gazebo_ros_imu_sensor.cpp: the end of UpdateChild()" linenums="180" hl_lines="7 8 9"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp:180:188"
```

- **The frame** is `body_name`, the parameter `frameName`: `imu_link`.
- **The stamp** is the time of the sensor's last measurement, in simulation time.
- **After `publish()`**, the plugin does nothing more, as the highlighted comment says. gazebo_plugins' original calls `ros::spinOnce()` at this place, in [line 150 of its source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L150). [Why the plugin doesn't spin](plugin.md#why-the-plugin-doesnt-spin) explains why the copy leaves this call out.

## The adapter

The plugin's message has an orientation, and the firmware's message has none. [`imu_raw_adapter.py`]({{ imu_sim_src_url }}/diffbot_gazebo/scripts/imu_raw_adapter.py) makes the simulated message match the firmware's. `diffbot_view.launch` starts the adapter in the `diffbot` namespace, in [line 39]({{ imu_sim_src_url }}/diffbot_gazebo/launch/diffbot_view.launch#L39). So the adapter's relative topic names become `/diffbot/imu/gazebo` and `/diffbot/imu/data_raw`. [`imu_raw_adapter.py`, lines 16–36]({{ imu_sim_src_url }}/diffbot_gazebo/scripts/imu_raw_adapter.py#L16-L36):

```python title="imu_raw_adapter.py" linenums="16" hl_lines="4 5 16 17 18 19 20"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/scripts/imu_raw_adapter.py:16:36"
```

- **How a field is marked unavailable.** A `sensor_msgs/Imu` message always has an orientation field. To say that a field holds no data, a publisher sets the first element of the field's covariance to −1. The [message definition's comments]({{ common_msgs_src_url }}/sensor_msgs/msg/Imu.msg#L10-L13) ask for that, and so does [REP 145](https://www.ros.org/reps/rep-0145.html#topics), the ROS convention for IMU drivers. A node that reads the message should check that element and ignore the field. `ORIENTATION_UNAVAILABLE` is such a covariance: −1, then eight zeros. [How the message stores a covariance](index.md#how-the-message-stores-a-covariance) shows the array next to the other cases.
- **A convention, not a rule to drop the orientation.** REP 145 doesn't ask an IMU driver to leave out the orientation. A driver whose sensor estimates an orientation can publish it on `imu/data`. The firmware's topic, `imu/data_raw`, is REP 145's name for the accelerometer's and the gyroscope's data alone.
- **The callback** sets the quaternion's four values to 0 and the orientation covariance to a copy of the constant. `list()` makes the copy, so that the messages don't share the constant's list. Then the callback publishes the same message object.
- **What stays:** the header with its stamp and frame, the angular velocity, the specific force and their covariances. The adapter doesn't stamp the message again. The [IMU sample validator](../robot_localization.md#the-imu-sample-validator) checks how old each sample is, and a new stamp would hide a delay.
