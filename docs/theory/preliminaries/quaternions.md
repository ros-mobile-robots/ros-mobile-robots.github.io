# Quaternions

ROS represents a rotation with a unit quaternion, four numbers in the order $x, y, z, w$. Remo's inertial measurement unit (IMU) is mounted a quarter-turn from the chassis axes. Its mounting shows how those four numbers tell us which way a sensor reading points.

The sections go from the four numbers to the robots' messages:

1. [A rotation as four numbers](#a-rotation-as-four-numbers): an axis and an angle, stored together.
2. [The length 1 and the rotation that does nothing](#the-length-1-and-the-rotation-that-does-nothing): what a unit quaternion is, and why four zeros are no rotation.
3. [Roll, pitch and yaw](#roll-pitch-and-yaw): three angles for the same rotation, and the heading of a robot on the floor.
4. [Remo's IMU, turned by −90°](#remos-imu-turned-by-90): the worked example, from the robot's URDF model to the transform that ROS publishes.
5. [How ROS stores a quaternion](#how-ros-stores-a-quaternion): the message's four fields, and where the robots' messages carry them.

## A rotation as four numbers

A rotation in space turns a body by an angle about an axis. Every rotation has such an axis and an angle ([Euler's rotation theorem](https://en.wikipedia.org/wiki/Euler%27s_rotation_theorem)). A **quaternion** $\mathbf q$ stores the axis and the angle together in four numbers. For a rotation by the angle $\alpha$ about the axis with the unit vector $\mathbf n = (n_x, n_y, n_z)$, the quaternion is:

$$
\mathbf q = \begin{pmatrix} q_x \\ q_y \\ q_z \\ q_w \end{pmatrix}
= \begin{pmatrix} n_x \sin(\alpha/2) \\ n_y \sin(\alpha/2) \\ n_z \sin(\alpha/2) \\ \cos(\alpha/2) \end{pmatrix}
$$

- **$q_x$, $q_y$ and $q_z$:** the axis, scaled with the sine of half the angle.
- **$q_w$:** the cosine of half the angle.
- **The direction of a positive angle:** by the right-hand rule, as ROS's convention [REP 103](https://www.ros.org/reps/rep-0103.html#chirality) asks. Point your right thumb along the axis. Your fingers then curl in the direction of a positive angle.

The four numbers are the parts of a quaternion, a number with one real and three imaginary parts, as a complex number has one real and one imaginary part. Wikipedia's [Quaternions and spatial rotation](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation) shows how the products of quaternions rotate vectors. This page uses a quaternion only as the four numbers of a rotation.

Three angles, the roll, the pitch and the yaw, can describe a rotation too, and [Roll, pitch and yaw](#roll-pitch-and-yaw) converts between the two forms. Three angles have special cases. At a pitch of ±90°, the roll and the yaw turn about the same axis, and several sets of angles describe the same rotation ([gimbal lock](https://en.wikipedia.org/wiki/Gimbal_lock)). A quaternion has no such special case. ROS's convention REP 103 prefers quaternions to the other forms of a rotation, for their compact form and because they have no such special cases ([REP 103: rotation representation](https://www.ros.org/reps/rep-0103.html#rotation-representation)). ROS messages and tf2, ROS's library for coordinate frames, store every rotation as a quaternion.

## The length 1 and the rotation that does nothing

The length of a rotation's quaternion is always 1. The axis $\mathbf n$ is a unit vector, so $n_x^2 + n_y^2 + n_z^2 = 1$, and $\sin^2 + \cos^2 = 1$ for every angle:

$$
\begin{aligned}
\lVert \mathbf q \rVert &= \sqrt{q_x^2 + q_y^2 + q_z^2 + q_w^2} \\
&= \sqrt{\left(n_x^2 + n_y^2 + n_z^2\right)\sin^2\frac{\alpha}{2} + \cos^2\frac{\alpha}{2}} \\
&= 1
\end{aligned}
$$

Four properties follow:

- **Only the length 1 is a rotation:** a quaternion with another length describes no rotation. A rotation's quaternion is also called a unit quaternion.
- **Three degrees of freedom:** four numbers with one condition leave three free numbers. So a rotation has three degrees of freedom, and the covariance of an orientation is a 3 × 3 matrix, as in [the IMU's message](noise-and-covariance.md#the-imus-message).
- **The identity:** the angle $\alpha$ = 0 gives $\mathbf q = (0, 0, 0, 1)$, the rotation that does nothing. DiffBot's URDF model mounts its IMU's frame `imu_link` without a rotation, with [`imu_rpy`]({{ ekf_src_url }}/diffbot_description/urdf/diffbot.urdf.xacro#L4-L10) = `0 0 0`, so that frame's quaternion is the identity. The mounting on DiffBot itself still needs to be measured, as the comment above the setting says.
- **Two quaternions for each rotation:** $\mathbf q$ and $-\mathbf q$ describe the same rotation. $-\mathbf q$ belongs to the angle $\alpha$ + 360° about the same axis.

**Four zeros:** $(0, 0, 0, 0)$ has the length 0, so the four zeros are no rotation at all. Yet the robots' IMU messages carry four zeros as their orientation. The firmware never sets the orientation, so its message keeps the four zeros that a new message starts with: the constructor of rosserial's [`geometry_msgs::Quaternion`](https://github.com/frankjoshua/rosserial_arduino_lib/blob/0.9.1/src/geometry_msgs/Quaternion.h#L24-L30) sets all four numbers to 0. In the simulation, the adapter sets the four numbers to 0 in its function [`callback()`]({{ ekf_src_url }}/diffbot_gazebo/scripts/imu_raw_adapter.py#L29-L36). A new `geometry_msgs/Quaternion` of ROS Noetic on the robot's computer starts as four zeros too. The −1 in `orientation_covariance[0]` says that the field holds no data, so a node that reads an orientation checks the −1 before it uses the four numbers. In ROS 2, the [definition of `geometry_msgs/Quaternion`](https://github.com/ros2/common_interfaces/blob/4.9.2/geometry_msgs/msg/Quaternion.msg) gives $q_w$ the default 1, so a new quaternion is the identity there.

## Roll, pitch and yaw

The roll, the pitch and the yaw are the angles of three rotations, about the x, the y and the z axis. The robots' URDF models and tf2 use the same order: first the roll about the x axis, then the pitch about the fixed y axis, then the yaw about the fixed z axis. The fixed axes are those of the frame that the rotation starts from, the parent frame. ROS's [URDF specification of a joint](http://wiki.ros.org/urdf/XML/joint) defines the `rpy` of a joint's origin in this way, and tf2's [`setRPY()`](https://github.com/ros/geometry2/blob/0.7.10/tf2/include/tf2/LinearMath/Quaternion.h#L91-L110) computes the quaternion for this order.

A rotation about one axis alone has a short quaternion. For the yaw $\psi$ alone, the rotation about the z axis, the axis is $\mathbf n = (0, 0, 1)$ and the angle $\alpha = \psi$:

$$
\mathbf q = \left(0,\ 0,\ \sin\frac{\psi}{2},\ \cos\frac{\psi}{2}\right)
$$

Back from a yaw-only quaternion, the yaw is $\psi = 2\,\operatorname{atan2}(q_z, q_w)$, up to a full turn of 360°. For any quaternion of the length 1, the yaw is $\psi = \operatorname{atan2}\big(2\,(q_w q_z + q_x q_y),\ 1 - 2\,(q_y^2 + q_z^2)\big)$, between −180° and +180°. A roll alone has only $q_x$ and $q_w$, and a pitch alone only $q_y$ and $q_w$.

**The heading on the floor:** a robot on a flat floor turns only about the z axis. Its heading $\theta$, the angle of [the robot's pose](../modeling-control/robot-representations.md#robot-pose), is the yaw $\psi$ of its orientation. So the orientation in the robots' odometry messages is a yaw-only quaternion. The drive controller turns its heading into a quaternion with [`tf::createQuaternionMsgFromYaw()`](https://github.com/ros-controls/ros_controllers/blob/0.22.0/diff_drive_controller/src/diff_drive_controller.cpp#L434-L435) in ros_controllers 0.22.0. The simulation test turns the quaternion back into the heading with `euler_from_quaternion()`, as [Connecting to the robot](../../development/simulation-test.md#3-connecting-to-the-robot) on the Simulation Test page shows.

??? info "All three angles at once"
    With the abbreviations $c_\text{r} = \cos(\text{roll}/2)$ and $s_\text{r} = \sin(\text{roll}/2)$, and the same for the pitch (p) and the yaw (y), tf2's `setRPY()` computes the quaternion as:

    $$
    \begin{aligned}
    q_x &= s_\text{r}\,c_\text{p}\,c_\text{y} - c_\text{r}\,s_\text{p}\,s_\text{y} \\
    q_y &= c_\text{r}\,s_\text{p}\,c_\text{y} + s_\text{r}\,c_\text{p}\,s_\text{y} \\
    q_z &= c_\text{r}\,c_\text{p}\,s_\text{y} - s_\text{r}\,s_\text{p}\,c_\text{y} \\
    q_w &= c_\text{r}\,c_\text{p}\,c_\text{y} + s_\text{r}\,s_\text{p}\,s_\text{y}
    \end{aligned}
    $$

    The result is the product of the three quaternions of the single angles, the yaw's times the pitch's times the roll's. With the roll and the pitch at 0, $c_\text{r} = c_\text{p} = 1$ and $s_\text{r} = s_\text{p} = 0$, and the formulas give the yaw-only quaternion above. In Python, the module `tf.transformations` of ROS Noetic converts in both directions with `quaternion_from_euler()` and `euler_from_quaternion()`, with the same order of the angles and the four numbers in the order $q_x$, $q_y$, $q_z$, $q_w$.

## Remo's IMU, turned by −90°

Remo's IMU board lies flat on the breadboard, turned so that the board's y axis points forward, its x axis to the right and its z axis up. A tilt test on Remo showed these axes, as [Bench checks](../../packages/imu/driver-bno055.md#bench-checks) on the BNO055 page describes. Remo's URDF model in remo_description 2.5.0 turns the IMU's frame `imu_link` by −90° about the z axis of `chassis_link`, the frame at the center of Remo's base plate. The excerpt shows the comment and the two arguments, [`imu_xyz`](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/remo.urdf.xacro#L26) for the position and [`imu_rpy`](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/remo.urdf.xacro#L27) for the orientation, from [`remo.urdf.xacro`, lines 12–27](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/remo.urdf.xacro#L12-L27):

```xml title="remo.urdf.xacro: the IMU's position and orientation" linenums="12" hl_lines="15 16"
--8<-- "https://raw.githubusercontent.com/ros-mobile-robots/remo_description/2.5.0/urdf/remo.urdf.xacro:12:27"
```

`imu_rpy` is the roll, the pitch and the yaw of the fixed joint from `chassis_link` to `imu_link`: a roll of 0, a pitch of 0 and a yaw $\psi = -\pi/2$ rad, −90°. Only the yaw isn't 0, so the yaw-only quaternion applies, with $\psi/2$ = −45°:

$$
\begin{aligned}
\mathbf q &= \big(0,\ 0,\ \sin(-45°),\ \cos(-45°)\big) \\
&= (0,\ 0,\ -0.7071,\ 0.7071)
\end{aligned}
$$

The length is 1, because $0.7071^2 + 0.7071^2 = 0.5 + 0.5 = 1$.

**What the rotation does:** a rotation by $\psi$ about the z axis has the rotation matrix $R$. Its columns are the axes of `imu_link`, written in the axes of `chassis_link`:

$$
\begin{aligned}
R &= \begin{pmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{pmatrix} \\
&= \begin{pmatrix} 0 & 1 & 0 \\ -1 & 0 & 0 \\ 0 & 0 & 1 \end{pmatrix}
\end{aligned}
$$

- **First column, the x axis of `imu_link`:** $(0, -1, 0)$, the negative y axis of `chassis_link`. `chassis_link`'s y axis points to the left ([REP 103](https://www.ros.org/reps/rep-0103.html#axis-orientation)), so the board's x axis points to the right.
- **Second column, the y axis of `imu_link`:** $(1, 0, 0)$, forward.
- **Third column, the z axis of `imu_link`:** $(0, 0, 1)$, up, as the z axis of `chassis_link`.

These are the axes that the tilt test found. $R$ also turns a reading of the IMU into `chassis_link`'s axes. The accelerometer's reading is the specific force $\mathbf f$ (m/s²), the acceleration minus gravity, which reads about +9.8 m/s² upward at rest, as the box at the top of [IMU: Bosch BNO055](../../packages/imu/driver-bno055.md) explains. When Remo speeds up forward by 0.5 m/s², in example values:

$$
\begin{aligned}
\mathbf f_\text{chassis} &= R\,\mathbf f_\text{imu} \\
&= \begin{pmatrix} 0 & 1 & 0 \\ -1 & 0 & 0 \\ 0 & 0 & 1 \end{pmatrix}
\begin{pmatrix} 0 \\ +0.5 \\ +9.8 \end{pmatrix} \\
&= \begin{pmatrix} +0.5 \\ 0 \\ +9.8 \end{pmatrix}\ \text{m/s}^2
\end{aligned}
$$

The forward acceleration appears in the board's $f_y$ and in the robot's $f_x$. The angular velocity turns the same way: $(\omega_x, \omega_y, \omega_z)$ of `imu_link` becomes $(\omega_y, -\omega_x, \omega_z)$ of `chassis_link`. The yaw rate $\omega_z$ stays the same, because both z axes point up. The EKF turns the IMU's measurement into the robot's frame in this way, as [The frames](../../packages/imu/robot_localization.md#the-frames) on the Localization page explains. The same rotation takes the angular velocity's covariance into chassis coordinates as $R\,\boldsymbol\Sigma\,R^\mathsf T$. Here $\boldsymbol\Sigma = \sigma^2 I_3$, so the [covariance sphere](noise-and-covariance.md#the-covariance-ellipsoid) has the same radius in both frames.

**In tf:** robot_state_publisher publishes the transform of each fixed joint of the robot's URDF model on the topic `/tf_static`, in its function [`publishFixedTransforms()`](https://github.com/ros/robot_state_publisher/blob/1.15.3/src/robot_state_publisher.cpp#L145-L168). The transform from `chassis_link` to `imu_link` carries the quaternion above as its rotation. `rosrun tf tf_echo chassis_link imu_link` prints the transform wherever robot_state_publisher runs with Remo's URDF model. For remo_description 2.5.0, in a ROS Noetic container on the PC, the output starts with these lines:

```console
At time 0.000
- Translation: [-0.040, 0.023, 0.013]
- Rotation: in Quaternion [0.000, 0.000, -0.707, 0.707]
            in RPY (radian) [0.000, 0.000, -1.571]
            in RPY (degree) [0.000, 0.000, -90.000]
```

The translation is the position from `imu_xyz` in m, rounded to millimeters. The quaternion is the one computed above, in the order $q_x$, $q_y$, $q_z$, $q_w$.

## How ROS stores a quaternion

ROS's message for a quaternion, [`geometry_msgs/Quaternion`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/geometry_msgs/msg/Quaternion.msg), has four numbers, with $q_w$ last. The excerpt shows its definition from ROS's common_msgs:

```text title="Quaternion.msg: the geometry_msgs/Quaternion message" linenums="1"
--8<-- "https://raw.githubusercontent.com/ros/common_msgs/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/geometry_msgs/msg/Quaternion.msg"
```

The fields `x`, `y`, `z` and `w` are $q_x$, $q_y$, $q_z$ and $q_w$. Some libraries take $q_w$ first: the constructor `Quaternion(w, x, y, z)` of the C++ library Eigen, for example ([Eigen's `Quaternion` class](https://libeigen.gitlab.io/eigen/docs-nightly/classEigen_1_1Quaternion.html)). Before you copy four numbers from one library to another, check the order that each one uses.

The robots' messages carry quaternions in three places:

- **The IMU's orientation:** `orientation` of [`sensor_msgs/Imu`](noise-and-covariance.md#the-imus-message) is four zeros with −1 in `orientation_covariance[0]`, so the field holds no data. The BNO055 can compute its own orientation as a quaternion ([BNO055 datasheet, section 3.6.5.5 on page 37](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf#page=37)). The chip's registers hold that quaternion with $q_w$ first. The firmware doesn't read that quaternion, because the EKF uses only the yaw rate.
- **The odometry's orientation:** `pose.pose.orientation` of `nav_msgs/Odometry` holds the heading $\theta$ as a yaw-only quaternion. The drive controller publishes this message on `/diffbot/mobile_base_controller/odom`, and the EKF on `/diffbot/odometry/filtered`, as [The two outputs](../../packages/imu/robot_localization.md#the-two-outputs) on the Localization page describes.
- **The transforms:** each transform on `/tf` and `/tf_static` has a rotation, for example the heading in the EKF's transform from `odom` to `base_footprint`, and Remo's −90° in the transform from `chassis_link` to `imu_link`.

## Further reading

- **Quaternions in ROS:** the ROS wiki's tutorial [quaternion basics](http://wiki.ros.org/tf2/Tutorials/Quaternions) shows how to create, invert and multiply quaternions with tf2.
- **Rotating a vector:** Wikipedia's [Quaternions and spatial rotation](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation) shows how a product of quaternions rotates a vector, which this page leaves out.
- **The covariance of an orientation:** [Noise and Covariance](noise-and-covariance.md) explains the matrix that says how uncertain a measurement is.
