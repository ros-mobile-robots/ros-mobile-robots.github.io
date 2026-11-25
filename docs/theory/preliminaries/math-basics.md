# Math Basics: Noise, Covariance and Quaternions

The pages about the robots' IMU and their EKF use two kinds of math. The first kind describes how uncertain a measurement is: the variance, the standard deviation and the covariance. The second kind describes a rotation in space: the quaternion. Every IMU message that DiffBot and Remo publish carries both. This page explains them from the start, with the robots' own numbers as examples. The other pages link its sections where they first use a term, so that a reader who meets a term there can look the term up here.

The sections build on each other, from one measured value to a vector of values and then to a rotation:

1. [The noise of one value](#the-noise-of-one-value): the bias, the mean, the variance and the standard deviation, worked by hand for three readings of a gyroscope.
2. [The normal distribution](#the-normal-distribution): the shape of the noise that the simulated IMU adds and that the EKF assumes.
3. [The covariance of two values](#the-covariance-of-two-values): whether the errors of two values go up and down together, and why independent errors have the covariance 0.
4. [The covariance matrix and its ellipsoid](#the-covariance-matrix-and-its-ellipsoid): all variances and covariances of a vector in one matrix, and the shape that the matrix describes.
5. [How ROS messages store a covariance](#how-ros-messages-store-a-covariance): a matrix as an array of 9 or 36 numbers, and the arrays with a special meaning.
6. [What the EKF does with a variance](#what-the-ekf-does-with-a-variance): a larger variance gives a measurement less weight.
7. [Quaternions](#quaternions): four numbers for a rotation, with the −90° turn of Remo's IMU board as the worked example.

Each example says where its numbers come from: a setting in the robots' code, a measurement on Remo, or example values that make a formula easy to follow.

## The noise of one value

A gyroscope measures how fast the sensor turns, as an angular velocity in rad/s. The robots' EKF uses one of its three values, the yaw rate $\omega_z$, the angular velocity about the upward axis. A robot at rest doesn't turn, so a perfect gyroscope at rest would read $\omega_z$ = 0 rad/s every time. A real gyroscope doesn't. Two things move its readings away from the true value:

- **The bias:** an offset that stays about the same for a while. Every reading is a little too large, or every reading is a little too small.
- **The noise:** a scatter that changes from one reading to the next. The readings jump around their average.

A recording of $N$ readings $\omega_{z,1}, \omega_{z,2}, \dots, \omega_{z,N}$ at rest describes both with three numbers. The mean says where the readings lie. The variance and the standard deviation say how far they scatter. The example below has three readings, so that every step can be followed by hand. A real recording has thousands.

### The mean

The mean $\bar\omega_z$ is the sum of the readings, divided by their number $N$:

$$
\bar\omega_z = \frac{1}{N}\sum_{i=1}^{N} \omega_{z,i}
$$

The robots' IMU, the BNO055, reports the angular velocity in whole steps of $\Delta$ = 1/900 rad/s, about 0.00111 rad/s ([BNO055 datasheet, table 3-22 on page 35](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf#page=35)). Take three readings of a gyroscope at rest, one step apart: $\omega_{z,1}$ = 0, $\omega_{z,2} = \Delta$ and $\omega_{z,3} = 2\Delta$. Their mean is:

$$
\bar\omega_z = \frac{0 + \Delta + 2\Delta}{3} = \Delta = 0.00111\ \text{rad/s}
$$

At rest, the true yaw rate is 0 rad/s. So the mean of a recording at rest estimates the bias, here one step. A long recording estimates the bias better than three readings, because the noise of many readings averages out.

### The variance and the standard deviation

The noise has two numbers of its own. They describe all the readings that the sensor could give:

- **The variance $\sigma^2$:** the mean of the squared distances of the readings from their true mean. Its unit is the square of the readings' unit, here (rad/s)².
- **The standard deviation $\sigma$:** the square root of the variance. Its unit is the readings' own, here rad/s, so the standard deviation says directly how far a reading typically lies from the mean.

Nobody knows these two numbers exactly for a real sensor. A recording estimates them, as the sample variance $s^2$ and the sample standard deviation $s$. The sample variance adds up the squared deviations of the readings from their mean and divides the sum by $N - 1$:

$$
s^2 = \frac{1}{N-1}\sum_{i=1}^{N} \left(\omega_{z,i} - \bar\omega_z\right)^2
$$

The three readings deviate from their mean $\bar\omega_z = \Delta$ by $-\Delta$, 0 and $+\Delta$. Their sample variance and sample standard deviation are:

$$
\begin{aligned}
s^2 &= \frac{(-\Delta)^2 + 0^2 + \Delta^2}{3 - 1} = \Delta^2 \\
&= 1.235 \cdot 10^{-6}\ (\text{rad/s})^2 \\
s &= \sqrt{s^2} = \Delta = 0.00111\ \text{rad/s}
\end{aligned}
$$

The formula has three parts that need a reason:

- **The squares:** a deviation above the mean and one below would cancel in a plain sum. Their squares are both positive, so every deviation adds to the scatter. A large deviation adds much more than a small one.
- **$N - 1$ instead of $N$:** the deviations are measured from the readings' own mean $\bar\omega_z$, not from the true mean, which nobody knows. The readings' own mean lies closer to the readings than the true mean does, so the squared deviations come out a little too small. Dividing by $N - 1$ makes up for that ([Bessel's correction](https://en.wikipedia.org/wiki/Bessel%27s_correction)). With $N$, the example's variance would be two thirds of $\Delta^2$.
- **At least two readings:** one reading has no scatter, and its $N - 1$ is 0.

Variances have a property that standard deviations don't have. When a value has two independent errors with the variances $\sigma_1^2$ and $\sigma_2^2$, the variance of their sum is $\sigma_1^2 + \sigma_2^2$. The standard deviations don't add up this way. So filters such as the EKF compute with variances. [The covariance of two values](#the-covariance-of-two-values) says what independent means.

The robots' angular velocity comes with these numbers. The first is a setting of the simulation, the second a measurement on Remo, and the last two are settings that the robots' messages carry:

| Source | Standard deviation | Variance |
|:--|:--|:--|
| The noise that the simulated IMU adds to each axis, the plugin's parameter [`gaussianNoise`]({{ ekf_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L251) | $\sigma$ = 0.005 rad/s | $\sigma^2$ = 2.5 · 10⁻⁵ (rad/s)² |
| Remo's BNO055 at rest, the gyroscope's three axes in a recording of 180 s ([bench checks](../../packages/imu/driver-bno055.md#bench-checks)) | $s$ = 0.0005 to 0.0008 rad/s | $s^2$ = 2 to 6 · 10⁻⁷ (rad/s)² |
| The firmware's setting [`angular_velocity_variance`]({{ ekf_src_url }}/diffbot_firmware/lib/config/diffbot_base_config.h#L71) for each axis, a starting value | 0.02 rad/s | 4 · 10⁻⁴ (rad/s)² |
| The wheel odometry's yaw rate, a setting of the drive controller ([The two lists](../../packages/imu/robot_localization.md#the-two-lists)) | 0.17 rad/s | 0.03 (rad/s)² |

The firmware's starting value is about a thousand times the variance that Remo's gyroscope showed at rest. Driving adds vibration, which a recording at rest doesn't contain, so the starting value stays until a measurement while driving shows what the EKF should use.

## The normal distribution

The variance says how far the noise scatters, but not how often each value appears. A **distribution** says that. The noise that the simulated IMU adds follows the **normal distribution**, also called the Gaussian distribution, and the EKF assumes that every error does. For a value $x$ with the mean $\mu$ and the standard deviation $\sigma$, the normal distribution has this density $p(x)$, the bell curve:

$$
p(x) = \frac{1}{\sigma\sqrt{2\pi}}\,\exp\left(-\frac{(x - \mu)^2}{2\sigma^2}\right)
$$

The short form $x \sim \mathcal N(\mu, \sigma^2)$ says that $x$ is normally distributed with the mean $\mu$ and the variance $\sigma^2$. Three properties matter on the robots' pages:

- **The area:** the area under the curve between two values is the probability that a value falls between them. The whole area is 1.
- **Two numbers:** $\mu$ sets where the bell stands and $\sigma$ how wide the bell is. The mean and the variance describe a normal distribution completely.
- **The 68–95–99.7 rule:** 68.3 % of the values lie within one standard deviation of the mean, 95.4 % within two and 99.7 % within three ([68–95–99.7 rule](https://en.wikipedia.org/wiki/68%E2%80%9395%E2%80%9399.7_rule)).

**The simulated IMU:** for every message, the IMU plugin of the robots' simulation adds a new random number $n$ to each value, from $\mathcal N(0, \sigma^2)$ with $\sigma$ = 0.005 rad/s for the angular velocity. The plugin's function [`GuassianKernel()`]({{ ekf_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L194-L206), spelled so in the code, turns two evenly distributed random numbers into one normally distributed number with the [Box–Muller transform](https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform) and scales the result with $\sigma$. So about 68 % of the noise values lie between −0.005 and +0.005 rad/s, about 95 % between −0.010 and +0.010 rad/s, and only 0.3 % beyond ±0.015 rad/s.

**Why this distribution:** an error that is the sum of many small, independent parts tends to be normally distributed, whatever the distributions of the parts are ([central limit theorem](https://en.wikipedia.org/wiki/Central_limit_theorem)). The Kalman filter and the EKF assume normally distributed errors. Then a mean and a covariance describe each error completely, and the filter only has to carry these numbers from one step to the next. *Probabilistic Robotics* by Thrun, Burgard and Fox derives these filters in its chapter 3, "Gaussian Filters" ([Amazon.com][amazon_book_probabilistic_robotics_us], [Amazon.de][amazon_book_probabilistic_robotics_ger], affiliate links).

**A real sensor:** the BNO055 reports whole steps of 1/900 rad/s. At rest, the standard deviation of Remo's gyroscope is smaller than one step ([bench checks](../../packages/imu/driver-bno055.md#bench-checks)), so its readings fall on a few steps around their mean. A histogram of them has a few bars, not a smooth bell. For such a sensor, the normal distribution is a model that the EKF's formulas need, not an exact description of the readings.

## The covariance of two values

The gyroscope measures three angular velocities at once, $\omega_x$, $\omega_y$ and $\omega_z$. The noise of each has its own variance. One question remains: do the errors of two values go up and down together? The **covariance** answers that question.

For $N$ pairs of readings of two values $x$ and $y$, such as $\omega_x$ and $\omega_y$, the sample covariance $s_{xy}$ multiplies the two deviations of each pair and averages the products, again with $N - 1$:

$$
s_{xy} = \frac{1}{N-1}\sum_{i=1}^{N} (x_i - \bar x)(y_i - \bar y)
$$

The covariance $\sigma_{xy}$ of the distribution is the same average over all the values that the two could take. Its sign says how the errors go together:

- **Positive:** when $x$ lies above its mean, $y$ tends to lie above its mean too. Most products are positive.
- **Negative:** when $x$ lies above its mean, $y$ tends to lie below its mean. Most products are negative.
- **Zero:** the positive and the negative products cancel. The deviation of $x$ says nothing about the sign of the deviation of $y$.

With $y = x$, the formula becomes the one for $s^2$, so the covariance of a value with itself is its variance.

**An example of four readings:** the table gives the deviations of four pairs of readings $i$ from their means, in steps $\Delta$ of the gyroscope. Each sign of the deviation of $\omega_x$ meets both signs of the deviation of $\omega_y$ once:

| $i$ | $\omega_{x,i} - \bar\omega_x$ | $\omega_{y,i} - \bar\omega_y$ | Product |
|:--|:--|:--|:--|
| 1 | $-\Delta$ | $-\Delta$ | $+\Delta^2$ |
| 2 | $-\Delta$ | $+\Delta$ | $-\Delta^2$ |
| 3 | $+\Delta$ | $-\Delta$ | $-\Delta^2$ |
| 4 | $+\Delta$ | $+\Delta$ | $+\Delta^2$ |

The products add up to 0, so $s_{xy}$ = 0. If the deviations of $\omega_y$ were $-\Delta$, $-\Delta$, $+\Delta$ and $+\Delta$ instead, always with the sign of the deviation of $\omega_x$, all four products would be $+\Delta^2$. Then $s_{xy} = 4\Delta^2/3$, as large as each of the two variances: the two errors go up and down fully together.

**The correlation:** the covariance has the unit of $x$ times the unit of $y$, so its size is hard to judge. Divided by both standard deviations, the covariance becomes the correlation $\rho_{xy} = s_{xy} / (s_x\,s_y)$, a number between −1 and +1 without a unit. In the second example, $\rho_{xy}$ = 1.

### Why independent errors have the covariance 0

Two errors are **independent** when one of them says nothing about the other. Then, whatever the error of $x$ is, the error of $y$ averages 0. So the product $(x - \mu_x)(y - \mu_y)$ averages to the error of $x$ times 0, which is 0. Written with the expected value $\mathrm E[\,\cdot\,]$, the mean over all the values that the errors could take:

$$
\begin{aligned}
\sigma_{xy} &= \mathrm E\big[(x - \mu_x)(y - \mu_y)\big] \\
&= \mathrm E[x - \mu_x] \cdot \mathrm E[y - \mu_y] \\
&= 0 \cdot 0 = 0
\end{aligned}
$$

The middle step holds for independent values only: the expected value of a product of independent values is the product of their expected values.

**The simulated IMU:** the plugin calls [`GuassianKernel()`]({{ ekf_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L194-L206) once for each value of each message, so each axis gets a random number of its own. The noise on one axis says nothing about the noise on another, and the covariance of every two axes is 0. That is why the plugin's message has zeros off the diagonal of its covariance matrices, which [the next section](#the-covariance-matrix-and-its-ellipsoid) introduces.

Two limits apply:

- **A recording isn't the model:** for independent noise, the sample covariance of a recording lies near 0, not at exactly 0, because a recording has only $N$ readings. The 0 in a message is the model's value.
- **The covariance 0 doesn't always mean independent:** the covariance only measures whether two errors grow together along a straight line. Errors can depend on each other in other ways and still have the covariance 0. For two errors whose pair follows a normal distribution, the covariance 0 does mean independent ([Covariance: uncorrelatedness and independence](https://en.wikipedia.org/wiki/Covariance#Uncorrelatedness_and_independence)).

## The covariance matrix and its ellipsoid

### The 3 × 3 matrix

A vector of three values, such as the angular velocity $\boldsymbol\omega = (\omega_x, \omega_y, \omega_z)$, has three variances, one for each value, and three covariances, one for each pair. The **covariance matrix** $\boldsymbol\Sigma_{\boldsymbol\omega}$ holds all of them. The element in row $i$ and column $j$ is the covariance of the $i$-th and the $j$-th value:

$$
\boldsymbol\Sigma_{\boldsymbol\omega} = \begin{pmatrix}
\sigma^2_{\omega_x} & \sigma_{\omega_x \omega_y} & \sigma_{\omega_x \omega_z} \\
\sigma_{\omega_y \omega_x} & \sigma^2_{\omega_y} & \sigma_{\omega_y \omega_z} \\
\sigma_{\omega_z \omega_x} & \sigma_{\omega_z \omega_y} & \sigma^2_{\omega_z}
\end{pmatrix}
$$

- **On the diagonal:** the three variances.
- **Off the diagonal:** the covariances, each one twice. The matrix is symmetric, $\sigma_{\omega_x \omega_y} = \sigma_{\omega_y \omega_x}$, because the product of two deviations is the same in either order.
- **The units:** each element has the unit of its row's value times the unit of its column's value. All nine elements of $\boldsymbol\Sigma_{\boldsymbol\omega}$ are in (rad/s)². In the covariance matrix of the wheel odometry's velocities, which mixes m/s and rad/s, the element of the forward speed $v_x$ and the yaw rate $\omega_z$ is in m·rad/s².

In one formula, the matrix is the expected value of the deviation vector times its own transpose, $\boldsymbol\Sigma_{\boldsymbol\omega} = \mathrm E\big[(\boldsymbol\omega - \boldsymbol\mu)(\boldsymbol\omega - \boldsymbol\mu)^\mathsf T\big]$, with the mean $\boldsymbol\mu$. A column of three deviations times a row of the same three deviations gives all nine products at once.

The robots' IMU messages carry diagonal matrices, with the same variance on every axis and zeros everywhere else. For the simulated IMU's angular velocity, with $\sigma^2 = 2.5 \cdot 10^{-5}$ (rad/s)², the matrix is:

$$
\boldsymbol\Sigma_{\boldsymbol\omega} = \begin{pmatrix} \sigma^2 & 0 & 0 \\ 0 & \sigma^2 & 0 \\ 0 & 0 & \sigma^2 \end{pmatrix} = \sigma^2 I_3
$$

$I_3$ is the 3 × 3 identity matrix. The firmware's matrix has the same form with 4 · 10⁻⁴ (rad/s)², as [What the firmware publishes](../../packages/imu/driver-bno055.md#what-the-firmware-publishes) lists.

### The covariance ellipsoid

Nine numbers are hard to picture, and a shape helps. Take the deviation $\mathbf e$ of a vector from its mean, with the three components $e_1$, $e_2$ and $e_3$. For a diagonal matrix with the standard deviations $\sigma_1$, $\sigma_2$ and $\sigma_3$, count each component in units of its own standard deviation. The deviations that lie exactly one standard deviation away fulfill this equation:

$$
\frac{e_1^2}{\sigma_1^2} + \frac{e_2^2}{\sigma_2^2} + \frac{e_3^2}{\sigma_3^2} = 1
$$

These points form an **ellipsoid** around the mean. Its semi-axes, the half widths along the three axes, are $\sigma_1$, $\sigma_2$ and $\sigma_3$:

- **The same variance on every axis:** the ellipsoid is a sphere with the radius $\sigma$. The noise prefers no direction. The simulated IMU's angular velocity has a sphere with the radius 0.005 rad/s, and the firmware's covariance a sphere with the radius 0.02 rad/s.
- **Different variances:** the ellipsoid is stretched along the axis with the largest standard deviation.
- **Covariances off the diagonal:** the ellipsoid turns. Its axes point along the matrix's **eigenvectors**, the directions that the matrix only stretches. Its semi-axes are the square roots of the matrix's **eigenvalues**, the stretch factors. The general equation of the ellipsoid is $\mathbf e^\mathsf T \boldsymbol\Sigma^{-1} \mathbf e = 1$, which gives the equation above for a diagonal matrix.

**A turned ellipse:** two values give an ellipse instead of an ellipsoid. Take a position estimate $(x, y)$ with this covariance matrix, in example values:

$$
\boldsymbol\Sigma = \begin{pmatrix} 5 & 3 \\ 3 & 5 \end{pmatrix}\ \text{cm}^2
$$

Without the covariance of 3 cm², the ellipse would be a circle with the radius $\sqrt{5}$ cm = 2.24 cm. With the covariance, the matrix has the eigenvalue 8 cm² along the direction (1, 1), at 45° between the x and the y axis, and the eigenvalue 2 cm² along (1, −1), at a right angle to the first. So the ellipse has the semi-axis $\sqrt{8}$ cm = 2.83 cm along the 45° line and $\sqrt{2}$ cm = 1.41 cm across. The positive covariance turns the ellipse toward the direction in which $x$ and $y$ are both larger or both smaller. Such a covariance appears, for example, when a robot drives at 45° to the x axis and is unsure how far the robot drove. An error in the distance then moves the estimate along the robot's path, in $x$ and $y$ together.

**The share inside:** for normally distributed values, the density is the same everywhere on the ellipsoid, and the ellipsoids of one, two and three standard deviations lie inside each other. The share of the values inside an ellipsoid depends on how many values the vector has. One value has an interval, two values an ellipse and three values an ellipsoid:

| Inside | 1 value | 2 values | 3 values |
|:--|:--|:--|:--|
| 1 standard deviation | 68.3 % | 39.3 % | 19.9 % |
| 2 standard deviations | 95.4 % | 86.5 % | 73.9 % |
| 3 standard deviations | 99.7 % | 98.9 % | 97.1 % |

The shares follow from the [chi-squared distribution](https://en.wikipedia.org/wiki/Chi-squared_distribution) with one, two and three degrees of freedom. So the ellipsoid of one standard deviation holds only a fifth of the values of three normally distributed values.

**In RViz:** RViz's Odometry display draws the covariance of each pose, as [Comparing the two odometries](../../packages/imu/robot_localization.md#comparing-the-two-odometries) on the Localization page sets the display up. RViz 1.14.26 draws the position's covariance as this ellipsoid, with semi-axes of one standard deviation along the eigenvectors, in its function [`computeShapeScaleAndOrientation3D()`](https://github.com/ros-visualization/rviz/blob/1.14.26/src/rviz/default_plugin/covariance_visual.cpp#L100-L137). The display's setting [Scale](https://github.com/ros-visualization/rviz/blob/1.14.26/src/rviz/default_plugin/covariance_property.cpp#L68-L71) multiplies the semi-axes, and its default 1 shows one standard deviation. When the pose's variances of the height $z$, the roll and the pitch are 0 or less, RViz draws the ellipse of $x$ and $y$ instead ([`pose_2d_`](https://github.com/ros-visualization/rviz/blob/1.14.26/src/rviz/default_plugin/covariance_visual.cpp#L338)).

## How ROS messages store a covariance

A ROS message has no type for a matrix. A message stores a covariance matrix as an array of numbers, row by row: first the elements of the first row, then those of the second, and so on. The message definitions call this order **row major**. With $n$ rows and $n$ columns, counted from 0 as in C++ and Python, the element in row $i$ and column $j$ has the index $k$ in the array:

$$
k = n\,i + j
$$

The robots' messages have arrays of two sizes:

- **A 3 × 3 matrix, `float64[9]`:** $n$ = 3. The variances, $i = j$, lie at the indices 0, 4 and 8.
- **A 6 × 6 matrix, `float64[36]`:** $n$ = 6. The variances lie at the indices 0, 7, 14, 21, 28 and 35.

The index of each element of a 3 × 3 matrix, at the element's place, with the variances in bold:

$$
\begin{pmatrix}
\mathbf 0 & 1 & 2 \\
3 & \mathbf 4 & 5 \\
6 & 7 & \mathbf 8
\end{pmatrix}
$$

### The IMU's message

The excerpt shows the definition of [`sensor_msgs/Imu`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/sensor_msgs/msg/Imu.msg), ROS's message for IMU data, from ROS's common_msgs. The highlighted lines are the comments about the covariances and the three arrays:

```text title="Imu.msg: the sensor_msgs/Imu message" linenums="1" hl_lines="5 6 7 8 10 11 12 13 18 21 24"
--8<-- "https://raw.githubusercontent.com/ros/common_msgs/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/sensor_msgs/msg/Imu.msg"
```

- **Three matrices:** each of the message's three measurements has its own 3 × 3 matrix, in lines 18, 21 and 24: [`orientation_covariance`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/sensor_msgs/msg/Imu.msg#L18), [`angular_velocity_covariance`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/sensor_msgs/msg/Imu.msg#L21) and [`linear_acceleration_covariance`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/sensor_msgs/msg/Imu.msg#L24). `angular_velocity_covariance[8]`, for example, is the variance of the yaw rate $\sigma^2_{\omega_z}$, the one element of the message that the robots' EKF uses.
- **The orientation's matrix is 3 × 3:** a quaternion has four numbers, but the rows and the columns of `orientation_covariance` stand for the rotation's errors about the x, y and z axes, as the comment in line 18 says: "about x, y, z axes". A rotation has three degrees of freedom, as [The length 1 and the rotation that does nothing](#the-length-1-and-the-rotation-that-does-nothing) explains.

The comments in lines 5–13 give three kinds of arrays a meaning. Here they are for the angular velocity:

- **The variances are known:** $[\,\sigma^2_{\omega_x}, 0, 0, 0, \sigma^2_{\omega_y}, 0, 0, 0, \sigma^2_{\omega_z}\,]$. A publisher that knows only the variances, for example from a datasheet, puts them on the diagonal (lines 5–6).
- **The covariance is unknown:** all nine elements are 0. A node that reads the message has to assume a covariance or get one from somewhere else (lines 7–8). The zeros don't say that the value is exact.
- **The field holds no estimate:** element 0 is −1. A node that reads the message ignores the field (lines 10–13). The comments don't define the other eight elements. In the robots' messages, they are 0. robot_localization follows the rule: its [`imuCallback()`](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ros_filter.cpp#L550-L555) skips the angular velocity of a message whose `angular_velocity_covariance[0]` is −1.

The robots' IMU messages use the first and the third kind:

- **The firmware on the robot:** the firmware writes its setting [`angular_velocity_variance`]({{ ekf_src_url }}/diffbot_firmware/lib/config/diffbot_base_config.h#L71), 4 · 10⁻⁴ (rad/s)², to the indices 0, 4 and 8 of `angular_velocity_covariance`, its setting for the acceleration, 10⁻² (m/s²)², to the same indices of `linear_acceleration_covariance`, and −1 to `orientation_covariance[0]`. [What the firmware publishes](../../packages/imu/driver-bno055.md#what-the-firmware-publishes) lists the fields.
- **The simulation:** the IMU plugin writes [`gn2`]({{ ekf_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L169), the square of its noise $\sigma$, 2.5 · 10⁻⁵, to the indices 0, 4 and 8 of each matrix, in (rad/s)² for the angular velocity and in (m/s²)² for the acceleration. The adapter [`imu_raw_adapter.py`]({{ ekf_src_url }}/diffbot_gazebo/scripts/imu_raw_adapter.py) then replaces the orientation's matrix with its constant [`ORIENTATION_UNAVAILABLE`]({{ ekf_src_url }}/diffbot_gazebo/scripts/imu_raw_adapter.py#L20), −1 and eight zeros, so that the simulated message has the firmware's form.

The three meanings belong to `sensor_msgs/Imu`. The covariances of `nav_msgs/Odometry` don't define them.

### The odometry's message

A [`nav_msgs/Odometry`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/nav_msgs/msg/Odometry.msg) message, which the drive controller and the EKF publish, has two 6 × 6 matrices: [`pose.covariance`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/geometry_msgs/msg/PoseWithCovariance.msg#L9) for the pose and [`twist.covariance`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/geometry_msgs/msg/TwistWithCovariance.msg#L9) for the velocities. The excerpt shows the definition of the pose's type, [`geometry_msgs/PoseWithCovariance`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/geometry_msgs/msg/PoseWithCovariance.msg), whose comment names the order of the six values:

```text title="PoseWithCovariance.msg: a pose with its covariance" linenums="1" hl_lines="5 6 7 8 9"
--8<-- "https://raw.githubusercontent.com/ros/common_msgs/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/geometry_msgs/msg/PoseWithCovariance.msg"
```

- **The pose's six values:** the position $x$, $y$, $z$ in m, then the rotations about the X, Y and Z axes in rad. Their variances lie at the indices 0, 7, 14, 21, 28 and 35. The heading $\theta$ is the rotation about Z, so `pose.covariance[35]` is the variance of the heading.
- **The orientation's errors:** the pose stores its orientation as a quaternion, but its covariance counts the orientation's errors as rotations about fixed axes, as line 6 says. A quaternion's four numbers don't appear in the matrix.
- **The velocities:** [`geometry_msgs/TwistWithCovariance`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/geometry_msgs/msg/TwistWithCovariance.msg) has the same comment and the same 36 numbers, for the velocities $v_x$, $v_y$, $v_z$, $\omega_x$, $\omega_y$ and $\omega_z$. `twist.covariance[35]` is the variance of the yaw rate, 0.03 (rad/s)² in the drive controller's message.

[The variances](../../packages/imu/robot_localization.md#the-variances) on the Localization page shows the IMU's matrix and the wheel odometry's matrix with the index of every element, and the elements that the EKF fuses.

## What the EKF does with a variance

The EKF on the robot fuses the wheel odometry's yaw rate and the IMU's yaw rate into one estimate, as [What the filter fuses](../../packages/imu/robot_localization.md#what-the-filter-fuses) on the Localization page explains. The variances decide how much each measurement counts. For one value, the filter's correction step has three formulas. The filter's estimate $\hat x$ has the variance $P$, and a measurement $z$ of the same value has the variance $R$:

$$
\begin{aligned}
K &= \frac{P}{P + R} \\
\hat x^+ &= \hat x + K\,(z - \hat x) \\
P^+ &= (1 - K)\,P
\end{aligned}
$$

- **The gain $K$:** a number between 0 and 1, the share of the difference $z - \hat x$ by which the estimate moves toward the measurement.
- **A large variance $R$:** the measurement is uncertain. $K$ is small, and the estimate moves little.
- **A small variance $R$:** the measurement is certain. $K$ is close to 1, and the estimate moves almost to the measurement.
- **The new variance $P^+$:** $P^+ = PR/(P + R)$, which is smaller than both $P$ and $R$. The fused estimate is more certain than each of its two inputs.

**An example with the robots' variances:** take the wheel odometry's yaw rate as the estimate, $\hat x$ = 0.50 rad/s with its variance $P$ = 0.03 (rad/s)². Then the IMU measures $z$ = 0.46 rad/s, with the firmware's variance $R$ = 4 · 10⁻⁴ (rad/s)². The two yaw rates are example values:

$$
\begin{aligned}
K &= \frac{0.03}{0.03 + 0.0004} = 0.987 \\
\hat x^+ &= 0.50 + 0.987 \cdot (0.46 - 0.50) \\
&= 0.4605\ \text{rad/s} \\
P^+ &= (1 - 0.987) \cdot 0.03 \\
&= 3.9 \cdot 10^{-4}\ (\text{rad/s})^2
\end{aligned}
$$

The fused yaw rate lies close to the IMU's, because the IMU's variance is 75 times smaller than the wheels'. The order doesn't matter: starting from the IMU's yaw rate and fusing the wheels' gives $K$ = 0.013 and the same 0.4605 rad/s. In the simulation, the IMU's variance is 2.5 · 10⁻⁵ (rad/s)², and $K$ is 0.9992.

The example leaves out the filter's other step. In the robot's filter, the estimate $\hat x$ and its variance $P$ come from the prediction, which moves the estimate forward in time with a model of the robot's motion. The prediction makes $P$ grow, so a measurement counts more after a longer time without one.

??? quote "ekf.cpp, lines 159–197: the correction step in robot_localization"
    robot_localization 2.7.7 computes the same three steps with matrices, because one measurement can carry several values. In the function [`correct()`](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ekf.cpp#L55), the estimate's covariance $P$ is `estimateErrorCovariance_`, the measurement's covariance $R$ is `measurementCovarianceSubset`, the gain $K$ is `kalmanGainSubset`, and the difference $z - \hat x$ is `innovationSubset`. `stateToMeasurementSubset`, $H$, picks the measured values out of the filter's state. For one value, $H$ is 1, and the formulas in the comments become the three formulas above. The update of the covariance in lines 191–197 is written in the Joseph form, which gives $(1 - K)\,P$ for one value with this gain and keeps the matrix symmetric despite rounding. Line 186 checks the Mahalanobis distance, which [What the validator doesn't check](../../packages/imu/robot_localization.md#what-the-validator-doesnt-check) on the Localization page explains:

    ```cpp linenums="159"
    --8<-- "https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ekf.cpp:159:197"
    ```

**A variance of 0:** with $R$ = 0, $K$ is 1, and the filter would take every measurement as exact. robot_localization replaces a measurement's variance below 10⁻⁹ with 10⁻⁹, and a negative variance with its absolute value, so that the gain stays computable. So a covariance of all zeros, which `sensor_msgs/Imu` defines as unknown, reaches the filter as almost certain. The robots' [IMU sample validator](../../packages/imu/robot_localization.md#checks-1-to-3-the-samples-form) drops a sample whose three variances of the angular velocity aren't all positive, before the EKF gets the sample.

??? quote "ekf.cpp, lines 119–143: negative and very small variances"
    The excerpt shows the two replacements in `correct()`, for each variance `measurementCovarianceSubset(i, i)` of the measurement:

    ```cpp linenums="119"
    --8<-- "https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ekf.cpp:119:143"
    ```

## Quaternions

### A rotation as four numbers

A rotation in space turns a body by an angle about an axis. Every rotation has such an axis and an angle ([Euler's rotation theorem](https://en.wikipedia.org/wiki/Euler%27s_rotation_theorem)). A **quaternion** $\mathbf q$ stores the axis and the angle together in four numbers. For a rotation by the angle $\alpha$ about the axis with the unit vector $\mathbf n = (n_x, n_y, n_z)$, the quaternion is:

$$
\mathbf q = \begin{pmatrix} q_x \\ q_y \\ q_z \\ q_w \end{pmatrix}
= \begin{pmatrix} n_x \sin(\alpha/2) \\ n_y \sin(\alpha/2) \\ n_z \sin(\alpha/2) \\ \cos(\alpha/2) \end{pmatrix}
$$

- **$q_x$, $q_y$ and $q_z$:** the axis, scaled with the sine of half the angle.
- **$q_w$:** the cosine of half the angle.
- **The direction of a positive angle:** by the right-hand rule, as ROS's convention [REP 103](https://www.ros.org/reps/rep-0103.html#chirality) asks. Point your right thumb along the axis. Your fingers then curl in the direction of a positive angle.

The four numbers are the parts of a quaternion, a number with one real and three imaginary parts, as a complex number has one real and one imaginary part. Wikipedia's [Quaternions and spatial rotation](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation) shows how the products of quaternions rotate vectors. This page uses a quaternion only as the four numbers of a rotation.

Three angles, the roll, the pitch and the yaw, can describe a rotation too, and [Roll, pitch and yaw](#roll-pitch-and-yaw) converts between the two forms. Three angles have special cases. At a pitch of ±90°, the roll and the yaw turn about the same axis, and the angles of a rotation are no longer unique ([gimbal lock](https://en.wikipedia.org/wiki/Gimbal_lock)). A quaternion has no such special case. ROS's convention REP 103 prefers quaternions to the other forms of a rotation, for their compact form and because they have no such special cases ([REP 103: rotation representation](https://www.ros.org/reps/rep-0103.html#rotation-representation)). ROS messages and tf2, ROS's library for coordinate frames, store every rotation as a quaternion.

### The length 1 and the rotation that does nothing

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
- **Three degrees of freedom:** four numbers with one condition leave three free numbers. So a rotation has three degrees of freedom, and the covariance of an orientation is a 3 × 3 matrix, as in [the IMU's message](#the-imus-message).
- **The identity:** the angle $\alpha$ = 0 gives $\mathbf q = (0, 0, 0, 1)$, the rotation that does nothing. DiffBot's URDF model mounts its IMU's frame `imu_link` without a rotation, with [`imu_rpy`]({{ ekf_src_url }}/diffbot_description/urdf/diffbot.urdf.xacro#L4-L10) = `0 0 0`, so that frame's quaternion is the identity. The mounting on DiffBot itself still needs to be measured, as the comment above the setting says.
- **Two quaternions for each rotation:** $\mathbf q$ and $-\mathbf q$ describe the same rotation. $-\mathbf q$ belongs to the angle $\alpha$ + 360° about the same axis.

**Four zeros:** $(0, 0, 0, 0)$ has the length 0, so the four zeros are no rotation at all. Yet the robots' IMU messages carry four zeros as their orientation. The firmware never sets the orientation, so its message keeps the four zeros that a new message starts with: the constructor of rosserial's [`geometry_msgs::Quaternion`](https://github.com/frankjoshua/rosserial_arduino_lib/blob/0.9.1/src/geometry_msgs/Quaternion.h#L24-L30) sets all four numbers to 0. In the simulation, the adapter sets the four numbers to 0 in its function [`callback()`]({{ ekf_src_url }}/diffbot_gazebo/scripts/imu_raw_adapter.py#L29-L36). A new `geometry_msgs/Quaternion` of ROS Noetic on the robot's computer starts as four zeros too. The −1 in `orientation_covariance[0]` says that the field holds no data, so a node that reads an orientation checks the −1 before it uses the four numbers. In ROS 2, the [definition of `geometry_msgs/Quaternion`](https://github.com/ros2/common_interfaces/blob/4.9.2/geometry_msgs/msg/Quaternion.msg) gives $q_w$ the default 1, so a new quaternion is the identity there.

### Roll, pitch and yaw

The roll, the pitch and the yaw are the angles of three rotations, about the x, the y and the z axis. The robots' URDF models and tf2 use the same order: first the roll about the x axis, then the pitch about the fixed y axis, then the yaw about the fixed z axis. The fixed axes are those of the frame that the rotation starts from, the parent frame. ROS's [URDF specification of a joint](http://wiki.ros.org/urdf/XML/joint) defines the `rpy` of a joint's origin in this way, and tf2's [`setRPY()`](https://github.com/ros/geometry2/blob/0.7.10/tf2/include/tf2/LinearMath/Quaternion.h#L91-L110) computes the quaternion for this order.

A rotation about one axis alone has a short quaternion. For the yaw $\psi$ alone, the rotation about the z axis, the axis is $\mathbf n = (0, 0, 1)$ and the angle $\alpha = \psi$:

$$
\mathbf q = \left(0,\ 0,\ \sin\frac{\psi}{2},\ \cos\frac{\psi}{2}\right)
$$

Back from the quaternion, the yaw is $\psi = 2\,\operatorname{atan2}(q_z, q_w)$. A roll alone has only $q_x$ and $q_w$, and a pitch alone only $q_y$ and $q_w$.

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

### Remo's IMU, turned by −90°

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

The forward acceleration appears in the board's $f_y$ and in the robot's $f_x$. The angular velocity turns the same way: $(\omega_x, \omega_y, \omega_z)$ of `imu_link` becomes $(\omega_y, -\omega_x, \omega_z)$ of `chassis_link`. The yaw rate $\omega_z$ stays the same, because both z axes point up. The EKF turns the IMU's measurement into the robot's frame in this way, as [The frames](../../packages/imu/robot_localization.md#the-frames) on the Localization page explains. The covariance turns with the rotation too, as $R\,\boldsymbol\Sigma\,R^\mathsf T$. The IMU's covariance is a sphere, $\sigma^2 I_3$, and a sphere looks the same from every direction: $R\,\sigma^2 I_3\,R^\mathsf T = \sigma^2 I_3$.

**In tf:** robot_state_publisher publishes the transform of each fixed joint of the robot's URDF model on the topic `/tf_static`, in its function [`publishFixedTransforms()`](https://github.com/ros/robot_state_publisher/blob/1.15.3/src/robot_state_publisher.cpp#L145-L168). The transform from `chassis_link` to `imu_link` carries the quaternion above as its rotation. `rosrun tf tf_echo chassis_link imu_link` prints the transform wherever robot_state_publisher runs with Remo's URDF model. For remo_description 2.5.0, in a ROS Noetic container on the PC, the output starts with these lines:

```console
At time 0.000
- Translation: [-0.040, 0.023, 0.013]
- Rotation: in Quaternion [0.000, 0.000, -0.707, 0.707]
            in RPY (radian) [0.000, 0.000, -1.571]
            in RPY (degree) [0.000, 0.000, -90.000]
```

The translation is the position from `imu_xyz` in m, rounded to millimeters. The quaternion is the one computed above, in the order $q_x$, $q_y$, $q_z$, $q_w$.

### How ROS stores a quaternion

ROS's message for a quaternion, [`geometry_msgs/Quaternion`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/geometry_msgs/msg/Quaternion.msg), has four numbers, with $q_w$ last. The excerpt shows its definition from ROS's common_msgs:

```text title="Quaternion.msg: the geometry_msgs/Quaternion message" linenums="1"
--8<-- "https://raw.githubusercontent.com/ros/common_msgs/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/geometry_msgs/msg/Quaternion.msg"
```

The fields `x`, `y`, `z` and `w` are $q_x$, $q_y$, $q_z$ and $q_w$. Some libraries take $q_w$ first: the constructor `Quaternion(w, x, y, z)` of the C++ library Eigen, for example ([Eigen's `Quaternion` class](https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html)). Before you copy four numbers from one library to another, check the order that each one uses.

The robots' messages carry quaternions in three places:

- **The IMU's orientation:** `orientation` of [`sensor_msgs/Imu`](#the-imus-message) is four zeros with −1 in `orientation_covariance[0]`, so the field holds no data. The BNO055 can compute its own orientation as a quaternion ([BNO055 datasheet, section 3.6.5.5 on page 37](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf#page=37)). The firmware doesn't read that quaternion, because the EKF uses only the yaw rate.
- **The odometry's orientation:** `pose.pose.orientation` of `nav_msgs/Odometry` holds the heading $\theta$ as a yaw-only quaternion. The drive controller publishes this message on `/diffbot/mobile_base_controller/odom`, and the EKF on `/diffbot/odometry/filtered`, as [The two outputs](../../packages/imu/robot_localization.md#the-two-outputs) on the Localization page describes.
- **The transforms:** each transform on `/tf` and `/tf_static` has a rotation, for example the heading in the EKF's transform from `odom` to `base_footprint`, and Remo's −90° in the transform from `chassis_link` to `imu_link`.

## Further reading

- **The covariance matrix:** Wikipedia's [covariance matrix](https://en.wikipedia.org/wiki/Covariance_matrix) gives the definition and its properties.
- **The normal distribution:** Wikipedia's [normal distribution](https://en.wikipedia.org/wiki/Normal_distribution) has the formulas and the history.
- **Quaternions in ROS:** the ROS wiki's tutorial [quaternion basics](http://wiki.ros.org/tf2/Tutorials/Quaternions) shows how to create, invert and multiply quaternions with tf2.
- **Gaussian filters:** *Probabilistic Robotics*, chapter 3, derives the Kalman filter and the EKF from the normal distribution ([Amazon.com][amazon_book_probabilistic_robotics_us], [Amazon.de][amazon_book_probabilistic_robotics_ger], affiliate links).
