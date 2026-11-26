# Noise and Covariance

A stationary gyroscope gives slightly different readings from one sample to the next. Variance describes the spread on one axis. Covariance describes how errors on two axes vary together. The robots' [extended Kalman filter (EKF)](../../packages/imu/robot_localization.md) uses these uncertainties when it combines the gyroscope with wheel odometry.

The sections build on each other, from one measured value to a vector of values and then to the filter:

1. [The noise of one value](#the-noise-of-one-value): the bias, the mean, the variance and the standard deviation, worked by hand for three readings of a gyroscope.
2. [The normal distribution](#the-normal-distribution): the shape of the noise that the simulated IMU adds and that the EKF assumes.
3. [The covariance of two values](#the-covariance-of-two-values): whether the errors of two values go up and down together, and why independent errors have the covariance 0.
4. [The covariance matrix and its ellipsoid](#the-covariance-matrix-and-its-ellipsoid): all variances and covariances of a vector in one matrix, and the shape that the matrix describes.
5. [How ROS messages store a covariance](#how-ros-messages-store-a-covariance): a matrix as an array of 9 or 36 numbers, and the arrays with a special meaning.
6. [What the EKF does with a variance](#what-the-ekf-does-with-a-variance): a larger variance gives a measurement less weight.

Each example says where its numbers come from: a setting in the robots' code, a measurement on Remo, or example values that make a formula easy to follow. The messages also carry a rotation as four numbers, which [Quaternions](quaternions.md) explains.

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

The robots' IMU, the BNO055, reports the angular velocity in whole steps of $\Delta$ = 1/900 rad/s, about 0.00111 rad/s ([BNO055 datasheet, table 3-22 on page 35](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf#page=35)). Take three readings of a gyroscope at rest, one step apart: $\omega_{z,1}$ = 0 rad/s, $\omega_{z,2} = \Delta$ and $\omega_{z,3} = 2\Delta$. Their mean is:

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
| The firmware's setting [`angular_velocity_variance`]({{ ekf_src_url }}/diffbot_firmware/lib/config/diffbot_base_config.h#L71) for each axis, a starting value | $\sigma$ = 0.02 rad/s | $\sigma^2$ = 4 · 10⁻⁴ (rad/s)² |
| The wheel odometry's yaw rate, a setting of the drive controller ([The two lists](../../packages/imu/robot_localization.md#the-two-lists)) | $\sigma \approx$ 0.17 rad/s | $\sigma^2$ = 0.03 (rad/s)² |

The firmware's starting value is about a thousand times the variance that Remo's gyroscope showed at rest. The comment above the setting in [`diffbot_base_config.h`](https://github.com/ros-mobile-robots/diffbot/blob/22d4d2854a1c88d6f97453c099708b8c37601741/diffbot_firmware/lib/config/diffbot_base_config.h#L55-L68) gives a reason for the larger value and a condition for changing the value:

- **The typical offset:** the BNO055's datasheet gives the gyroscope a typical offset of ±1 °/s, about 0.017 rad/s, with the chip's own offset cancellation switched off ([BNO055 datasheet, table 0-2 on page 15](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf#page=15)). The chosen standard deviation of 0.02 rad/s covers that offset too, so that the EKF doesn't take a yaw rate with a bias as exact.
- **Vibration:** driving adds vibration, which a recording at rest doesn't contain. Before the value changes, the comment asks for a comparison with a recording while driving, in which the vibration has to be told apart from the robot's real turning.

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

The figure draws the density $p(n)$ of the simulated IMU's noise $n$. Each bracket under the axis spans one, two or three standard deviations on both sides of the mean, and its number is the share of the area under the curve between the bracket's ends:

--8<-- "includes/math-normal-distribution.html"

**Why this distribution:** an error that is the sum of many small, independent parts tends to be normally distributed, whatever the distributions of the parts are ([central limit theorem](https://en.wikipedia.org/wiki/Central_limit_theorem)). The Kalman filter and the EKF assume normally distributed errors. Then a mean and a covariance describe each error completely, and the filter only has to carry these numbers from one step to the next. *Probabilistic Robotics* by Thrun, Burgard and Fox derives these filters in its chapter 3, "Gaussian Filters" ([Amazon.com][amazon_book_probabilistic_robotics_us], [Amazon.de][amazon_book_probabilistic_robotics_ger], affiliate links).

**A real sensor:** the BNO055 reports whole steps of 1/900 rad/s. At rest, the standard deviation of Remo's gyroscope is smaller than one step ([bench checks](../../packages/imu/driver-bno055.md#bench-checks)), so its readings fall on a few steps around their mean. A histogram of them has a few bars, not a smooth bell. For such a sensor, the normal distribution is a model that the EKF's formulas need, not an exact description of the readings.

## The covariance of two values

The gyroscope measures three angular velocities at once, $\omega_x$, $\omega_y$ and $\omega_z$. The noise of each has its own variance. One question remains: do the errors of two values go up and down together? The **covariance** answers that question.

For $N$ pairs of readings of two values $x$ and $y$, such as $\omega_x$ and $\omega_y$, the sample covariance $s_{xy}$ multiplies the two deviations of each pair and averages the products, again with $N - 1$:

$$
s_{xy} = \frac{1}{N-1}\sum_{i=1}^{N} (x_i - \bar x)(y_i - \bar y)
$$

The covariance $\sigma_{xy}$ of the distribution, the population covariance, is the expected product of the two deviations from the distribution's means $\mu_x$ and $\mu_y$: the average of the product over all the values that the two could take. Two deviations with the same sign give a positive product. Two deviations with opposite signs give a negative product. The covariance is:

- **Positive:** when the positive products outweigh the negative ones on average.
- **Negative:** when the negative products outweigh the positive ones on average.
- **Zero:** when the average product is zero.

The products count with their sizes, so a few large products can decide the sign against many small ones. Take these six pairs of deviations, in example values: (3, 3), (−3, −3), (1, −1), (1, −1), (−1, 1) and (−1, 1). Four of the six products are negative. The sample covariance is still positive, (9 + 9 − 1 − 1 − 1 − 1) / 5 = 2.8.

The figure shows example pairs from a distribution with a positive covariance and from one with the covariance 0: the deviations $x - \mu_x$ and $y - \mu_y$ of a position from its mean $\mu$, in cm. Compare the two clouds. In this example of a positive covariance, many dots lie in the two quadrants marked +, where both deviations have the same sign. In the example of the covariance 0, dots appear about equally often in all four quadrants. The signs and the sizes of the products both matter. The ellipses over each cloud belong to the cloud's distribution, as [The covariance ellipsoid](#the-covariance-ellipsoid) explains:

--8<-- "includes/math-covariance-scatter.html"

With $y = x$, the formula becomes the one for $s^2$, so the covariance of a value with itself is its variance.

**An example of four readings:** the table gives the deviations of four pairs of readings $i$ from their means, in steps $\Delta$ of the gyroscope. Each sign of the deviation of $\omega_x$ meets both signs of the deviation of $\omega_y$ once:

| $i$ | $\omega_{x,i} - \bar\omega_x$ | $\omega_{y,i} - \bar\omega_y$ | Product |
|:--|:--|:--|:--|
| 1 | $-\Delta$ | $-\Delta$ | $+\Delta^2$ |
| 2 | $-\Delta$ | $+\Delta$ | $-\Delta^2$ |
| 3 | $+\Delta$ | $-\Delta$ | $-\Delta^2$ |
| 4 | $+\Delta$ | $+\Delta$ | $+\Delta^2$ |

The products add up to 0, so $s_{xy}$ = 0. If the deviations of $\omega_y$ were $-\Delta$, $-\Delta$, $+\Delta$ and $+\Delta$ instead, always with the sign of the deviation of $\omega_x$, all four products would be $+\Delta^2$. Then $s_{xy} = 4\Delta^2/3$, as large as each of the two variances: the two errors go up and down fully together.

**The correlation:** the covariance has the unit of $x$ times the unit of $y$, so its size is hard to judge. Divided by both sample standard deviations, the sample covariance becomes the sample correlation $r_{xy} = s_{xy} / (s_x\,s_y)$, a number between −1 and +1 without a unit. The division needs both sample standard deviations to be positive. In the second example, $r_{xy}$ = 1.

### Why independent errors have the covariance 0

Two errors are **independent** when one of them says nothing about the other. For independent variables, knowing $x$ leaves the average of $y - \mu_y$ at zero. So the product $(x - \mu_x)(y - \mu_y)$ averages to the error of $x$ times 0, which is 0. Written with the expected value $\mathrm E[\,\cdot\,]$, the mean over all the values that the errors could take:

$$
\begin{aligned}
\sigma_{xy} &= \mathrm E\big[(x - \mu_x)(y - \mu_y)\big] \\
&= \mathrm E[x - \mu_x] \cdot \mathrm E[y - \mu_y] \\
&= 0 \cdot 0 = 0
\end{aligned}
$$

Independence guarantees the factorization in the middle line: the expected value of a product of independent values is the product of their expected values. Dependent variables can also have the covariance 0, as the second limit below shows.

**The simulated IMU:** the plugin calls [`GuassianKernel()`]({{ ekf_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L194-L206) once for each value of each message, so each axis gets a random number of its own. The noise on one axis says nothing about the noise on another, and the covariance of every two axes is 0. That is why the plugin's message has zeros off the diagonal of its covariance matrices, which [the next section](#the-covariance-matrix-and-its-ellipsoid) introduces.

Two limits apply:

- **A recording isn't the model:** even for independent errors, the sample covariance of a recording can be positive, negative or exactly zero. With enough independent pairs, the sample covariance approaches the population's value. The round cloud of the figure above comes from a distribution with the covariance 0, and its 200 pairs have a sample covariance of about −0.009 cm². The 0 in a message is the model's value.
- **The covariance 0 doesn't always mean independent:** the covariance only measures whether two errors grow together along a straight line. Errors can depend on each other in other ways and still have the covariance 0. Take an $x$ that is −2, −1, 1 or 2 with the same probability, and $y = x^2$. Then $x$ determines $y$ completely, and the covariance is still 0: the products of the deviations are −3, +1.5, −1.5 and +3, which average 0. For two errors whose pair follows a normal distribution, the covariance 0 does mean independent ([Covariance: uncorrelatedness and independence](https://en.wikipedia.org/wiki/Covariance#Uncorrelatedness_and_independence)).

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

For the angular velocity and the acceleration, the robots' IMU messages carry diagonal matrices, with the same variance on every axis and zeros everywhere else. For the simulated IMU's angular velocity, with $\sigma^2 = 2.5 \cdot 10^{-5}$ (rad/s)², the matrix is:

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

The number $d = \sqrt{\mathbf e^\mathsf T \boldsymbol\Sigma^{-1} \mathbf e}$ is called the **Mahalanobis distance** of the deviation $\mathbf e$. The Mahalanobis distance counts how many standard deviations $\mathbf e$ lies from the mean, in the direction of $\mathbf e$. The ellipsoid of one standard deviation holds the deviations with $d$ = 1, and the ellipsoid of two standard deviations those with $d$ = 2. robot_localization can reject a measurement whose $d$ is too large, as [What the validator doesn't check](../../packages/imu/robot_localization.md#what-the-validator-doesnt-check) on the Localization page explains.

**A turned ellipse:** two values give an ellipse instead of an ellipsoid. Take a position estimate $(x, y)$ with this covariance matrix, in example values, the matrix of the example pairs with the positive covariance in [The covariance of two values](#the-covariance-of-two-values):

$$
\boldsymbol\Sigma = \begin{pmatrix} 5 & 3 \\ 3 & 5 \end{pmatrix}\ \text{cm}^2
$$

Without the covariance of 3 cm², the ellipse would be a circle with the radius $\sqrt{5}$ cm = 2.24 cm. With the covariance, the matrix has the eigenvalue 8 cm² along the direction (1, 1), at 45° between the x and the y axis, and the eigenvalue 2 cm² along (1, −1), at a right angle to the first. So the ellipse has the semi-axis $\sqrt{8}$ cm = 2.83 cm along the 45° line and $\sqrt{2}$ cm = 1.41 cm across. The positive covariance turns the ellipse toward the direction in which $x$ and $y$ are both larger or both smaller. Such a covariance appears, for example, when a robot drives at 45° to the x axis and is unsure how far the robot drove. An error in the distance then moves the estimate along the robot's path, in $x$ and $y$ together.

The figure shows this ellipse of one standard deviation. Follow the two thick semi-axes from the mean: they lie along the eigenvectors, and the longer one is turned by 45° from the x axis. The dashed circle is the ellipse without the covariance:

--8<-- "includes/math-covariance-axes.html"

**The share inside:** for normally distributed values, the density is the same everywhere on the ellipsoid, and the ellipsoids of one, two and three standard deviations lie inside each other. The share of the values inside an ellipsoid depends on how many values the vector has. One value has an interval, two values an ellipse and three values an ellipsoid:

| Inside | 1 value | 2 values | 3 values |
|:--|:--|:--|:--|
| 1 standard deviation | 68.3 % | 39.3 % | 19.9 % |
| 2 standard deviations | 95.4 % | 86.5 % | 73.9 % |
| 3 standard deviations | 99.7 % | 98.9 % | 97.1 % |

The shares follow from the [chi-squared distribution](https://en.wikipedia.org/wiki/Chi-squared_distribution) with one, two and three degrees of freedom. So for a vector of three normally distributed values, the ellipsoid of one standard deviation holds only about a fifth of the vectors.

The figure shows the ellipses of one, two and three standard deviations of the example matrix, the deviations with the Mahalanobis distance $d$ = 1, 2 and 3. The label on each ellipse is the share for two values from the table, the share of the positions inside that ellipse:

--8<-- "includes/math-covariance-shares.html"

**In RViz:** RViz's Odometry display draws the covariance of each pose, as [Comparing the two odometries](../../packages/imu/robot_localization.md#comparing-the-two-odometries) on the Localization page sets the display up. RViz 1.14.26 draws the position's covariance as this ellipsoid, with semi-axes of one standard deviation along the eigenvectors, in its function [`computeShapeScaleAndOrientation3D()`](https://github.com/ros-visualization/rviz/blob/1.14.26/src/rviz/default_plugin/covariance_visual.cpp#L100-L137). The display's setting [Scale](https://github.com/ros-visualization/rviz/blob/1.14.26/src/rviz/default_plugin/covariance_property.cpp#L68-L71) multiplies the semi-axes, and its default 1 shows one standard deviation. When the pose's variances of the height $z$, the roll and the pitch are 0 or less, RViz draws the ellipse of $x$ and $y$ instead ([`pose_2d_`](https://github.com/ros-visualization/rviz/blob/1.14.26/src/rviz/default_plugin/covariance_visual.cpp#L338)).

## How ROS messages store a covariance

A ROS message has no type for a matrix. A message stores a covariance matrix as an array of numbers, row by row: first the elements of the first row, then those of the second, and so on. The message definitions call this order **row major**. ROS's convention [REP 103: covariance representation](https://www.ros.org/reps/rep-0103.html#covariance-representation) asks for the same order in every message. With $n$ rows and $n$ columns, counted from 0 as in C++ and Python, the element in row $i$ and column $j$ has the index $k$ in the array:

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

- **Three matrices:** each of the message's three measurements has its own 3 × 3 matrix, in lines 18, 21 and 24: [`orientation_covariance`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/sensor_msgs/msg/Imu.msg#L18), [`angular_velocity_covariance`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/sensor_msgs/msg/Imu.msg#L21) and [`linear_acceleration_covariance`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/sensor_msgs/msg/Imu.msg#L24). `angular_velocity_covariance[8]`, for example, is the variance of the yaw rate $\sigma^2_{\omega_z}$, the only element of the three matrices that the robots' EKF uses.
- **The orientation's matrix is 3 × 3:** the three rows and columns of `orientation_covariance` describe small rotation errors about the fixed x, y and z axes, as the comment in line 18 says: "about x, y, z axes". A [unit quaternion](quaternions.md#the-length-1-and-the-rotation-that-does-nothing) has four components but only three rotational degrees of freedom.

The comments in lines 5–13 give three kinds of arrays a meaning. ROS's convention for IMU drivers, [REP 145: topics](https://www.ros.org/reps/rep-0145.html#topics), asks for the two special kinds, all zeros and −1, in the same way. Here they are for the angular velocity:

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
    robot_localization 2.7.7 computes the same three steps with matrices, because one measurement can carry several values. In the function [`correct()`](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ekf.cpp#L55), the estimate's covariance $P$ is `estimateErrorCovariance_`, the measurement's covariance $R$ is `measurementCovarianceSubset`, the gain $K$ is `kalmanGainSubset`, and the difference $z - \hat x$ is `innovationSubset`. `stateToMeasurementSubset`, $H$, picks the measured values out of the filter's state. When the state is one value that the sensor measures directly, $H$ is 1, and the formulas in the comments become the three formulas above. The update of the covariance in lines 191–197 is written in the Joseph form, which gives $(1 - K)\,P$ for one value with this gain and keeps the matrix symmetric despite rounding. Line 186 checks the Mahalanobis distance, which [The covariance ellipsoid](#the-covariance-ellipsoid) introduces:

    ```cpp linenums="159"
    --8<-- "https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ekf.cpp:159:197"
    ```

**A variance of 0:** with $R$ = 0, $K$ is 1, and the filter would take the measurement as exact. If the estimate's variance $P$ were 0 as well, $K$ would be 0/0, which can't be computed. robot_localization's function `correct()` prevents both. The function first replaces a negative variance of a measurement with its absolute value, and then a variance below 10⁻⁹ with 10⁻⁹. So a covariance of all zeros, which `sensor_msgs/Imu` defines as unknown, reaches the filter as almost certain. The robots' [IMU sample validator](../../packages/imu/robot_localization.md#checks-1-to-3-the-samples-form) drops a sample whose three variances of the angular velocity aren't all positive, before the EKF gets the sample.

??? quote "ekf.cpp, lines 119–143: negative and very small variances"
    The excerpt shows the two replacements in `correct()`, for each variance `measurementCovarianceSubset(i, i)` of the measurement:

    ```cpp linenums="119"
    --8<-- "https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ekf.cpp:119:143"
    ```

## Further reading

- **The covariance matrix:** Wikipedia's [covariance matrix](https://en.wikipedia.org/wiki/Covariance_matrix) gives the definition and its properties.
- **The normal distribution:** Wikipedia's [normal distribution](https://en.wikipedia.org/wiki/Normal_distribution) has the formulas and the history.
- **Gaussian filters:** *Probabilistic Robotics*, chapter 3, derives the Kalman filter and the EKF from the normal distribution ([Amazon.com][amazon_book_probabilistic_robotics_us], [Amazon.de][amazon_book_probabilistic_robotics_ger], affiliate links).
