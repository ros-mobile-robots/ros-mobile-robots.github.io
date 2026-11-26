# IMU Simulation: From Measurement to Message

This page follows one sample of the simulated IMU from Gazebo's IMU sensor to the message on [`/diffbot/imu/data_raw`](index.md#what-the-simulated-imu-publishes), in the order in which the code runs. Read the page to find where a value of the message is set. This page belongs to [IMU: Simulation](index.md), which shows the finished message.

## One sample, through the three parts

Three parts make the sample, one after the other. The numbers are from a run of Remo's simulation, taken while Remo turned left on the spot at 0.5 rad/s. The run recorded Gazebo's own IMU messages and the two ROS topics side by side. The sample pairs one ROS message with Gazebo's newest IMU message whose stamp isn't later than the ROS message's stamp:

1. **Gazebo's IMU sensor measures:** the IMU sensor measures the angular velocity $\boldsymbol\omega$ and the specific force $\mathbf f$ of [`imu_link`](position-and-axes.md#the-two-frames-and-their-fixed-joint). In Gazebo's message of this pair, the yaw rate $\omega_z$, the angular velocity about the upward axis, is 0.50000 rad/s, and the measurement's time is 9.592 s of simulation time. [Part 1](#part-1-gazebos-imu-sensor-measures) shows Gazebo's code.
2. **The robots' IMU plugin makes the message:** when the plugin publishes, it reads the IMU sensor's latest values and adds noise to each of them. The message's yaw rate $\tilde\omega_z$ is 0.49882 rad/s, which is 0.00118 rad/s less than the yaw rate of Gazebo's paired message. The plugin fills the covariances and stamps the message with 9.593 s, the IMU sensor's update time, which is 1 ms later than the paired measurement's time. The plugin publishes the message on [`/diffbot/imu/gazebo`](#the-header-and-publishing). This message also has an orientation $\mathbf q$, a quaternion with four numbers. [Part 2](#part-2-the-plugin-makes-the-message) shows the plugin's code.
3. **The adapter gives the message the firmware's form:** the firmware's message on the real robot has no orientation. So the adapter marks the orientation unavailable, with four zeros and −1 in [`orientation_covariance[0]`](index.md#how-the-message-stores-a-covariance), and publishes the message on [`/diffbot/imu/data_raw`](index.md#what-the-simulated-imu-publishes). The yaw rate, 0.49882 rad/s, and the stamp, 9.593 s, stay as the plugin set them. [Part 3](#part-3-the-adapter) shows the adapter's code.

The pairing is by time alone. The recording can't tell which of Gazebo's measurements the plugin read for a message, so the 0.00118 rad/s is the difference between the two paired readings, not a measured noise value.

The robot's URDF model declares the IMU sensor and its plugin. [The IMU sensor in the robot's URDF model](#the-imu-sensor-in-the-robots-urdf-model) shows where, before anything runs.

## The code's flow

The figure shows what each part does, in the order in which the code runs. The sections of this page follow the figure from top to bottom. The arrows show how the data reaches the message, not who calls whom. Gazebo runs the IMU sensor and the plugin in two separate loops, and the plugin reads the IMU sensor's latest measurement. Most calls of the plugin's [`UpdateChild()`](#part-2-the-plugin-makes-the-message) end at its checks and publish nothing. [The checks before a call publishes](#the-checks-before-a-call-publishes) has a figure of the checks with their exits.

```mermaid
graph TB
  subgraph PART1 [ ]
    MEASURE["<b>1. Gazebo's IMU sensor</b><br/>UpdateImpl(), every 20 ms<br/>of simulation time on average<br/>measures angularVel and linearAcc<br/>of imu_link"]
  end
  subgraph PART2 [ ]
    CHECKS["<b>2. The robots' IMU plugin</b><br/>UpdateChild(), at every simulation step<br/>the checks: clock went back,<br/>rate limit, subscriber<br/>most calls end here"]
    READ["reads the sensor and adds noise<br/>gyroscope_data, accelerometer_data<br/>GuassianKernel()"]
    COVARIANCE["fills the covariances<br/>gaussian_noise squared<br/>at the elements 0, 4 and 8"]
    PUBLISH["sets the header and publishes<br/>frame_id, stamp, publish()<br/>then stores last_time"]
    CHECKS -->|a call that passes the checks| READ
    READ --> COVARIANCE
    COVARIANCE --> PUBLISH
  end
  subgraph PART3 [ ]
    CALLBACK["<b>3. The adapter</b><br/>callback(), for every received message<br/>marks the orientation unavailable"]
  end
  MEASURE -->|the plugin reads the latest measurement| CHECKS
  PUBLISH -->|imu/gazebo| CALLBACK
  CALLBACK -->|imu/data_raw| READERS["IMU sample validator, tests"]
```

- [The IMU sensor in the robot's URDF model](#the-imu-sensor-in-the-robots-urdf-model) comes first. This section shows where the IMU sensor and the plugin are declared, before anything runs.
- [Part 1](#part-1-gazebos-imu-sensor-measures) is Gazebo's code.
- [Part 2](#part-2-the-plugin-makes-the-message) is the plugin's function `UpdateChild()`, from its first line to its last: [the checks](#the-checks-before-a-call-publishes), [the sensor's values with noise](#reading-the-sensor-and-adding-noise), [the covariances](#filling-the-covariances), and [the header and the call that publishes](#the-header-and-publishing).
- [Part 3](#part-3-the-adapter) is the adapter's callback.

## The IMU sensor in the robot's URDF model

Each robot's URDF model declares the simulated IMU sensor in a [xacro](position-and-axes.md#the-link-and-the-joint-in-the-robots-urdf-model) macro named [`imu_sensor`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L204). [Position and Axes](position-and-axes.md#the-link-and-the-joint-in-the-robots-urdf-model) explains what a macro and its call are. The diagram shows the macro's call, what the macro adds to the robot's URDF model, and who reads each part. Read the diagram from top to bottom. The call supplies two things, the parent link and the origin, and both meet in the joint. The two boxes with a bold first line hold what the macro adds. The labels on the arrows are the names that xacro and the URDF use for these connections:

```mermaid
graph TB
  CALL["The robot's xacro file<br/>calls the macro imu_sensor"]
  PARENT["chassis_link<br/>a link of the robot's URDF model,<br/>there before the call"]
  ORIGIN["the call's origin<br/>the IMU's xyz and rpy<br/>on chassis_link"]
  subgraph BOTH [ ]
    JOINT["<b>The macro adds, for ROS and Gazebo</b><br/>imu_joint: a fixed joint"]
    LINK["imu_link: a link with<br/>the board's shape and mass"]
    JOINT -->|child| LINK
  end
  subgraph BLOCK [ ]
    SENSOR["<b>The macro adds, for Gazebo only</b><br/>the Gazebo block of imu_link:<br/>Gazebo's IMU sensor, type imu,<br/>50 measurements per second"]
    PLUGIN["in the IMU sensor's element:<br/>the robots' IMU plugin,<br/>a library file"]
    SENSOR -->|contains| PLUGIN
  end
  CALL -->|parent_link| PARENT
  CALL -->|origin| ORIGIN
  PARENT -->|parent| JOINT
  ORIGIN -->|places| JOINT
  LINK -->|the Gazebo block's reference| SENSOR
```

- **[`imu_joint`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L236) and [`imu_link`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L208):** ROS and Gazebo both read them. The joint names [`chassis_link`](position-and-axes.md#the-two-frames-and-their-fixed-joint) as its parent and `imu_link` as its child, and the call's origin places the joint. `robot_state_publisher` publishes the joint's transform, and Gazebo places the IMU sensor with this transform.
- **The Gazebo block:** only Gazebo reads this block. URDF tools and ROS ignore the block. The block's attribute [`reference`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L242) names `imu_link`, the link that carries the IMU sensor. The IMU sensor's element contains the plugin's element.

DiffBot's macro is in [`common_sensors.xacro`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L204-L258) of [`diffbot_description`](../../../robot-description.md), Remo's in [remo_description's file of the same name](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/include/common_sensors.xacro#L211-L268).

??? example "The complete macro `imu_sensor` of DiffBot's URDF model"

    The excerpt shows the whole macro with the comment above it, from [`common_sensors.xacro`, lines 191–258]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L191-L258). The link is in lines 208–234, the joint in lines 236–240 and the Gazebo block in lines 242–257:

    ```xml title="common_sensors.xacro: the macro imu_sensor" linenums="191"
    --8<-- "{{ imu_sim_src_raw }}/diffbot_description/urdf/include/common_sensors.xacro:191:258"
    ```

The next excerpt shows the macro's Gazebo block, which declares the IMU sensor and its plugin, from [`common_sensors.xacro`, lines 242–257]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L242-L257):

```xml title="common_sensors.xacro: the IMU sensor and its plugin" linenums="242" hl_lines="2 5 10"
--8<-- "{{ imu_sim_src_raw }}/diffbot_description/urdf/include/common_sensors.xacro:242:257"
```

- **`<gazebo reference="imu_link">`:** this block holds what only Gazebo reads. `reference` attaches the IMU sensor to `imu_link`, so the IMU sensor measures at `imu_link`'s position and along its axes.
- **[`type="imu"`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L243):** this is Gazebo's IMU sensor. [`update_rate`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L245) is the rate at which the IMU sensor measures, 50 times per second of simulation time.
- **[`<plugin>`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L246):** the plugin turns the IMU sensor's values into ROS messages. `filename` names its library, [`libdiffbot_gazebo_ros_imu_sensor.so`](plugin.md#why-the-robots-have-their-own-plugin), the robots' own copy of the IMU plugin that comes with ROS. [The Robots' Own IMU Plugin](plugin.md) explains why the robots have a copy, and how [Remo's URDF model gets the library's name](plugin.md#the-plugins-library-in-remos-urdf-model).

The lines inside `<plugin>` are the plugin's parameters:

- **[`robotNamespace`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L247) and [`topicName`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L248):** `/diffbot` and `imu/gazebo` give the topic [`/diffbot/imu/gazebo`](#the-header-and-publishing).
- **[`frameName`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L249):** `imu_link` becomes the message's `header.frame_id`.
- **[`updateRateHZ`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L250):** 50 is the plugin's own limit, at most 50 messages per second.
- **[`gaussianNoise`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L251):** 0.005, in the last highlighted line, is the standard deviation $\sigma$ of the [noise that the plugin adds](#reading-the-sensor-and-adding-noise).
- **[`xyzOffset`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L252):** this parameter has no effect. The plugin reads the value and never uses it, as the comment in its source says: "POSITION OFFSET, UNUSED", in [line 275 of `gazebo_ros_imu_sensor.cpp`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L275). The plugin that comes with ROS has the same comment.
- **[`rpyOffset`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L253) and [`initialOrientationAsReference`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L254):** both change only the orientation, which the adapter removes. `rpyOffset` is the one part of the offset that the plugin applies, in [line 150]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L150).

Where the IMU sensor sits and how it's turned comes from the joint's origin in the robot's URDF model, not from the two offsets.

??? info "If you inspect the SDF model that Gazebo makes from the robot's URDF model"

    Gazebo converts the robot's URDF model into a model in its own format, SDF, before it simulates the robot. This SDF model has no `imu_link`. Gazebo merges links that are joined by fixed joints into one link, and `imu_link` hangs on a fixed joint.

    The IMU sensor is still there, at the same place. In the SDF model, the IMU sensor belongs to the link that remains, [`base_footprint`](../robot_localization.md#localization-wheel-odometry-and-imu-ekf) for DiffBot, and the sensor's pose in that link is `imu_link`'s position and rotation. `gz sdf -p` prints the SDF model when you [convert the robot's URDF model as on the Gazebo Package page](../../../diffbot_gazebo.md#troubleshooting).

    For ROS nothing changes. `imu_link` stays the IMU's frame, in the message's header and in the transforms.

## Part 1: Gazebo's IMU sensor measures

The IMU sensor derives its measurements from motion that Gazebo has already computed. Gazebo's physics engine computes the motion of every link in every simulation step, and the IMU sensor reads the motion of the link that carries it. From that motion, the IMU sensor computes two vectors along [`imu_link`](position-and-axes.md#the-two-frames-and-their-fixed-joint)'s axes: the angular velocity $\boldsymbol\omega$ and the specific force $\mathbf f$.

**Who runs this code, and when:** Gazebo's sensor manager runs the sensors in a loop of its own, apart from the physics engine. For each sensor, the loop calls [`Sensor::Update()`]({{ gazebo_src_url }}/gazebo/sensors/Sensor.cc#L235). This function reads the world's simulation time into [`simTime`]({{ gazebo_src_url }}/gazebo/sensors/Sensor.cc#L260). It adds two times: the time since the sensor's last update, and [`updateDelay`]({{ gazebo_src_url }}/gazebo/sensors/Sensor.cc#L278), the lateness that an earlier update left over. Once this sum reaches the update period, `Sensor::Update()` calls the IMU sensor's [`ImuSensor::UpdateImpl()`]({{ gazebo_src_url }}/gazebo/sensors/ImuSensor.cc#L302), which measures. Then `Sensor::Update()` stores `simTime` as the sensor's [`lastUpdateTime`]({{ gazebo_src_url }}/gazebo/sensors/Sensor.cc#L292). With the [`update_rate`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L245) of 50 in the robot's URDF model, the period is 20 ms of simulation time. Because of the carried lateness, two updates can be less than 20 ms apart: with 1 ms carried, the next update can run as early as 19 ms later. In the recorded run, the stamps of Gazebo's consecutive IMU messages, which are the measurements' times, were 19, 20 or 21 ms apart. The robots' IMU plugin takes no part in the measurement. The plugin only reads the results afterwards.

??? example "`Sensor::Update()`: when Gazebo runs a measurement, and the update time that it stores"

    The excerpt shows the part of `Sensor::Update()` that runs for the robots' IMU sensor, from [`Sensor.cc` of Gazebo Classic 11.15.1, lines 254–295]({{ gazebo_src_url }}/gazebo/sensors/Sensor.cc#L254-L295). Line 260 reads the world's simulation time. Lines 272–273 compute [`adjustedElapsed`]({{ gazebo_src_url }}/gazebo/sensors/Sensor.cc#L272-L273): the time since the last update plus `updateDelay`. Lines 275–276 return while `adjustedElapsed` is less than the update period. Lines 278–279 keep by how much `adjustedElapsed` exceeds the period, as the `updateDelay` of the next update. Lines 285–286 clear this carried delay when it reaches a full period: with a period of 20 ms, an `adjustedElapsed` of 40 ms leaves no delay for the next call. Line 289 calls [`UpdateImpl()`]({{ gazebo_src_url }}/gazebo/sensors/Sensor.cc#L289), and line 292 stores `simTime` as `lastUpdateTime`:

    ```cpp title="Sensor.cc (Gazebo Classic 11.15.1): the update period and lastUpdateTime" linenums="254" hl_lines="7 19 20 22 23 25 26 32 33 36 39"
    --8<-- "{{ gazebo_src_raw }}/gazebo/sensors/Sensor.cc:254:295"
    ```

The excerpt shows the lines of Gazebo's source code that compute both vectors, from `ImuSensor::UpdateImpl()` in [`ImuSensor.cc` of Gazebo Classic 11.15.1, lines 322–380]({{ gazebo_src_url }}/gazebo/sensors/ImuSensor.cc#L322-L380), under the [Apache License 2.0]({{ gazebo_src_url }}/LICENSE):

```cpp title="ImuSensor.cc (Gazebo Classic 11.15.1): the two vectors" linenums="322" hl_lines="1 3 5 17 24 25 30 31 44 45 48 49 54 55"
--8<-- "{{ gazebo_src_raw }}/gazebo/sensors/ImuSensor.cc:322:380"
```

The variables, in the order in which the code computes them:

1. **[`msg`]({{ gazebo_src_url }}/gazebo/sensors/ImuSensor.cc#L322):** in line 322, the latest motion data of the link that carries the IMU sensor. In Gazebo's SDF model that is the link that remains after the merging, as the box above explains. At the end of every simulation step, this link publishes its linear and angular velocity, in [`Link::PublishData()`]({{ gazebo_src_url }}/gazebo/physics/Link.cc#L1265-L1281). The IMU sensor receives each of these messages in [`ImuSensor::OnLinkData()`]({{ gazebo_src_url }}/gazebo/sensors/ImuSensor.cc#L252-L258) and keeps the newest one. These messages stay inside Gazebo. They aren't ROS messages.
2. **[`timestamp`]({{ gazebo_src_url }}/gazebo/sensors/ImuSensor.cc#L324):** in line 324, the simulation time of this motion data. This time is the measurement's time. Line 338 writes the measurement's time into Gazebo's own IMU message, which stays inside Gazebo as the link's messages do.
3. **[`dt`]({{ gazebo_src_url }}/gazebo/sensors/ImuSensor.cc#L326):** in line 326, the time between this motion data and the motion data of the IMU sensor's last measurement.
4. **[`imuWorldPose`]({{ gazebo_src_url }}/gazebo/sensors/ImuSensor.cc#L342):** in line 342, where the IMU sensor is in the simulated world, and how the IMU sensor is turned. `imuWorldPose` combines the link's pose with the sensor's pose in the link.
5. **[`linkWorldAngularVel`]({{ gazebo_src_url }}/gazebo/sensors/ImuSensor.cc#L345-L346):** in line 345, the link's angular velocity from `msg`, along the world's axes.
6. **[`angularVel`]({{ gazebo_src_url }}/gazebo/sensors/ImuSensor.cc#L351-L352):** in line 351, the same angular velocity along the IMU sensor's axes, which are `imu_link`'s. `imuWorldPose.Rot().Inverse().RotateVector()` converts a vector from the world's axes to the sensor's. This is the angular velocity $\boldsymbol\omega$:

    $$
    \boldsymbol\omega = \begin{pmatrix} \omega_x \\ \omega_y \\ \omega_z \end{pmatrix}
    $$

7. **[`linkWorldLinearVel`]({{ gazebo_src_url }}/gazebo/sensors/ImuSensor.cc#L360-L361):** in line 360, the link's linear velocity from `msg`, along the world's axes.
8. **[`imuWorldLinearVel`]({{ gazebo_src_url }}/gazebo/sensors/ImuSensor.cc#L365-L366):** in line 365, the velocity of the point where the IMU sensor sits. That point isn't the link's origin, so a turning link moves that point too. The cross product adds this share.
9. **[`linearAcc`]({{ gazebo_src_url }}/gazebo/sensors/ImuSensor.cc#L369-L370):** in line 369, how much `imuWorldLinearVel` changed since the last measurement, `lastImuWorldLinearVel`, divided by `dt`, and converted to the sensor's axes. This is the acceleration $\mathbf a$ of the point where the IMU sensor sits.
10. **`linearAcc` again:** in line 375, the code subtracts gravity $\mathbf g$, converted to the sensor's axes too. `linearAcc` is now the **specific force** $\mathbf f$, in m/s²:

    $$
    \mathbf f = \mathbf a - \mathbf g =
    \begin{pmatrix} a_x - g_x \\ a_y - g_y \\ a_z - g_z \end{pmatrix}
    $$

The IMU sensor keeps both results. The plugin fetches them later with [`sensor->AngularVelocity()`]({{ gazebo_src_url }}/gazebo/sensors/ImuSensor.cc#L261) and [`sensor->LinearAcceleration()`]({{ gazebo_src_url }}/gazebo/sensors/ImuSensor.cc#L270).

**Two times of a measurement:** Gazebo keeps two times for the IMU sensor, and they aren't the same.

- **The measurement's time** is `timestamp`, the simulation time of the motion data that the measurement was computed from. Gazebo's own IMU message carries this time.
- **The sensor's update time** is `lastUpdateTime`, the world's simulation time that `Sensor::Update()` read when it ran the measurement. [`sensor->LastUpdateTime()`]({{ gazebo_src_url }}/gazebo/sensors/Sensor.cc#L420) returns the update time. The plugin stamps the ROS message with this time, as [the header and publishing](#the-header-and-publishing) shows.

The sensor loop runs apart from the physics engine, so the world's clock has usually moved on since the link published its motion data. The run of [the sample above](#one-sample-through-the-three-parts) paired each of 524 ROS messages with Gazebo's newest IMU message whose stamp wasn't later. For 523 of them, the ROS message's stamp was 1 ms later than the stamp of Gazebo's message, one simulation step. For one, the two stamps were equal. These numbers are from one run and from a pairing by time. They aren't a rule of Gazebo.

At rest, $\mathbf a$ is zero. With `imu_link`'s z axis up, gravity is $\mathbf g = (0, 0, -9.8)$ m/s², so the IMU sensor reads $\mathbf f = (0, 0, +9.8)$ m/s². The overview shows [the angular velocity](index.md#the-angular-velocity) and [the linear acceleration](index.md#the-linear-acceleration) each in a figure, and explains why an accelerometer reads the acceleration minus gravity.

Gazebo's gravity is 9.8 m/s² in the robots' worlds. That is the default of SDF, Gazebo's format for worlds and models, as the [SDF specification of `<gravity>`](http://sdformat.org/spec?ver=1.6&elem=world#world_gravity) says.

## Part 2: the plugin makes the message

The robots' IMU plugin makes the message in one function, [`UpdateChild()`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L137-L192). [`Load()`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L131-L132) connects `UpdateChild()` to Gazebo's world update event. The excerpt shows this call, from [`gazebo_ros_imu_sensor.cpp`, lines 131–132]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L131-L132):

```cpp title="gazebo_ros_imu_sensor.cpp: Load() connects UpdateChild()" linenums="131"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp:131:132"
```

So Gazebo's physics thread, the part of Gazebo that computes the simulation's steps, calls `UpdateChild()` at the start of every step. With the 1 ms step of the simulation test's world, that is 1000 calls per second of simulation time. Most calls end after the checks. A call that passes them goes through the four steps below in this order, and publishes at the end.

### The checks before a call publishes

The first lines of [`UpdateChild()`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L137-L147) decide whether this call makes a message at all. The figure shows the two decisions and where a call leaves:

```mermaid
graph TB
  CALL["UpdateChild() starts<br/>reads current_time<br/>clock went back:<br/>last_time = current_time"]
  RATE{"one period<br/>since last_time?"}
  RETURN["returns,<br/>publishes nothing"]
  SUBSCRIBER{"a subscriber?"}
  MESSAGE["fills and publishes<br/>the message"]
  STORE["stores<br/>last_time = current_time"]
  CALL --> RATE
  RATE -->|no| RETURN
  RATE -->|yes| SUBSCRIBER
  SUBSCRIBER -->|yes| MESSAGE
  MESSAGE --> STORE
  SUBSCRIBER -->|no| STORE
```

The excerpt shows these lines, from [`gazebo_ros_imu_sensor.cpp`, lines 137–147]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L137-L147):

```cpp title="gazebo_ros_imu_sensor.cpp: the start of UpdateChild()" linenums="137" hl_lines="3 5 6"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp:137:147"
```

- **Two times:** [`current_time`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L139) is the IMU sensor's update time, [`sensor->LastUpdateTime()`]({{ gazebo_src_url }}/gazebo/sensors/Sensor.cc#L420) in line 139: the world's simulation time at which Gazebo last ran a measurement of the IMU sensor. It isn't the measurement's own time, as [two times of a measurement](#part-1-gazebos-imu-sensor-measures) in Part 1 explains. [`last_time`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L191) is the value that `current_time` had in the last call that got past the rate limit.
- **A clock that went back:** this is the first [`if`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L141-L142), lines 141–142. A reset of the simulation sets Gazebo's clock back to zero. Then `current_time` is earlier than `last_time`, and the plugin sets `last_time` to `current_time`. The rate limit counts from the new time, and the plugin can publish again one period later. [Publishing again after a reset](plugin.md#publishing-again-after-a-reset) explains what would happen without these two lines.
- **The rate limit:** this is the second `if`, lines 144–145, the figure's first diamond. [`update_rate`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L254) is the parameter [`updateRateHZ`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L250), 50. A call returns right away while less than one period, 1/50 s = 20 ms, has passed since `last_time`. So at most 50 calls per second of simulation time go on. Of the 1000 calls per second, about 950 end here.
- **A subscriber:** this is the third `if`, line 147, the figure's second diamond. A call that got past the rate limit fills and publishes a message only while the topic has a subscriber. Without one, the call skips to the line that stores `last_time`. In the simulation, the adapter always subscribes.

### Reading the sensor and adding noise

A call that passed the checks reads the IMU sensor's latest values and writes them into the IMU message, with noise. The excerpt shows these lines, from [`gazebo_ros_imu_sensor.cpp`, lines 149–166]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L149-L166):

```cpp title="gazebo_ros_imu_sensor.cpp: the sensor's values, with noise" linenums="149" hl_lines="3 4"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp:149:166"
```

- **[`accelerometer_data`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L151) and [`gyroscope_data`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L152):** the highlighted lines fetch the two results of [Part 1](#part-1-gazebos-imu-sensor-measures) from Gazebo's IMU sensor. `accelerometer_data` is the specific force $\mathbf f$, and `gyroscope_data` is the angular velocity $\boldsymbol\omega$.
- **[`orientation`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L150):** line 150 fetches the IMU sensor's orientation $\mathbf q$ in the simulated world, a quaternion of length 1.
- **[`imu_msg`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L155-L166):** the lines from 155 on write each number into the message and add one random number to each.

!!! warning "The orientation on `/diffbot/imu/gazebo` isn't ready to use"

    Lines 155–158 add a separate random number to each of the quaternion's four numbers, x, y, z and w. The plugin doesn't bring the result back to the length 1. A quaternion describes a rotation only with the length 1. In one run of Remo's simulation, the length of the plugin's quaternion lay between 0.983 and 1.016. So don't read the robot's attitude from [`/diffbot/imu/gazebo`](#the-header-and-publishing). The topic that the robots' nodes read, [`/diffbot/imu/data_raw`](index.md#what-the-simulated-imu-publishes), has no orientation: the adapter marks it unavailable in [Part 3](#part-3-the-adapter).

The message's values are the IMU sensor's values plus noise:

$$
\tilde{\boldsymbol\omega} = \boldsymbol\omega + \mathbf n_\omega, \qquad \tilde{\mathbf f} = \mathbf f + \mathbf n_f
$$

$\tilde{\boldsymbol\omega}$ and $\tilde{\mathbf f}$ are what the message carries. Each component $n$ of $\mathbf n_\omega$ and $\mathbf n_f$ is a new random number in every message, from a normal distribution with mean 0 and standard deviation $\sigma$:

$$
n \sim \mathcal N(0, \sigma^2), \qquad
\sigma = \begin{cases} 0.005 \text{ rad/s} & \text{for } \mathbf n_\omega \\ 0.005 \text{ m/s}^2 & \text{for } \mathbf n_f \end{cases}
$$

**Where $\sigma$ comes from:** $\sigma$ is the plugin's parameter [`gaussianNoise`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L251), 0.005, one number without a unit for both quantities, in the Gazebo block of the robot's URDF model, [line 251 of the macro](#the-imu-sensor-in-the-robots-urdf-model). The plugin reads the parameter into its variable [`gaussian_noise`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L266) when Gazebo loads the plugin. The excerpt shows these lines, from [`gazebo_ros_imu_sensor.cpp`, lines 263–273]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L263-L273):

```cpp title="gazebo_ros_imu_sensor.cpp: the parameter gaussianNoise" linenums="263" hl_lines="2 3 4"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp:263:273"
```

**How a random number is drawn:** [`GuassianKernel(0, gaussian_noise)`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L194-L206) returns one random number with mean 0 and the standard deviation `gaussian_noise`. The excerpt above calls `GuassianKernel()` once for each value.

??? example "The function `GuassianKernel()`"

    The excerpt shows the function that draws one random number, from [`gazebo_ros_imu_sensor.cpp`, lines 194–206]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L194-L206). It turns two evenly distributed random numbers into one normally distributed number, and scales that number with [`sigma`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L194):

    ```cpp title="gazebo_ros_imu_sensor.cpp: GuassianKernel()" linenums="194"
    --8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp:194:206"
    ```

**Two places where noise can be set:** the simulated IMU could get noise from two parts, and the robots use one of them.

- **Gazebo's IMU sensor** can add noise of its own, before the plugin reads the values. A URDF model asks for this noise with [`<noise>`](https://github.com/gazebosim/sdformat/blob/1774642ee6ac2d46f9fb335470dd7e881d0e6cc5/sdf/1.7/imu.sdf#L83-L113) elements inside the sensor's [`<imu>`](https://github.com/gazebosim/sdformat/blob/1774642ee6ac2d46f9fb335470dd7e881d0e6cc5/sdf/1.7/imu.sdf) element, one for each axis of the angular velocity and of the linear acceleration, as the [SDF specification of the IMU sensor](https://github.com/gazebosim/sdformat/blob/1774642ee6ac2d46f9fb335470dd7e881d0e6cc5/sdf/1.7/imu.sdf#L83-L113) lists them. The robots' URDF models have no such element. So Gazebo's IMU sensor adds no noise.
- **The plugin** adds noise when it fills the message. `gaussianNoise` is its only setting for that. All the noise that the robots' URDF models configure comes from the plugin.

**One value for both quantities:** $\sigma$ is 0.005 rad/s for the angular velocity and 0.005 m/s² for the specific force. This is a limit of the plugin, which has one parameter for all values, and not a model of a real sensor. A gyroscope's noise in rad/s and an accelerometer's noise in m/s² are different quantities with different sizes. The acceleration's noise doesn't matter to the EKF today, because the EKF fuses only the yaw rate $\omega_z$. The yaw rate's noise and variance do reach the EKF. They are simulation settings too. So the simulation doesn't show how the real sensor's noise affects the EKF.

**A simulation setting, not a measurement:** 0.005 isn't the BNO055's noise. At rest on Remo, the real BNO055's gyroscope scatters with a standard deviation of 0.0005 to 0.0008 rad/s, and its accelerometer with about 0.02 to 0.03 m/s². [Bias and noise at rest](../driver-bno055.md#bias-and-noise-at-rest) on the BNO055 page has these measurements.

### Filling the covariances

After the values, the same call writes the covariances. The excerpt shows these lines, from [`gazebo_ros_imu_sensor.cpp`, lines 168–178]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L168-L178):

```cpp title="gazebo_ros_imu_sensor.cpp: the covariances" linenums="168"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp:168:178"
```

[`gn2`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L169) is [`gaussian_noise`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L266) squared, the variance $\sigma^2 = 2.5 \cdot 10^{-5}$. The plugin writes this variance into the elements 0, 4 and 8 of each covariance, in (rad/s)² for the angular velocity and in (m/s²)² for the specific force. These three elements are the matrix's diagonal, as [how the message stores a covariance](index.md#how-the-message-stores-a-covariance) on the overview shows. Element 8 of [`angular_velocity_covariance`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L173-L175) is the variance of the yaw rate $\omega_z$, which the EKF uses. The other elements stay 0. For the angular velocity:

$$
\Sigma_\omega = \begin{pmatrix} \sigma^2 & 0 & 0 \\ 0 & \sigma^2 & 0 \\ 0 & 0 & \sigma^2 \end{pmatrix} = \sigma^2 I_3
$$

$I_3$ is the 3 × 3 identity matrix. The specific force's covariance $\Sigma_f$ has the same numbers, in (m/s²)².

**Why the other elements are 0:** an element off the diagonal is the covariance of two axes' errors. The plugin draws a separate random number for each axis, so the noise on one axis says nothing about the noise on another. The two numbers are independent, and the covariance of two independent numbers is 0, as Wikipedia's article on [covariance](https://en.wikipedia.org/wiki/Covariance#Uncorrelatedness_and_independence) shows. On a real sensor, the errors of two axes can be correlated: they tend to rise and fall together. Correlated errors can give elements off the diagonal that aren't 0.

Lines 170–172 of the excerpt write the orientation's covariance. The adapter replaces this covariance in [Part 3](#part-3-the-adapter).

### The header and publishing

The last lines of [`UpdateChild()`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L180-L192) set the header, publish the message and store the time for the next call's rate limit. The excerpt shows them, from [`gazebo_ros_imu_sensor.cpp`, lines 180–192]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L180-L192):

```cpp title="gazebo_ros_imu_sensor.cpp: the end of UpdateChild()" linenums="180" hl_lines="6 12"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp:180:192"
```

- **The frame:** [`body_name`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L240-L242) is the parameter [`frameName`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L249), [`imu_link`](position-and-axes.md#the-two-frames-and-their-fixed-joint).
- **The stamp:** [`current_time`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L139) is the IMU sensor's update time, from the first line of `UpdateChild()`. So [`header.stamp`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L182-L183) is the time at which Gazebo ran the measurement, not the measurement's own time, which only Gazebo's own IMU message carries. [Two times of a measurement](#part-1-gazebos-imu-sensor-measures) in Part 1 explains the difference. In the recorded run, the stamp was 1 ms later than the paired Gazebo stamp for 523 of the 524 paired messages.
- **[`publish()`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L185):** line 185, the first highlighted line, publishes the finished message on [`/diffbot/imu/gazebo`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L247-L248). This is the one place where the plugin publishes.
- **[`last_time`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L191):** line 191, the second highlighted line, stores `current_time` for the rate limit of the next calls. Line 191 also runs when no node subscribes.

!!! note "The original plugin calls `ros::spinOnce()` after `publish()`"

    [gazebo_plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/2.9.3/gazebo_plugins)' original IMU plugin calls [`ros::spinOnce()`](plugin.md#one-request-with-each-plugin) right after `publish()`, in [line 150 of its source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L150). The robots' copy has the comment of lines 186–188 at this place instead. This one call is the main reason why the robots have their own plugin: with `ros::spinOnce()` at this place, the simulation's clock can stop for good while the controllers start. [Why the plugin doesn't spin](plugin.md#why-the-plugin-doesnt-spin) explains how.

## Part 3: the adapter

The adapter is there so that the nodes that read the IMU get its message in the same form in the simulation as on the real robot: a [`sensor_msgs/Imu`](index.md#what-the-simulated-imu-publishes) message on `/diffbot/imu/data_raw`, with the orientation marked unavailable. On the real robot, the Teensy's firmware publishes this topic without an orientation, as [what the firmware publishes](../driver-bno055.md#what-the-firmware-publishes) shows. The plugin's message on [`/diffbot/imu/gazebo`](#the-header-and-publishing) has an orientation $\mathbf q$. So [`imu_raw_adapter.py`]({{ imu_sim_src_url }}/diffbot_gazebo/scripts/imu_raw_adapter.py) takes each of the plugin's messages, marks the orientation unavailable as the firmware does, and publishes the result on `/diffbot/imu/data_raw`. Of the plugin's values, the adapter changes nothing else. The stamp, the frame, the angular velocity, the specific force and the two vectors' covariances stay as the plugin set them. So the noise and the variances are the simulation's, not the firmware's, as [the simulation and the real robot](index.md#the-simulation-and-the-real-robot) compares.

[`diffbot_view.launch`]({{ imu_sim_src_url }}/diffbot_gazebo/launch/diffbot_view.launch#L37-L39) starts the adapter in the [`diffbot`]({{ imu_sim_src_url }}/diffbot_gazebo/launch/diffbot_view.launch#L39) namespace. So the adapter's relative topic names become `/diffbot/imu/gazebo` and `/diffbot/imu/data_raw`. The excerpt shows the line that starts the adapter, with its comment, from [`diffbot_view.launch`, lines 37–39]({{ imu_sim_src_url }}/diffbot_gazebo/launch/diffbot_view.launch#L37-L39):

```xml title="diffbot_view.launch: the adapter's node" linenums="37"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/launch/diffbot_view.launch:37:39"
```

The next excerpt shows the adapter's code, from [`imu_raw_adapter.py`, lines 16–36]({{ imu_sim_src_url }}/diffbot_gazebo/scripts/imu_raw_adapter.py#L16-L36):

```python title="imu_raw_adapter.py" linenums="16" hl_lines="4 5 16 17 18 19 20"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/scripts/imu_raw_adapter.py:16:36"
```

- **How a field is marked unavailable:** a `sensor_msgs/Imu` message always has an orientation field. To say that a field holds no data, a publisher sets the first element of the field's covariance to −1. The [message definition's comments]({{ common_msgs_src_url }}/sensor_msgs/msg/Imu.msg#L10-L13) ask for that, and so does [REP 145](https://www.ros.org/reps/rep-0145.html#topics), the ROS convention for IMU drivers. A node that reads the message should check that element and ignore the field. [`ORIENTATION_UNAVAILABLE`]({{ imu_sim_src_url }}/diffbot_gazebo/scripts/imu_raw_adapter.py#L20) is such a covariance: −1, then eight zeros. [How the message stores a covariance](index.md#how-the-message-stores-a-covariance) shows the array next to the other cases.
- **A convention, not a rule to drop the orientation:** REP 145 doesn't ask an IMU driver to leave out the orientation. A driver whose sensor estimates an orientation can publish the orientation on [`imu/data`](https://www.ros.org/reps/rep-0145.html). The firmware's topic, `imu/data_raw`, is REP 145's name for the accelerometer's and the gyroscope's data alone.
- **[`callback()`]({{ imu_sim_src_url }}/diffbot_gazebo/scripts/imu_raw_adapter.py#L29-L36):** the callback sets the four numbers of the quaternion $\mathbf q$ to 0 and the orientation covariance to a copy of the constant. [`list()`](https://docs.python.org/3.8/library/stdtypes.html#list) makes the copy, so that the messages don't share the constant's list. Then the callback publishes the same message object.
- **What stays:** the header's stamp and frame, the angular velocity $\tilde{\boldsymbol\omega}$, the specific force $\tilde{\mathbf f}$ and their covariances $\Sigma_\omega$ and $\Sigma_f$. The adapter doesn't stamp the message again. The [IMU sample validator](../robot_localization.md#the-imu-sample-validator) checks how old each sample is, and a new stamp would hide a delay.
- **What doesn't stay, besides the orientation:** the header's sequence number, [`header.seq`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/msg.py#L142-L145). ROS numbers the messages of each publisher when it sends them. So the adapter's messages count on their own. In [the sample above](#one-sample-through-the-three-parts), the plugin's message had the `seq` 460 and the adapter's message the `seq` 447.

??? example "Where rospy sets `header.seq`"

    The excerpt shows the lines of rospy's [`serialize_message()`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/msg.py#L126) that write the publisher's own count into the header before a message is sent, from [`msg.py` in ros_comm 1.17.4, lines 142–145]({{ ros_comm_src_url }}/clients/rospy/src/rospy/msg.py#L142-L145), under the BSD license. `seq` is the number that the publisher counts up with each message. The adapter's callback never sets `header.seq` itself:

    ```python title="msg.py (rospy 1.17.4): the publisher's count goes into header.seq" linenums="142" hl_lines="4"
    --8<-- "{{ ros_comm_src_raw }}/clients/rospy/src/rospy/msg.py:142:145"
    ```
