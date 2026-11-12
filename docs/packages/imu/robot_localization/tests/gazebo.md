# Test: The EKF in Gazebo

This page belongs to [Localization: Wheel Odometry and IMU (EKF)](../../robot_localization.md). It walks through [`test_ekf.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_ekf.py), the test that turns the simulated robot with the EKF on.

The test proves one thing in the simulation: when the wheel odometry reports a wrong turn, the EKF's heading $\theta$ still follows the robot's true turn, because the filter takes the yaw rate $\omega_z$ from the IMU. The test makes the wheel odometry wrong on purpose. It gives the drive controller a wheel separation $b_c$ that is 25 % larger than the simulated robot's wheel separation $b$. The same simulation also checks that only the EKF publishes the transform `odom → base_footprint`. Both simulated robots run the test, DiffBot and Remo. The test doesn't show how the filter behaves on the real robot. [What the test doesn't show](#what-the-test-doesnt-show) lists the differences.

`test_ekf.py` has two tests:

| Test | What it checks | Limit |
|:--|:--|:--|
| `test_ekf_follows_the_true_turn_with_a_wrong_wheel_separation` | In a left turn of 3 s: the IMU sample validator forwards the IMU's samples, the wheel odometry's turn falls behind the true turn, and the EKF's turn is the true turn | more than 45 Hz, more than 0.15 rad behind, within 0.05 rad |
| `test_ekf_is_the_only_odom_tf_publisher` | Only the node `/diffbot/ekf_localization` publishes the transform `odom → base_footprint` on the topic `/tf` | exactly one node |

## The turn from above

The figure shows the turn test from above, with the values of a saved run and one drawing for each simulated robot. In a drawing, the robot starts with its heading along the dashed line to the right and turns left, counterclockwise. Each arc is one turn from this start: the robot's true turn $\Delta\theta_\text{true}$ from Gazebo, the turn $\Delta\theta_\text{EKF}$ of the EKF's estimate and the turn $\Delta\theta_\text{odom}$ of the wheel odometry. The label at the end of an arc is the arc's name in the line that the test prints, `true`, `EKF` or `odometry`, with the measured turn. The robot is drawn as it stands after the turn, and the line on its body points forward. The small green sector at the end of the EKF's arc is where this arc has to end. The line above a drawing gives the robot's two wheel separations $b$ and $b_c$, and the list under the drawing gives the results of the two checks on the turn.

--8<-- "includes/localization-test-gazebo.html"

## What the simulation starts

Two rostest files run the test, [`sim_ekf_diffbot.test`]({{ diffbot_repo_url }}/diffbot_gazebo/test/sim_ekf_diffbot.test) for the simulated DiffBot and [`sim_ekf_remo.test`]({{ diffbot_repo_url }}/diffbot_gazebo/test/sim_ekf_remo.test) for the simulated Remo. A rostest file is a launch file with a `<test>` element. rostest starts the file's nodes, runs the test node and stops everything when the test node ends, as [the test file on Simulation Test](../../../../development/simulation-test.md#1-the-test-file) explains. The excerpt shows what DiffBot's file starts, from [`sim_ekf_diffbot.test`, lines 9–21]({{ ekf_src_url }}/diffbot_gazebo/test/sim_ekf_diffbot.test#L9-L21):

```xml title="sim_ekf_diffbot.test: the simulation with the EKF, the wheel separation and the test node" linenums="9"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/sim_ekf_diffbot.test:9:21"
```

- **`sim_drive.launch`:** starts the simulation that the other simulation tests use too. `ekf` set to `true` switches the EKF on.
- **`gazebo_port`:** is the port of this test file's Gazebo server. Each simulation test file has a port of its own, because catkin runs the test files at the same time.
- **`wheel_separation`:** the `<param>` line comes after the include, so its value replaces the value that the included files loaded from the robot's `geometry.yaml`. 0.1725 m is 1.25 times DiffBot's 0.138 m. `sim_ekf_remo.test` sets 0.18 m, 1.25 times Remo's 0.144 m.
- **The test node:** `ekf_diffbot` runs `test_ekf.py`. The node's private parameter `model` names the robot whose `geometry.yaml` the test reads.

The next excerpt shows how `ekf` reaches the drive controller's launch file, from [`sim_drive.launch`, lines 15–25]({{ ekf_src_url }}/diffbot_gazebo/test/sim_drive.launch#L15-L25):

```xml title="sim_drive.launch: Gazebo without a window, and the controllers with the ekf argument" linenums="15"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/sim_drive.launch:15:25"
```

`diffbot_view.launch` starts Gazebo's server and spawns the robot from its URDF model. `diffbot_control.launch` loads the drive controller's configuration, with the robot's `geometry.yaml`, and starts the controllers. With `ekf` true, `diffbot_control.launch` also sets the controller's `enable_odom_tf` to `false` and includes `ekf.launch`, which starts the IMU sample validator and the EKF, as [Which launch file starts what](../../robot_localization.md#which-launch-file-starts-what) shows.

## Why a wrong wheel separation

The drive controller, `diff_drive_controller`, uses its `wheel_separation` parameter twice. It turns a commanded yaw rate into two wheel speeds, and it turns the wheels' movement back into the odometry's yaw rate. $b$ is the wheel separation of the simulated robot, the distance between its two wheels in the URDF model. $b_c = 1.25\,b$ is the controller's value. The factor 1.25 is the test's `SEPARATION_SCALE`. For a commanded yaw rate $\omega_\text{cmd}$, the controller makes the right wheel's speed $v_r$ larger than the left wheel's speed $v_l$ by

$$
v_r - v_l = \omega_\text{cmd}\, b_c
$$

The wheels of the simulated robot are $b$ apart, so the robot turns with the yaw rate

$$
\omega_z = \frac{v_r - v_l}{b} = \frac{b_c}{b}\,\omega_\text{cmd} = 1.25\,\omega_\text{cmd}
$$

The wheel odometry divides the same speed difference by $b_c$ and reports $\omega_\text{cmd}$. So the robot turns 25 % more than its wheel odometry says. This constant error stands in for the errors of a real robot's wheels, slip and a wheel separation that is a little off. The simulated IMU measures the robot's true yaw rate $\omega_z$. A filter that follows the IMU ends at the true heading, and a filter that follows the wheels ends where the wheel odometry ends.

The test commands $\omega_\text{cmd}$ = 0.5 rad/s (`TURN_VELOCITY`) for 3 s (`DURATION`). For these round numbers, the wheel odometry reports 1.5 rad and the robot turns 1.875 rad.

In a run, the measured turns are a little smaller than these round numbers, as [the figure](#the-turn-from-above) shows. The true turn stays 1.25 times the wheel odometry's turn.

## The constants

??? example "The constants of `test_ekf.py`, lines 41–69"

    The excerpt shows the topics, the names and the numbers of the test, from [`test_ekf.py`, lines 41–69]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf.py#L41-L69). `STARTUP_TIMEOUT` and `POLL_INTERVAL` are seconds of wall time, the computer's clock. `DURATION` and `SETTLE` are seconds of simulation time, Gazebo's clock.

    ```python title="test_ekf.py: the constants" linenums="41"
    --8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf.py:41:69"
    ```

## Reading the three headings

The test compares three headings $\theta$ of the robot: the true heading from Gazebo, the heading of the wheel odometry and the heading of the EKF's estimate. Each of these three sources publishes on a topic. The class `HeadingObserver` subscribes to the topics and keeps the latest heading of each source. [Waiting until everything runs](#waiting-until-everything-runs) and [The turn](#the-turn) show where the test reads them.

A message of these topics carries a pose, not a heading. The excerpt shows the two functions that turn a pose into a heading and two headings into a turn, from [`test_ekf.py`, lines 86–94]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf.py#L86-L94):

```python title="test_ekf.py: yaw() and turned()" linenums="86"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf.py:86:94"
```

- **`yaw()`:** a pose stores its orientation as a quaternion. `euler_from_quaternion()` returns roll, pitch and yaw, and `yaw()` returns the third, the heading $\theta$.
- **`turned()`:** returns the change of the heading from `start` to `end`, between $-\pi$ and $\pi$. The turn of this test is less than 2 rad, so the result is the turn itself.

The next excerpt shows the type `Headings` and how a `HeadingObserver` starts, from [`test_ekf.py`, lines 106–134]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf.py#L106-L134):

```python title="test_ekf.py: Headings and HeadingObserver.__init__()" linenums="106"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf.py:106:134"
```

- **`Headings`:** the robot's heading at one moment by each source, in rad. `true` is from Gazebo, `odometry` from the drive controller's wheel odometry and `ekf` from the EKF's estimate.
- **`true_heading`, `odometry_heading` and `ekf_heading`:** the observer's latest heading of each source. Each is `None` until the first message of its source arrives.
- **`sample_count`:** the number of samples that the IMU sample validator forwarded since the observer was created.
- **The four subscribers:** they read the drive controller's wheel odometry (`WHEEL_ODOMETRY_TOPIC`), the EKF's estimate (`FILTERED_TOPIC`), Gazebo's model states (`MODEL_STATES_TOPIC`), which carry the robot's true pose, and the validator's output (`VALIDATED_IMU_TOPIC`). The subscriber of the model states keeps one message, `MODEL_STATES_QUEUE_SIZE`, because only the latest true pose matters. The subscriber of the validated samples keeps up to 100 messages, `IMU_QUEUE_SIZE`, because the test counts every sample.
- **`lock`:** rospy calls each callback in a thread of its own, and the test reads the observer in another thread. The lock lets one thread at a time read or change the observer's values. So the test gets the three headings as one consistent copy, and no callback changes a heading while the test copies them. The three headings are the latest of each source, not headings of the same instant.

Each callback stores what the test needs from one message. The excerpt shows the four callbacks, from [`test_ekf.py`, lines 136–155]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf.py#L136-L155):

```python title="test_ekf.py: the callbacks of HeadingObserver" linenums="136"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf.py:136:155"
```

- **`on_wheel_odometry()` and `on_filtered_odometry()`:** both topics carry `nav_msgs/Odometry` messages. Each callback stores the heading of the message's pose, `yaw(message.pose.pose)`.
- **`on_model_states()`:** Gazebo publishes the poses of all models in one message. The callback takes the pose of the model named `MODEL_NAME`. Gazebo's model of the robot is named `diffbot` for DiffBot and for Remo.
- **`on_validated_imu()`:** counts a sample that the IMU sample validator forwarded.
- **`with self.lock`:** each callback holds the lock while it changes its value.

The test reads the observer through three methods. The excerpt shows them, from [`test_ekf.py`, lines 157–175]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf.py#L157-L175):

```python title="test_ekf.py: latest(), silent_sources() and validated_sample_count()" linenums="157"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf.py:157:175"
```

- **`latest()`:** returns the latest heading of each source as one `Headings`. It returns `None` until each of the three sources has sent a heading.
- **`silent_sources()`:** returns the names of the sources that haven't sent a heading yet, out of `Gazebo`, `odometry` and `EKF`. `setUpClass()` puts them into its failure message.
- **`validated_sample_count()`:** returns the number of validated IMU samples so far.

## Waiting until everything runs

unittest calls `setUpClass()` once, before the first test of the class. `setUpClass()` creates the test's node, a `HeadingObserver`, the observer of the topic `/tf` and the publisher of the velocity commands. Then it waits until `ready()` returns true. The excerpt shows both methods, from [`test_ekf.py`, lines 185–204]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf.py#L185-L204):

```python title="test_ekf.py: setUpClass() and ready()" linenums="185"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf.py:185:204"
```

- **`cls.heading_observer`:** from its creation on, the `HeadingObserver` keeps the latest headings and counts the validated IMU samples.
- **`cls.odom_tf_observer`:** `OdomTfObserver()` starts to count who publishes the transform `odom → base_footprint`, for the second test. [The Transform's Publisher](odom-tf.md#the-observer) explains this observer.
- **`cls.command`:** publishes the velocity commands on `CMD_VEL_TOPIC`, the drive controller's command topic.
- **`ready()`:** is true when `latest()` returns headings, so a heading arrived from each of the three sources, and the drive controller has connected to the command topic. ROS loses a command that is published before this connection exists.
- **`STARTUP_TIMEOUT`:** after 120 s of wall time, `setUpClass()` raises an `AssertionError`. Its message names the sources that sent no heading, from `silent_sources()`, and the number of subscribers to the commands.

## The turn

`measure_turn()` turns the robot on the spot and returns what each source saw. It reads the headings with `latest_headings()`. The excerpt shows both methods, from [`test_ekf.py`, lines 206–233]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf.py#L206-L233):

```python title="test_ekf.py: latest_headings() and measure_turn()" linenums="206"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf.py:206:233"
```

`latest_headings()` returns what `latest()` of the `HeadingObserver` returns. Its `assert` stops the test if that is `None`. After `setUpClass()`, which waited for a heading of each source, `latest()` returns a `Headings`.

1. `measure_turn()` reads the three headings with `latest_headings()` into `start`, notes the count of validated IMU samples and reads the simulation time.
2. For `DURATION`, 3 s of simulation time, it publishes a `Twist` command with `angular.z` set to `TURN_VELOCITY`, 0.5 rad/s, at `COMMAND_RATE`, 20 times per second. A positive yaw rate is a left turn. The command has no forward speed, so the robot turns on the spot.
3. It computes the rate of the validated IMU samples: the number of samples that `validated_sample_count()` counted during the turn, divided by the time that the turn took.
4. In the `finally` block, it publishes a command of zero. The robot stops also when an error ends the loop early.
5. It waits `SETTLE`, 1 s of simulation time, until the robot stands.
6. It reads the three headings again into `stopped` and returns a `TurnMeasurement`. For each source, `turned()` gives the turn from the heading in `start` to the heading in `stopped`, for example `turned(start.true, stopped.true)`.

The excerpt shows the result's type, from [`test_ekf.py`, lines 72–83]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf.py#L72-L83):

```python title="test_ekf.py: TurnMeasurement" linenums="72"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf.py:72:83"
```

`report()` returns the three turns as one line of text. The test prints this line and puts it into the message of a failed check.

## The checks on the turn

The excerpt shows the test method, from [`test_ekf.py`, lines 271–281]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf.py#L271-L281). The method first reads the controller's wheel separation from the parameter server and checks it, so a wrong setup fails before the robot is commanded to turn. Then the method measures one turn and makes the other three assertions:

```python title="test_ekf.py: the turn test" linenums="271"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf.py:271:281"
```

The next excerpt shows the four assertion methods, from [`test_ekf.py`, lines 235–259]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf.py#L235-L259):

```python title="test_ekf.py: the four assertions" linenums="235"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf.py:235:259"
```

In the list, $\Delta\theta_\text{true}$ is the robot's true turn, $\Delta\theta_\text{odom}$ the wheel odometry's turn and $\Delta\theta_\text{EKF}$ the turn of the EKF's estimate, as in [the figure](#the-turn-from-above).

1. **The wheel separation is the scaled one**, `assert_wheel_separation_is_scaled()`: the parameter `/diffbot/mobile_base_controller/wheel_separation` is `SEPARATION_SCALE` times the `wheel_separation` in the robot's `geometry.yaml`, within `SEPARATION_TOLERANCE`, 10⁻⁶ m. This check shows that the rostest file's value reached the controller's parameter and still fits the robot's geometry.
2. **The validator forwards the IMU**, `assert_validated_imu_rate()`: during the turn, the samples on the topic `/diffbot/imu/data_validated` arrived at more than `MIN_IMU_RATE_SHARE` times `IMU_RATE`, 0.9 · 50 Hz = 45 Hz. Without the validated samples, the EKF would have only the wheel odometry.
3. **The wheel odometry falls behind**, `assert_odometry_falls_behind()`: $\Delta\theta_\text{true} - \Delta\theta_\text{odom}$ is more than `MIN_ODOMETRY_ERROR`, 0.15 rad. This check shows that the error that the test needs is there. With the scaled separation, the wheel odometry reports 1 / 1.25 of the true turn, so the difference is a fifth of the true turn.
4. **The EKF follows the true turn**, `assert_ekf_follows_the_true_turn()`: $|\Delta\theta_\text{EKF} - \Delta\theta_\text{true}|$ is at most `EKF_TOLERANCE`, 0.05 rad.

`assert_wheel_separation_is_scaled()` gets the robot's own value from `nominal_wheel_separation()`. The excerpt shows this function, from [`test_ekf.py`, lines 97–103]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf.py#L97-L103):

```python title="test_ekf.py: nominal_wheel_separation()" linenums="97"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf.py:97:103"
```

The table lists what the test measured in a run in the dev container's image, at the commit `3ef4a91` of [diffbot#154](https://github.com/ros-mobile-robots/diffbot/pull/154):

| Simulated robot | Controller's wheel separation | True turn | Wheel odometry's turn | EKF's turn | Validated IMU |
|:--|:--|:--|:--|:--|:--|
| DiffBot | 0.1725 m | 1.849 rad | 1.479 rad | 1.844 rad | 49.7 Hz |
| Remo | 0.18 m | 1.873 rad | 1.499 rad | 1.870 rad | 50.0 Hz |

In this run, the wheel odometry fell behind the true turn by 0.370 rad for DiffBot and 0.374 rad for Remo. The EKF's turn differed from the true turn by 0.005 rad for DiffBot and 0.003 rad for Remo.

## Who publishes the transform

With the EKF on, the EKF has to publish the transform `odom → base_footprint`, and the drive controller must not. The excerpt shows the second test, from [`test_ekf.py`, lines 261–269]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf.py#L261-L269):

```python title="test_ekf.py: only the EKF publishes odom → base_footprint" linenums="261"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf.py:261:269"
```

1. The test waits until `self.odom_tf_observer` has seen a first transform `odom → base_footprint`, at most `STARTUP_TIMEOUT`.
2. It waits `OBSERVE`, 3 s of simulation time, while the observer keeps counting.
3. It expects exactly one publishing node in the observer's counts, `EKF_NODE`, which is `/diffbot/ekf_localization`.
4. It expects the controller's parameter `enable_odom_tf`, which `ODOM_TF_PARAMETER` names, to be `false`.

This observer counts from `setUpClass()` on. unittest runs a class's test methods in alphabetical order, so the turn test runs first, and the count covers the turn too. In the saved run, the observer counted 375 transforms for DiffBot and 371 for Remo, all from `/diffbot/ekf_localization`. [The Transform's Publisher](odom-tf.md) explains the observer and the same check without the EKF.

## Run it

In the [dev container](../../../../development/dev-container.md), in the built workspace:

```console
rostest diffbot_gazebo sim_ekf_diffbot.test
rostest diffbot_gazebo sim_ekf_remo.test
```

[Run it](../../../../development/simulation-test.md#run-it) and [Read the results](../../../../development/simulation-test.md#read-the-results) on Simulation Test explain rostest's options, its output and its result files. A passing run of `sim_ekf_diffbot.test` ends with:

```console
[diffbot_gazebo.rosunit-ekf_diffbot/test_ekf_follows_the_true_turn_with_a_wrong_wheel_separation][passed]
[diffbot_gazebo.rosunit-ekf_diffbot/test_ekf_is_the_only_odom_tf_publisher][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0
```

The test prints the measured turn in one line, which rostest copies into the result file `rosunit-ekf_diffbot.xml`. In the saved run, the line was:

```text
controller's wheel separation 0.1725 m; turned: true 1.849 rad, odometry 1.479 rad, EKF 1.844 rad; validated IMU 49.7 Hz
```

CI runs both files with the repository's other tests, as [Testing and CI](../../../../development/ci.md#ekf-tests) describes.

## What the test doesn't show

- **The real gyroscope:** the simulated IMU adds random noise to the true yaw rate and reports the matching variance $\sigma^2$. It has no bias and no mounting error. [The simulated IMU](../../../../diffbot_gazebo.md#inertial-measurement-unit-imu) describes the sensor.
- **The real robot's rates and delays:** in the simulation, the wheel odometry and the IMU both arrive at 50 Hz, stamped with simulation time. [Limits](../../robot_localization.md#limits) names what differs on the robot.
- **Other errors of the wheels:** the test's error is one constant factor on a turn on the spot. The test has no slip that comes and goes and no driving forward.
- **The position:** the test compares the three turns only, not the positions $x$ and $y$.
- **Turns of more than $\pi$:** `turned()` returns a change of the heading between $-\pi$ and $\pi$.
