# IMU Simulation: Tests

What CI checks on the simulated IMU, and how the tests do it. This page belongs to [IMU: Simulation](index.md).

Two test files check the simulated IMU in Gazebo, [`test_imu.py`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py) and [`test_imu_reset.py`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py), with three tests:

| Test | What it checks | Why |
|:--|:--|:--|
| [The test at rest](#the-test-at-rest), in `test_imu.py` | The message on `/diffbot/imu/data_raw` while the robot stands still: its rate, its stamps, its frame, the orientation marked unavailable, usable values, and gravity on the upward axis | The EKF and the IMU sample validator need the message this way, and the firmware publishes it this way |
| [The left turn](#the-left-turn), in `test_imu.py` | The yaw rate while the robot turns left: positive, and about as large as the commanded turn rate | The yaw rate is the one value that the EKF fuses |
| [The reset test](#the-reset-test), in `test_imu_reset.py` | The IMU publishes again right after a reset of the simulation | The IMU sample validator and the EKF start over at a reset and need samples right away |

[What the tests don't show](#what-the-tests-dont-show) lists what a pass doesn't prove.

## How the tests run

Both simulation test files, `sim_drive_diffbot.test` and `sim_drive_remo.test`, run each of the two Python files as a test node of its own: `imu_diffbot` and `imu_reset_diffbot` for DiffBot, `imu_remo` and `imu_reset_remo` for Remo. [`sim_drive_diffbot.test`, lines 28–34]({{ imu_sim_src_url }}/diffbot_gazebo/test/sim_drive_diffbot.test#L28-L34):

```xml title="sim_drive_diffbot.test: the two IMU test nodes" linenums="28"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/sim_drive_diffbot.test:28:34"
```

rostest starts Gazebo and the robot again for each of these test nodes, as for the other test nodes in [the test file on Simulation Test](../../../development/simulation-test.md#1-the-test-file). Simulation Test also shows how to [run the tests](../../../development/simulation-test.md#run-it) and [read their results](../../../development/simulation-test.md#read-the-results).

`test_imu.py` has two test methods. unittest runs a test class's methods in [alphabetical order](https://docs.python.org/3/library/unittest.html#unittest.TestLoader.sortTestMethodsUsing). So the left turn, `test_imu_measures_a_positive_yaw_rate_in_a_left_turn`, runs before the test at rest, `test_imu_publishes_the_firmware_message_at_rest`.

??? example "The constants of `test_imu.py`, lines 22–48"

    The comments say what each number is. `STARTUP_TIMEOUT` and `POLL_INTERVAL` are seconds of wall time, the computer's clock. `SETTLE`, `SPEED_UP` and `MEASURE` are seconds of simulation time, Gazebo's clock. [Wall time and simulation time](#wall-time-and-simulation-time) explains the difference.

    ```python title="test_imu.py: the constants" linenums="22"
    --8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:22:48"
    ```

## Waiting until the robot can drive

Both tests of `test_imu.py` send velocity commands to the robot. A command that the robot doesn't receive would make the test measure a robot at rest. So before each test, `setUp()` waits until the IMU publishes and the drive controller listens. unittest calls `setUp()` before each test method. [`test_imu.py`, lines 53–65]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L53-L65):

```python title="test_imu.py: setUp()" linenums="53"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:53:65"
```

- **`List[Imu]`.** `Imu` is the Python class of the message type `sensor_msgs/Imu`, imported from `sensor_msgs.msg` in [line 20]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L20). Each IMU message arrives as an `Imu` object. `List[Imu]` is a type hint from Python's [`typing`](https://docs.python.org/3/library/typing.html) module: it says that `self.messages` is a list of `Imu` messages. Python doesn't check it when the test runs. It tells readers and tools such as mypy what the list holds.
- **The subscriber.** `rospy.Subscriber(TOPIC, Imu, self.messages.append, queue_size=QUEUE_SIZE)` subscribes to `/diffbot/imu/data_raw`. Its arguments are the topic, the message class, and the callback that rospy calls for each message. The callback is the list's `append` method, so each message is added to `self.messages`. rospy runs the callback in one of its background threads, so the list fills while the test method waits. The test keeps no variable for the subscriber: rospy keeps the subscription until the node shuts down.
- **The publisher.** `rospy.Publisher(COMMAND, Twist, queue_size=COMMAND_QUEUE_SIZE)` creates a publisher for `geometry_msgs/Twist` velocity commands on the drive controller's topic. `self.command_publisher` keeps it for the test methods.
- **The first IMU message.** `rospy.wait_for_message()` waits for it. It comes early: Gazebo's sensor and the adapter publish soon after the robot is in the simulation.
- **An IMU message doesn't mean that the robot can drive.** The controller spawner starts the drive controller, `mobile_base_controller`, later, as [the launch file on Simulation Test](../../../development/simulation-test.md#2-the-launch-file) shows. So `setUp()` waits for the controller's first odometry message, which shows that the controller runs.
- **A running controller can still miss the first commands.** ROS connects a publisher and a subscriber only after both exist, and a message published before the connection is lost. `get_num_connections()` counts the subscribers connected to the test's publisher. The loop waits until the drive controller has connected, at most `STARTUP_TIMEOUT`, 60 s.

## The test at rest

`test_imu_publishes_the_firmware_message_at_rest` checks that the message on `/diffbot/imu/data_raw` is what its readers need. The EKF runs at 50 Hz, the firmware's rate. The IMU sample validator forwards only samples with the frame `imu_link`, rising stamps, finite values and a positive variance. And the firmware's message has no orientation. The robot stands still during this test, because then the reading on the upward axis is known: gravity alone.

The test uses two helpers, which the left turn uses too. [`test_imu.py`, lines 67–82]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L67-L82):

```python title="test_imu.py: send_command() and collect_messages()" linenums="67"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:67:82"
```

- **`send_command()`** publishes a velocity command for `seconds` of simulation time, 20 times per second (`COMMAND_RATE`). One command isn't enough: the drive controller stops the robot when no command arrives for 0.5 s.
- **`collect_messages()`** returns the IMU's messages of the next `seconds`. It empties the list, sends the command for that time while the subscriber fills the list, and returns a copy of the list.

The test starts like this, [`test_imu.py`, lines 84–87]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L84-L87):

```python title="test_imu.py: stand still, then collect" linenums="84"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:84:87"
```

`Twist()` is a command with all velocities 0. The test sends it for `SETTLE`, 1 s, because the robot may still be braking after the left turn. Then the test collects $T$ = `MEASURE` = 2 s of messages. The checks follow, each with its lines.

**The rate.** [`test_imu.py`, lines 88–91]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L88-L91):

```python title="test_imu.py: the rate" linenums="88"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:88:91"
```

With $N$ messages in $T$ = 2 s, the check is $N / T > 0.9 \cdot 50 \text{ Hz} = 45 \text{ Hz}$. The firmware publishes at 50 Hz, and the EKF runs at that rate.

**The stamps.** [`test_imu.py`, lines 92–94]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L92-L94):

```python title="test_imu.py: the stamps" linenums="92"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:92:94"
```

Each stamp must be later than the one before. The IMU sample validator drops a sample that isn't later than the last one it forwarded.

**The frame and the orientation.** For each message, [`test_imu.py`, lines 95–98]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L95-L98):

```python title="test_imu.py: the frame and the orientation" linenums="95"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:95:98"
```

- The frame must be `imu_link`. The EKF looks up this frame to convert the yaw rate into the robot's axes.
- `orientation_covariance[0]` must be −1: the orientation is marked unavailable, as in the firmware's message. [The adapter](measurement-to-message.md#the-adapter) sets it.

**The values and the yaw rate's variance.** In the same loop, [`test_imu.py`, lines 99–104]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L99-L104):

```python title="test_imu.py: the values and the variance" linenums="99"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:99:104"
```

- `angular_velocity.z` and `linear_acceleration.z` must be finite: not NaN, not infinite. The IMU sample validator drops samples with values that aren't finite.
- `angular_velocity_covariance[8]` must be positive and finite. Element 8 is the yaw rate's variance, $\sigma^2 = 2.5 \cdot 10^{-5}$ (rad/s)² in the simulation, as [how the message stores a covariance](index.md#how-the-message-stores-a-covariance) and [noise and covariance](measurement-to-message.md#noise-and-covariance) show. The EKF weighs the yaw rate with it, and the validator drops samples without a positive, finite variance.

**Gravity.** [`test_imu.py`, lines 105–109]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L105-L109):

```python title="test_imu.py: gravity" linenums="105"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:105:109"
```

The check averages the upward axis's $N$ readings, $\bar f_z = \frac{1}{N} \sum_{i=1}^{N} f_{z,i}$, and needs

$$
|\bar f_z - 9.81 \text{ m/s}^2| \le 0.5 \text{ m/s}^2
$$

At rest, the upward axis reads gravity, $f_z = +9.8$ m/s² in Gazebo, as [the linear acceleration](index.md#the-linear-acceleration) explains. An IMU frame that is upside down reads about −9.8 m/s², and a plugin that leaves gravity out reads about 0.

## The left turn

`test_imu_measures_a_positive_yaw_rate_in_a_left_turn` checks the yaw rate, the one value that the EKF fuses, while the robot turns. At rest, the yaw rate is 0 however the IMU is mounted. Only a turn shows that the IMU measures the turn, and with the right sign. [`test_imu.py`, lines 111–121]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L111-L121):

```python title="test_imu.py: the left turn" linenums="111"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:111:121"
```

- The test commands a turn on the spot at `TURN_RATE`, 0.5 rad/s. [`send_command()`](#the-test-at-rest) sends the command for `SPEED_UP`, 1 s, so that the robot reaches the turn rate.
- [`collect_messages()`](#the-test-at-rest) keeps the robot turning for `MEASURE`, 2 s, and returns the messages of that time. Then the test stops the robot with `Twist()`.
- The check averages the yaw rates, $\bar\omega_z$, and needs $|\bar\omega_z - 0.5 \text{ rad/s}| \le 0.15 \text{ rad/s}$. A left turn, counterclockwise seen from above, is positive about the upward z axis, as [the angular velocity](index.md#the-angular-velocity) shows. So the check fails when the IMU's frame is upside down, or when the IMU doesn't measure the turn.

## The reset test

`test_imu_reset.py` checks that the IMU publishes again right after a reset of the simulation. A reset sets Gazebo's clock back to zero. A plugin that doesn't notice this stays silent for as long as the simulation ran before the reset, and the EKF gets no yaw rate in that time. [Publishing again after a reset](plugin.md#publishing-again-after-a-reset) explains how the robots' plugin handles a clock that went back.

The test's constants, [`test_imu_reset.py`, lines 32–46]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L32-L46):

```python title="test_imu_reset.py: the constants" linenums="32" hl_lines="5 6"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu_reset.py:32:46"
```

`RUN_BEFORE_RESET` and `RECOVERY`, the highlighted lines, are seconds of simulation time. The test compares them with the samples' stamps. The three timeouts and `POLL_INTERVAL` are seconds of wall time.

`setUp()` subscribes to `/diffbot/imu/data_raw` with a callback that notes each sample's stamp in the list `self.stamps`. Then `setUp()` waits for Gazebo's reset service, in [lines 51–60]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L51-L60). The test method, [`test_imu_reset.py`, lines 89–96]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L89-L96):

```python title="test_imu_reset.py: run, reset, wait for fresh samples" linenums="89" hl_lines="7"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu_reset.py:89:96"
```

1. **Run.** The test waits until the latest stamp has reached `RUN_BEFORE_RESET`, 3 s. So the IMU has published for 3 s of simulation time, and `last_time` in the plugin is about 3 s.
2. **Reset.** The test notes how many samples have arrived, and calls `/gazebo/reset_simulation`, the highlighted line. Gazebo's clock starts from zero again.
3. **Wait for fresh samples.** The test waits until `FRESH_SAMPLE_COUNT`, 10, fresh samples have arrived, at most `RECOVERY_TIMEOUT`, 30 s of wall time. The test fails when the samples don't arrive in that time. The failure message, in [lines 104–111]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L104-L111), names the stamps of the first samples after the reset.

Which samples count as fresh, [`test_imu_reset.py`, lines 66–73]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L66-L73):

```python title="test_imu_reset.py: fresh_stamps()" linenums="66"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu_reset.py:66:73"
```

A fresh sample arrived after the test called the reset, and its stamp is earlier than `RECOVERY`, 1 s. Such a stamp can only come from the new simulation time: before the reset, the stamps had reached 3 s. With the reset handled, the first samples after the reset are stamped 0.02 s, 0.04 s and 0.06 s, and ten of them arrive within 0.2 s of simulation time. A plugin that keeps the old time publishes its next sample only after the clock has passed 3 s again. Then no sample is stamped earlier than 1 s, and the test fails.

## Wall time and simulation time

The tests wait on two clocks:

- **Wall time** is the computer's clock, the time that a clock on the wall shows.
- **Simulation time** is Gazebo's clock. It starts at 0 when the simulation starts and moves on by 1 ms with every simulation step. It stands still while Gazebo starts, and for good when the simulation stalls.

**Where the two differ.** The two clocks don't run at the same speed. Their ratio is Gazebo's real time factor: 1 when a second of simulation takes a second of wall time, and less than 1 when the computer is too slow for that, as Gazebo's tutorial on [physics parameters](https://classic.gazebosim.org/tutorials?tut=physics_params&cat=physics) explains. The test's world asks for real time, with 1000 steps of 1 ms per second in [lines 14–16 of `diffbot_empty.world`]({{ imu_sim_src_url }}/diffbot_gazebo/worlds/diffbot_empty.world#L14-L16). A busy computer, such as a CI runner, reaches less. So 2 s of simulation time can take 2 s of wall time, or 5 s.

**Which clock a ROS node reads.** A ROS node reads the computer's clock, unless the parameter `use_sim_time` is set. Then `rospy.Time.now()`, `rospy.sleep()` and `rospy.Rate` follow Gazebo's clock, which Gazebo publishes on the topic `/clock`. The ROS wiki's page on the [clock](http://wiki.ros.org/Clock) describes this. The simulation's launch file sets `use_sim_time`, in [lines 14 and 30 of `diffbot_view.launch`]({{ imu_sim_src_url }}/diffbot_gazebo/launch/diffbot_view.launch#L14-L30). Python's own `time.monotonic()` and `time.sleep()` always count wall time.

Not every rospy function follows `use_sim_time`. `rospy.wait_for_message()` counts its `timeout` on the computer's clock, with `time.time()`. rospy's lines, [`client.py` in ros_comm 1.17.4, lines 413–420]({{ ros_comm_src_url }}/clients/rospy/src/rospy/client.py#L413-L420), under the BSD license:

```python title="client.py (rospy 1.17.4): the timeout of wait_for_message()" linenums="413" hl_lines="4 7"
--8<-- "{{ ros_comm_src_raw }}/clients/rospy/src/rospy/client.py:413:420"
```

`rospy.wait_for_service()` counts its `timeout` the same way, in [lines 128–130 of `tcpros_service.py`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/impl/tcpros_service.py#L128-L130).

**Which clock each wait of the tests counts:**

| Wait | Lines | Clock | Why this clock |
|:--|:--|:--|:--|
| `test_imu.py`: the first IMU message and the first odometry message, each at most `STARTUP_TIMEOUT`, 60 s | [57 and 60]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L57-L60) | Wall time | While Gazebo starts, the simulation's clock stands still. A limit in simulation time would never run out |
| `test_imu.py`: the drive controller's connection, at most `STARTUP_TIMEOUT`, 60 s | [61–65]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L61-L65) | Wall time, with `time.monotonic()` and `time.sleep()` | The same reason |
| `test_imu.py`: `send_command()` for `SETTLE`, `SPEED_UP` and `MEASURE`, 1 s, 1 s and 2 s | [67–73]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L67-L73) | Simulation time, with `rospy.Time.now()` and `rospy.Rate` | The robot's motion and the number of IMU messages depend on simulated seconds. `MEASURE` always covers 2 s of motion and about 100 IMU messages, however fast the computer runs the simulation |
| `test_imu_reset.py`: Gazebo's reset service, at most `STARTUP_TIMEOUT`, 60 s | [55]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L55) | Wall time | While Gazebo starts, the simulation's clock stands still |
| `test_imu_reset.py`: `wait_for()` with `RUN_TIMEOUT`, 60 s, and `RECOVERY_TIMEOUT`, 30 s | [79–87]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L79-L87) | Wall time, with `time.monotonic()` and `time.sleep()` | The test sets the simulation's clock back on purpose. `rospy.sleep()` raises an exception when the clock goes back while it waits, in [lines 161–163 of rospy's `timer.py`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/timer.py#L161-L163) |
| `test_imu_reset.py`: `RUN_BEFORE_RESET`, 3 s, and `RECOVERY`, 1 s | [64 and 73]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L64-L73) | Simulation time, read from the samples' stamps | They say how long the IMU ran before the reset and how early it came back, in the IMU's own time. The test reads no clock for them |
| Both: rostest's `time-limit`, 120 s for `test_imu.py` and 180 s for `test_imu_reset.py` | [30 and 34 of `sim_drive_diffbot.test`]({{ imu_sim_src_url }}/diffbot_gazebo/test/sim_drive_diffbot.test#L30-L34) | Wall time | It stops a test node that hangs |

**When a wait in wall time runs out** in `test_imu.py`: without an IMU message or an odometry message in time, `rospy.wait_for_message()` raises an exception, and the test reports an error. When the drive controller doesn't connect in time, `assertLess()` fails, and the test reports a failure. In `test_imu_reset.py`, the three timeouts only limit how long a failing run takes.

## What the tests don't show

Each item says what isn't tested, why not, and what a pass therefore doesn't prove.

- **That the simulation can't stall while the controllers start.** The stall of gazebo_plugins' original plugin depends on timing: it needs a request of the controller spawner in the callback queue at the moment that the plugin spins, as [what goes wrong with the original plugin](plugin.md#what-goes-wrong-with-the-original-plugin) explains. Most starts pass, with the original too, and each test node starts the simulation once. So a pass of `test_imu.py` doesn't show that the deadlock is gone. The counts of many starts, with the original and with the copy, are in [diffbot#185](https://github.com/ros-mobile-robots/diffbot/issues/185). If a start did stall, `setUp()` would end with an error, because no odometry comes. A longer `STARTUP_TIMEOUT` wouldn't help, because a stopped clock doesn't start again.
- **Which way the IMU's x and y axes point.** `test_imu.py` checks only the upward axis: the yaw rate and the reading of gravity. A rotation of the IMU about that axis changes neither. So a pass doesn't show that the model's `imu_rpy` is right. Without the −90° of Remo's mounting, the test would still pass.
- **The forward and the sideways acceleration.** No test drives the robot forward and compares the reading with the robot's acceleration. The EKF doesn't fuse the acceleration, so nothing in the simulation depends on the acceleration yet. A pass says nothing about `linear_acceleration.x` and `linear_acceleration.y`.
- **The real BNO055.** The simulated noise is a setting, 0.005, and the simulated IMU has no bias and no delay. So a pass says nothing about the real sensor's noise, bias and mounting. The [bench checks](../driver-bno055.md#bench-checks) measure those on the robot.
- **The rest of the simulation after a reset.** `test_imu_reset.py` checks the IMU's topic after one reset, 3 s after the start. A pass doesn't show that the other publishers come back at once. The drive controller's odometry and the joint states don't, as [the EKF's limits](../robot_localization.md#limits) on the Localization page explain.
- **The EKF.** These tests read `/diffbot/imu/data_raw`, the validator's input. Whether the validator forwards the samples and the EKF uses them is the subject of the [EKF's tests](../robot_localization.md#how-its-tested) on the Localization page.
