# IMU Simulation: Tests

In the Gazebo simulation, DiffBot and Remo have a simulated IMU. The simulated IMU publishes on the same topic and with the same message type as the IMU on the real robot. CI checks the simulated IMU with three tests in Gazebo. This page shows what the tests check, how to run the tests yourself and what a pass proves. Read the page before you change where the simulated IMU sits in a robot's URDF model or the code that makes the IMU's message. This page belongs to [IMU: Simulation](index.md), which explains that message.

Two Python files check the simulated IMU in Gazebo, [`test_imu.py`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py) and [`test_imu_reset.py`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py), with three tests. Each test reads the topic [`/diffbot/imu/data_raw`](index.md#what-the-simulated-imu-publishes):

1. **[The test at rest](#the-test-at-rest)**, in `test_imu.py`: the robot stands still.
    - **A pass shows:** the message has the form that its readers need. More than 90 messages arrive in 2 s of simulation time, each stamp is later than the one before, the frame is [`imu_link`](position-and-axes.md#the-two-frames-and-their-fixed-joint), the orientation $\mathbf q$ is marked unavailable, the yaw rate $\omega_z$ and the upward reading $f_z$ are finite, the yaw rate's variance is positive, the mean of $f_z$ lies within 0.5 m/s² of 9.81 m/s², which is gravity, and the mean of $\omega_z$ lies within 0.05 rad/s of 0 rad/s.
    - **Why:** the IMU sample validator and the EKF need the message this way, and the firmware publishes it this way.
2. **[The left turn](#the-left-turn)**, in `test_imu.py`: the robot turns left on the spot at 0.5 rad/s.
    - **A pass shows:** more than 90 messages arrive in 2 s of the turn, and the mean of their yaw rate $\omega_z$ lies within 0.15 rad/s of the commanded 0.5 rad/s, nominally between 0.35 and 0.65 rad/s.
    - **Why:** the yaw rate is the one value that the EKF fuses, and its sign has to be that of the turn.
3. **[The reset test](#the-reset-test)**, in `test_imu_reset.py`: the test resets the simulation after about 3 s.
    - **A pass shows:** after the test saved its count of samples, just before it calls the reset, at least ten samples arrive whose stamps advance: each later than 0 s, later than the one that counted before it, and below 1 s.
    - **Why:** the IMU sample validator and the EKF start over at a reset and need samples right away.

[What the tests don't show](#what-the-tests-dont-show) lists what a pass doesn't prove. A yaw rate that stays at one value fails one of the two tests of `test_imu.py`: about zero fails the left turn, and about the turn rate fails at rest ([what a pass of the left turn shows](#what-a-pass-of-the-left-turn-shows)).

## Run the tests

You need the [dev container](../../../development/dev-container.md) with its workspace `~/catkin_ws` built and sourced, as the container's setup leaves it. In a terminal of the dev container, each of the two commands runs one robot's simulation test file, which contains the IMU tests:

```console
rostest diffbot_gazebo sim_drive_diffbot.test
rostest diffbot_gazebo sim_drive_remo.test
```

Each command takes about two minutes on our test PC, because [rostest](#how-rostest-runs-the-imu-tests) starts Gazebo again for every test node. A passing run of the first command ends with these lines, with the diffbot repository at `cd41ead`. The commit that this page shows, `8c83f53`, runs the same IMU tests with stricter checks:

```text hl_lines="3 4 5"
[diffbot_gazebo.rosunit-drive_diffbot/test_forward][passed]
[diffbot_gazebo.rosunit-drive_diffbot/test_turn][passed]
[diffbot_gazebo.rosunit-imu_diffbot/test_imu_measures_a_positive_yaw_rate_in_a_left_turn][passed]
[diffbot_gazebo.rosunit-imu_diffbot/test_imu_publishes_the_firmware_message_at_rest][passed]
[diffbot_gazebo.rosunit-imu_reset_diffbot/test_imu_publishes_again_right_after_a_simulation_reset][passed]
[diffbot_gazebo.rosunit-publish_diffbot/test_publish][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 6
 * ERRORS: 0
 * FAILURES: 0
```

The three highlighted lines are the IMU's tests. At `8c83f53`, the test file also runs the test node [`odom_tf_diffbot`](../robot_localization/tests/odom-tf.md#where-the-test-runs), so the output has one more line and counts 7 tests, as [read the results](../../../development/simulation-test.md#read-the-results) on Simulation Test explains. What to expect from each of the IMU's tests:

- **The left turn** passes when more than 90 messages arrive in 2 s and their mean yaw rate lies within 0.15 rad/s of 0.5 rad/s. A failure of the mean prints the measured mean. With a yaw rate that stays at 0 rad/s, for example, the message is `0.0 != 0.5 within 0.15 delta (0.5 difference) : turning left at 0.5 rad/s, the IMU measured 0.000 rad/s about z`.
- **The test at rest** passes without output. A failure names the check that failed and the value that it found.
- **The reset test** prints one line, also when it passes. rostest keeps the line in the test node's result file. The next command prints the line of DiffBot's last run:

    ```console
    grep -o 'last stamp before the reset[^<&]*' ~/.ros/test_results/diffbot_gazebo/rosunit-imu_reset_diffbot.xml
    ```

    ```text
    last stamp before the reset 3.000 s; after the reset 10 samples, first stamps ['0.020', '0.040', '0.060'], 10 of them with advancing stamps earlier than 1.0 s
    ```

    Remo's result file is `rosunit-imu_reset_remo.xml`. The numbers differ a little from run to run: a run with Remo printed 3.019 s, with the same first stamps. [The reset test](#the-reset-test) explains the numbers.

Simulation Test also shows how to [run all tests as CI does](../../../development/simulation-test.md#run-it) and how to [read the results](../../../development/simulation-test.md#read-the-results).

### How rostest runs the IMU tests

Both simulation test files, [`sim_drive_diffbot.test`]({{ imu_sim_src_url }}/diffbot_gazebo/test/sim_drive_diffbot.test#L34-L40) and [`sim_drive_remo.test`]({{ imu_sim_src_url }}/diffbot_gazebo/test/sim_drive_remo.test#L34-L40), run each of the two Python files as a test node of its own: [`imu_diffbot`]({{ imu_sim_src_url }}/diffbot_gazebo/test/sim_drive_diffbot.test#L36) and [`imu_reset_diffbot`]({{ imu_sim_src_url }}/diffbot_gazebo/test/sim_drive_diffbot.test#L40) for DiffBot, [`imu_remo`]({{ imu_sim_src_url }}/diffbot_gazebo/test/sim_drive_remo.test#L36) and [`imu_reset_remo`]({{ imu_sim_src_url }}/diffbot_gazebo/test/sim_drive_remo.test#L40) for Remo. The excerpt shows DiffBot's two test nodes, from [`sim_drive_diffbot.test`, lines 34–40]({{ imu_sim_src_url }}/diffbot_gazebo/test/sim_drive_diffbot.test#L34-L40):

```xml title="sim_drive_diffbot.test: the two IMU test nodes" linenums="34"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/sim_drive_diffbot.test:34:40"
```

rostest starts Gazebo and the robot again for each of these test nodes, as for the other test nodes in [the test file on Simulation Test](../../../development/simulation-test.md#1-the-test-file). So the reset of [`test_imu_reset.py`](#the-reset-test) doesn't reach the other tests.

[`test_imu.py`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py) has two test methods. [unittest](https://docs.python.org/3.8/library/unittest.html) runs a test class's methods in [alphabetical order](https://docs.python.org/3.8/library/unittest.html#unittest.TestLoader.sortTestMethodsUsing). So the left turn, [`test_imu_measures_a_positive_yaw_rate_in_a_left_turn`](#the-left-turn), runs before the test at rest, [`test_imu_publishes_the_firmware_message_at_rest`](#the-test-at-rest). This page explains the test at rest first, because its checks are the simpler ones.

Before each of the two test methods, [`setUp()`](#waiting-until-the-robot-can-drive) waits until the IMU publishes and the drive controller listens to commands. [Waiting until the robot can drive](#waiting-until-the-robot-can-drive) in the reference part of this page explains these waits.

## The test at rest

[`test_imu_publishes_the_firmware_message_at_rest`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L100-L127) checks that the message on [`/diffbot/imu/data_raw`](index.md#what-the-simulated-imu-publishes) is what its readers need. The EKF runs at 50 Hz, the firmware's rate. The IMU sample validator forwards only samples with the frame [`imu_link`](position-and-axes.md#the-two-frames-and-their-fixed-joint), rising stamps, finite values and a positive variance. And the firmware's message has no orientation. The robot stands still during this test, because then the reading on the upward axis, $f_z$, is known: gravity alone.

The figure shows what happens during the test, from left to right over simulation time: the command that the test sends, the robot, the reading that the test checks, and the IMU's messages that the test keeps.

--8<-- "includes/imu-sim-test-at-rest.html"

**What the robot does:** the test sends the command to stand still, first for [`SETTLE`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L37), 1 s, and then for [`MEASURE`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L39), 2 s. The excerpt shows these two lines at the start of the test method, from [`test_imu.py`, lines 100–103]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L100-L103):

```python title="test_imu.py: stand still, then collect" linenums="100"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:100:103"
```

[`stand_still`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L101) is [`Twist()`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html), a command with all velocities 0. The test sends this command for `SETTLE` first, because the robot may still be braking after the left turn. Then the test collects the IMU's messages of $T$ = `MEASURE` = 2 s in the list [`messages`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L103).

The test uses two helpers for this, which the left turn uses too. The excerpt shows both helpers, from [`test_imu.py`, lines 72–87]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L72-L87):

```python title="test_imu.py: send_command() and collect_messages()" linenums="72"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:72:87"
```

- **[`send_command()`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L72-L78)** publishes a velocity command for [`seconds`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L72) of simulation time, 20 times per second ([`COMMAND_RATE`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L36)). One command isn't enough: the drive controller stops the robot when no command arrives for 0.5 s.
- **[`collect_messages()`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L80-L87)** returns the IMU's messages of the next `seconds`. It empties the list, sends the command for that time while the subscriber fills the list, and returns a copy of the list.

The checks follow, each with its lines.

**The rate:** the test passes the messages to [`assert_message_rate()`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L89-L98) first, which the left turn uses too. The excerpt shows the helper, from [`test_imu.py`, lines 89–98]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L89-L98):

```python title="test_imu.py: the rate" linenums="89"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:89:98"
```

[`rate`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L95) is $N / T$, with $N$ messages in $T$ = 2 s. The check is $N / T > 0.9 \cdot 50 \text{ Hz} = 45 \text{ Hz}$. So the test needs more than 90 messages, which is at least 91. The firmware publishes at 50 Hz, and the EKF runs at that rate. Because this check comes first, a mean is never taken over one message or none.

**The stamps:** the excerpt shows the check of the stamps, from [`test_imu.py`, lines 105–107]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L105-L107):

```python title="test_imu.py: the stamps" linenums="105"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:105:107"
```

Each stamp must be later than the one before. The IMU sample validator drops a sample that isn't later than the last one it forwarded.

**The frame and the orientation:** the excerpt shows the first two checks that the test makes for each message, from [`test_imu.py`, lines 108–111]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L108-L111):

```python title="test_imu.py: the frame and the orientation" linenums="108"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:108:111"
```

- The frame must be `imu_link`. The EKF looks up this frame to convert the yaw rate into the robot's axes.
- [`orientation_covariance[0]`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L110) must be −1: the orientation $\mathbf q$, a quaternion with four numbers, is marked unavailable, as in the firmware's message. [The adapter](measurement-to-message.md#part-3-the-adapter) sets it.

**The values and the yaw rate's variance:** the excerpt shows the other checks in the same loop, from [`test_imu.py`, lines 112–117]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L112-L117):

```python title="test_imu.py: the values and the variance" linenums="112"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:112:117"
```

- [`angular_velocity.z`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L112-L113), the yaw rate $\omega_z$, and `linear_acceleration.z`, the upward reading $f_z$, must be finite: not NaN, not infinite. The IMU sample validator drops samples with values that aren't finite. Whether the yaw rate is near 0 rad/s is the test's last check, after gravity.
- [`variance`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L115-L117), the message's `angular_velocity_covariance[8]`, must be positive and finite. Element 8 is the yaw rate's variance, $\sigma^2 = 2.5 \cdot 10^{-5}$ (rad/s)² in the simulation, as [how the message stores a covariance](index.md#how-the-message-stores-a-covariance) and [filling the covariances](measurement-to-message.md#filling-the-covariances) show. The EKF weighs the yaw rate with it, and the validator drops samples without a positive, finite variance.

**Gravity:** the excerpt shows the check of the upward reading, from [`test_imu.py`, lines 118–122]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L118-L122):

```python title="test_imu.py: gravity" linenums="118"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:118:122"
```

[`mean_upward_reading`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L119) is the average of the upward axis's $N$ readings, $\bar f_z = \frac{1}{N} \sum_{i=1}^{N} f_{z,i}$. The check needs

$$
|\bar f_z - 9.81 \text{ m/s}^2| \le 0.5 \text{ m/s}^2
$$

At rest, the upward axis reads gravity, $f_z = +9.8$ m/s² in Gazebo, as [the linear acceleration](index.md#the-linear-acceleration) explains. An IMU frame that is upside down reads about −9.8 m/s², and a plugin that leaves gravity out reads about 0 m/s².

**The yaw rate at rest:** the excerpt shows the last check, from [`test_imu.py`, lines 123–127]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L123-L127):

```python title="test_imu.py: the yaw rate at rest" linenums="123"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:123:127"
```

[`mean_yaw_rate`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L124) is the average of the $N$ yaw rates, $\bar\omega_z$. The check needs

$$
|\bar\omega_z| \le 0.05 \text{ rad/s}
$$

The tolerance is [`YAW_RATE_AT_REST_TOLERANCE`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L53), a tenth of the turn rate of [the left turn](#the-left-turn). It is far more than the mean of the simulated noise and far less than a yaw rate that stays at the turn rate. So an IMU whose yaw rate stays at 0.5 rad/s fails here, although it passes the left turn.

## The left turn

[`test_imu_measures_a_positive_yaw_rate_in_a_left_turn`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L129-L140) checks the yaw rate $\omega_z$, the one value that the EKF fuses, while the robot turns. At rest, the yaw rate is 0 rad/s however the IMU is mounted. Only a turn shows the sign of the yaw rate.

The figure shows what happens during the test: the turn command, the robot's headings that follow from the command at the start, after [`SPEED_UP`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L38) and after [`MEASURE`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L39), the yaw rate $\omega_z$ that the test checks, and the IMU's messages that the test keeps.

--8<-- "includes/imu-sim-test-left-turn.html"

The excerpt shows the whole test method, from [`test_imu.py`, lines 129–140]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L129-L140):

```python title="test_imu.py: the left turn" linenums="129"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:129:140"
```

- **What the robot does:** [`turn_left`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L130) is the command, a turn on the spot at [`TURN_RATE`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L48), 0.5 rad/s, in the command's [`angular.z`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html). [`send_command()`](#the-test-at-rest) sends `turn_left` for `SPEED_UP`, 1 s, so that the robot reaches the turn rate. Then `collect_messages()` keeps the robot turning for `MEASURE`, 2 s, and returns the IMU's messages of that time, which the test keeps in [`messages`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L133). Then the test stops the robot with `Twist()`.
- **What the test checks:** first the rate, with `assert_message_rate()`: more than 90 messages in the 2 s of `MEASURE`. Then [`mean_yaw_rate`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L137) is the average of the yaw rates of `messages`, $\bar\omega_z$. The check is [`assertAlmostEqual(mean_yaw_rate, TURN_RATE, delta=TURN_RATE_TOLERANCE)`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L138-L139), which needs $|\bar\omega_z - 0.5 \text{ rad/s}| \le 0.15 \text{ rad/s}$. So the nominal range of the mean is 0.35 to 0.65 rad/s. The comparison runs with floating-point numbers: a mean of exactly 0.35 or 0.65 rad/s fails, because the computed difference is 0.15000000000000002. A left turn, counterclockwise seen from above, is positive about the upward z axis, as [the angular velocity](index.md#the-angular-velocity) shows.

### What a pass of the left turn shows

A pass shows two things: more than 90 messages arrived in the 2 s of [`MEASURE`](#reference-how-the-tests-wait), and the mean of their yaw rates lies within 0.15 rad/s of 0.5 rad/s. So the test fails when the IMU's frame is upside down, which gives about −0.5 rad/s, when the yaw rate stays near 0 rad/s, and when one message or none arrives, with the rate's message instead of an error.

A yaw rate that stays at one value fails one of the two tests of `test_imu.py`: about 0 rad/s fails the left turn, and about 0.5 rad/s fails [the test at rest](#the-test-at-rest). Neither test alone shows that the yaw rate follows the robot's motion, and together they show it only at rest and in this one turn.

## The reset test

[`test_imu_reset.py`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py) checks that the IMU publishes again right after a reset of the simulation. A reset sets Gazebo's clock back to zero. A plugin that doesn't notice this stays silent for as long as the simulation ran before the reset, and the EKF gets no yaw rate in that time. [Publishing again after a reset](plugin.md#publishing-again-after-a-reset) explains how the robots' plugin handles a clock that went back.

The figure shows the stamps of the IMU's samples during the test, with the robots' IMU plugin and with a plugin that keeps the time from before the reset. The figure is an idealized example: the reset comes at exactly 3 s, and the IMU publishes exactly 50 samples per second. In a run, the reset comes a little later, for example at 3.019 s. The robot stands still in this test. What changes is the simulation's clock.

--8<-- "includes/imu-sim-test-reset.html"

The excerpt shows the test's constants, from [`test_imu_reset.py`, lines 35–50]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L35-L50):

```python title="test_imu_reset.py: the constants" linenums="35" hl_lines="5 6"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu_reset.py:35:50"
```

[`RUN_BEFORE_RESET`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L39) and [`RECOVERY`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L40), the highlighted lines, are seconds of simulation time. The test compares them with the samples' stamps. The three timeouts and [`POLL_INTERVAL`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L50) are seconds of wall time.

[`setUp()`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L55-L60) subscribes to [`/diffbot/imu/data_raw`](index.md#what-the-simulated-imu-publishes) with the callback [`note_stamp()`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L62-L64), which notes each sample's stamp in the list [`self.stamps`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L56). Then `setUp()` waits for Gazebo's reset service.

??? example "`setUp()`, `note_stamp()` and `imu_ran_before_reset()`"

    The excerpt shows how the test collects the stamps and how it decides that the IMU has run long enough, from [`test_imu_reset.py`, lines 55–68]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L55-L68). [`self.stamps[-1]`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L68) is the stamp of the sample that arrived last:

    ```python title="test_imu_reset.py: setUp(), note_stamp() and imu_ran_before_reset()" linenums="55"
    --8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu_reset.py:55:68"
    ```

The next excerpt shows the start of the test method, from [`test_imu_reset.py`, lines 100–107]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L100-L107):

```python title="test_imu_reset.py: run, reset, wait for fresh samples" linenums="100" hl_lines="7"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu_reset.py:100:107"
```

1. **Run:** the test waits until the latest stamp is at least `RUN_BEFORE_RESET`, 3 s. The test looks at the stamps every `POLL_INTERVAL`, 0.02 s of wall time, so the last stamp before the reset is 3 s or a little later. In the two runs of [Run the tests](#run-the-tests), the last stamp was 3.000 s and 3.019 s. [`last_time`](measurement-to-message.md#the-checks-before-a-call-publishes) in the plugin then holds about that time.
2. **Reset:** the test notes how many samples have arrived, in [`sample_count_at_reset`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L104), and calls [`/gazebo/reset_simulation`](plugin.md#publishing-again-after-a-reset), the highlighted line. Gazebo's clock starts from zero again.
3. **Wait for fresh samples:** the test waits until [`FRESH_SAMPLE_COUNT`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L44), 10, fresh samples have arrived, at most [`RECOVERY_TIMEOUT`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L49), 30 s of wall time. The test fails when the samples don't arrive in that time.

### Which samples count as fresh

The excerpt shows which samples count as fresh, from [`test_imu_reset.py`, lines 70–88]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L70-L88):

```python title="test_imu_reset.py: fresh_stamps() and imu_recovered()" linenums="70"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu_reset.py:70:88"
```

[`fresh_stamps()`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L70-L84) goes through the stamps that arrived after the test noted [`sample_count_at_reset`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L104), in the order of their arrival. A stamp counts when it is later than 0 s, later than [`latest_fresh_stamp`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L79), the last stamp that counted, and below [`RECOVERY`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L40), 1 s. [`imu_recovered()`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L86-L88) is true when ten stamps count. So a pass shows this much: after the test saved the count, at least ten samples arrived whose stamps advance and lie below 1 s. Such a stamp belongs to the new simulation time, because the stamps had reached 3 s before the reset.

- **With the robots' plugin**, the first sample after the reset is stamped about 0.02 s, one period after the clock went back. Ten fresh samples then take about 0.2 s of simulation time. The test doesn't require these two values: it accepts any ten advancing stamps below 1 s.
- **With a plugin that keeps the old time**, the next sample comes only after the clock has passed 3 s again. Then no sample is stamped below 1 s, and the test fails.
- **Stamps that don't advance:** a stamp of 0 s, a stamp that repeats the last one that counted, and a stamp that goes back don't count. So ten samples with one and the same stamp give one fresh sample at most, and the test fails. The IMU sample validator would forward one of them at most too. A sample that the IMU published before the reset and that arrives after it has a stamp of 3 s or later, so it doesn't count either.
- **A repeat isn't a failure of its own:** the test fails only when ten advancing stamps below 1 s don't arrive within `RECOVERY_TIMEOUT`. A stream with a repeated stamp among ten good ones passes.

After the wait, the test prints one line and then checks the result.

??? example "The printed line and the failure message"

    The excerpt shows the end of the test method, from [`test_imu_reset.py`, lines 108–122]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L108-L122). [`print()`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L110) writes the line that [Run the tests](#run-the-tests) shows: the last stamp before the reset, the number of samples after the reset, the first three of their stamps, and how many of them are fresh. The failure message of [`assertTrue()`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L115) adds the limits:

    ```python title="test_imu_reset.py: the printed line and the failure message" linenums="108"
    --8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu_reset.py:108:122"
    ```

## What the tests don't show

Each item says what isn't tested, why not, and what a pass therefore doesn't prove.

- **That the yaw rate follows the robot's motion:** the two tests of [`test_imu.py`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py) check the yaw rate's mean at rest and in one left turn, as [what a pass of the left turn shows](#what-a-pass-of-the-left-turn-shows) explains. A yaw rate that keeps one value fails one of them, but a pass doesn't show how the yaw rate follows other motions.
- **That a repeated stamp after a reset is rare:** [`test_imu_reset.py`](#which-samples-count-as-fresh) doesn't count a repeated stamp and needs ten advancing ones. A stream with a few repeats among them still passes.
- **That the simulation can't stall while the controllers start:** the stall of [gazebo_plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/2.9.3/gazebo_plugins)' original plugin depends on timing: it needs a request of the controller spawner in the global callback queue at the moment that the plugin spins, as [one request, with each plugin](plugin.md#one-request-with-each-plugin) explains. Most starts pass, with the original too, and each test node starts the simulation once. So a pass of `test_imu.py` doesn't show that the deadlock is gone. The counts of many starts, with the original and with the copy, are in [diffbot#185](https://github.com/ros-mobile-robots/diffbot/issues/185). If a start did stall, [`setUp()`](#waiting-until-the-robot-can-drive) would end with an error, because no odometry comes. A longer [`STARTUP_TIMEOUT`](#reference-how-the-tests-wait) wouldn't help, because a stopped clock doesn't start again.
- **Which way the IMU's x and y axes point:** `test_imu.py` checks only the upward axis: the yaw rate and the reading of gravity. A rotation of the IMU about that axis changes neither. So a pass doesn't show that the [`imu_rpy`](position-and-axes.md#the-mounting-on-each-robot) of the robot's URDF model is right. Without the −90° of Remo's mounting, the test would still pass.
- **The size and the direction of the horizontal acceleration:** no test drives the robot forward and compares the reading with the robot's acceleration. A pass says nothing about the size and the sign of $f_x$ and $f_y$, the message's [`linear_acceleration.x`](index.md#the-linear-acceleration) and `linear_acceleration.y`. The EKF doesn't fuse the acceleration, but the IMU sample validator still reads it: the validator drops a sample when one of its six values isn't finite, with the sample's yaw rate. The excerpt shows this check, from [`imu_validator.py`, lines 268–273]({{ ekf_src_url }}/diffbot_control/scripts/imu_validator.py#L268-L273):

    ```python title="imu_validator.py: check_finite_values()" linenums="268"
    --8<-- "{{ ekf_src_raw }}/diffbot_control/scripts/imu_validator.py:268:273"
    ```

    The test at rest checks that $\omega_z$ and $f_z$ are finite. It doesn't look at the other four values, $\omega_x$, $\omega_y$, $f_x$ and $f_y$.

- **The real BNO055:** the simulated noise is a setting, $\sigma$ = 0.005 rad/s and 0.005 m/s², and the simulation adds no bias and no sensor delay to the values. So a pass says nothing about the real sensor's noise, bias, delay and mounting. The [bench checks](../driver-bno055.md#bench-checks) measure the real sensor's rate, noise, bias and axes on the robot. They don't measure a delay of the sensor. Without an added delay, a subscriber of the simulation still gets each sample a little after its stamp. In one run of Remo's simulation, a subscriber's callback for the messages on [`/diffbot/imu/data_raw`](index.md#what-the-simulated-imu-publishes) ran 1 to 2 ms of simulation time after the messages' stamps.
- **The rest of the simulation after a reset:** `test_imu_reset.py` checks the IMU's topic after one reset, about 3 s after the start. A pass doesn't show that the other publishers come back at once. Two of them don't:
    - **The drive controller's odometry:** [the EKF's limits](../robot_localization.md#limits) on the Localization page explain why, with the times of a run.
    - **The joint states:** [`joint_state_controller`](../../../development/simulation-test.md#what-runs-in-the-simulation) publishes only when the clock has passed [`last_publish_time_`](https://github.com/ros-controls/ros_controllers/blob/0.22.0/joint_state_controller/src/joint_state_controller.cpp#L105) by one publishing period. A reset sets the clock back to zero, and `last_publish_time_` keeps its value from before the reset. So the controller publishes the joint states again only when the clock has reached that time again. This follows from the controller's code and isn't measured in a run. The excerpt shows the condition in the controller's [`update()`](https://github.com/ros-controls/ros_controllers/blob/0.22.0/joint_state_controller/src/joint_state_controller.cpp#L102), from [`joint_state_controller.cpp` in ros_controllers 0.22.0, lines 104–110](https://github.com/ros-controls/ros_controllers/blob/0.22.0/joint_state_controller/src/joint_state_controller.cpp#L104-L110):

        ```cpp title="joint_state_controller.cpp: when update() publishes the joint states" linenums="104"
        --8<-- "https://raw.githubusercontent.com/ros-controls/ros_controllers/0.22.0/joint_state_controller/src/joint_state_controller.cpp:104:110"
        ```

- **The EKF:** these tests read `/diffbot/imu/data_raw`, the validator's input. Whether the validator forwards the samples and the EKF uses them is the subject of the [EKF's tests](../robot_localization.md#how-its-tested) on the Localization page.

## Reference: how the tests wait

This part explains the code around the checks: the constants, what [`setUp()`](#waiting-until-the-robot-can-drive) waits for, and which clock each wait of the tests counts.

??? example "The constants of `test_imu.py`"

    The excerpt shows all constants of the file, from [`test_imu.py`, lines 23–53]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L23-L53). The comments say what each number is. [`STARTUP_TIMEOUT`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L32) and [`POLL_INTERVAL`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L33) are seconds of wall time, the computer's clock. [`SETTLE`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L37), [`SPEED_UP`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L38) and [`MEASURE`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L39) are seconds of simulation time, Gazebo's clock. [Wall time and simulation time](#wall-time-and-simulation-time) explains the difference.

    ```python title="test_imu.py: the constants" linenums="23"
    --8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:23:53"
    ```

### Waiting until the robot can drive

Both tests of [`test_imu.py`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py) send velocity commands to the robot. A command that the robot doesn't receive would make the test measure a robot at rest. So before each test, [`setUp()`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L58-L70) waits until the IMU publishes and the drive controller listens. [unittest](https://docs.python.org/3.8/library/unittest.html#unittest.TestCase.setUp) calls `setUp()` before each test method. The excerpt shows `setUp()`, from [`test_imu.py`, lines 58–70]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L58-L70):

```python title="test_imu.py: setUp()" linenums="58"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:58:70"
```

- **[`List[Imu]`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L59):** [`Imu`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L21) is the Python class of the message type [`sensor_msgs/Imu`](index.md#what-the-simulated-imu-publishes), imported from `sensor_msgs.msg` in [line 21]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L21). Each IMU message arrives as an `Imu` object. `List[Imu]` is a type hint from Python's [`typing`](https://docs.python.org/3.8/library/typing.html) module: it says that `self.messages` is a list of `Imu` messages. Python doesn't check it when the test runs. It tells readers and tools such as mypy what the list holds.
- **The subscriber:** [`rospy.Subscriber(TOPIC, Imu, self.messages.append, queue_size=QUEUE_SIZE)`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L60) subscribes to `/diffbot/imu/data_raw`. Its arguments are the topic, the message class, and the callback that rospy calls for each message. The callback is the list's [`append`](https://docs.python.org/3.8/tutorial/datastructures.html#more-on-lists) method, so each message is added to `self.messages`. rospy runs the callback in one of its background threads, so the list fills while the test method waits. The test keeps no variable for the subscriber: rospy keeps the subscription until the node shuts down.
- **The publisher:** [`rospy.Publisher(COMMAND, Twist, queue_size=COMMAND_QUEUE_SIZE)`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L61) creates a publisher for [`geometry_msgs/Twist`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) velocity commands on the drive controller's topic. `self.command_publisher` keeps it for the test methods.
- **The first IMU message:** [`rospy.wait_for_message()`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L62) waits for this message. The message comes early, because Gazebo's IMU sensor, the plugin and the adapter publish soon after the robot is in the simulation.
- **An IMU message doesn't mean that the robot can drive:** the controller spawner starts the drive controller, [`mobile_base_controller`](../../../development/simulation-test.md#what-runs-in-the-simulation), later, as [the launch file on Simulation Test](../../../development/simulation-test.md#2-the-launch-file) shows. So `setUp()` waits for the controller's first odometry message, which shows that the controller runs.
- **A running controller can still miss the first commands:** ROS connects a publisher and a subscriber only after both exist, and a message published before the connection is lost. [`get_num_connections()`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L67) counts the subscribers connected to the test's publisher. The loop waits until the drive controller has connected, at most [`STARTUP_TIMEOUT`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L32), 60 s.

### Wall time and simulation time

The tests wait on two clocks:

- **Wall time** is the computer's clock, the time that a clock on the wall shows.
- **Simulation time** is Gazebo's clock. It starts at 0 s when the simulation starts and moves on by 1 ms with every simulation step. It stands still while Gazebo starts, and for good when the simulation stalls.

**Where the two differ:** the two clocks don't run at the same speed. Their ratio is Gazebo's real time factor: 1 when a second of simulation takes a second of wall time, and less than 1 when the computer is too slow for that, as Gazebo's tutorial on [physics parameters](https://classic.gazebosim.org/tutorials?tut=physics_params&cat=physics) explains. The test's world asks for real time, with 1000 steps of 1 ms per second in [lines 14–16 of `diffbot_empty.world`]({{ imu_sim_src_url }}/diffbot_gazebo/worlds/diffbot_empty.world#L14-L16). A busy computer, such as a CI runner, reaches less. So 2 s of simulation time can take 2 s of wall time, or 5 s.

**Which clock a ROS node reads:** a ROS node reads the computer's clock, unless the parameter [`use_sim_time`](../robot_localization/tests/validator.md#where-the-ros-time-comes-from) is set. Then [`rospy.Time.now()`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/rostime.py#L144), [`rospy.sleep()`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/timer.py#L116) and [`rospy.Rate`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/timer.py#L47) follow Gazebo's clock, which Gazebo publishes on the topic `/clock`. The ROS wiki's page on the [clock](http://wiki.ros.org/Clock) describes this. The simulation's launch file sets `use_sim_time`, in [lines 14 and 30 of `diffbot_view.launch`]({{ imu_sim_src_url }}/diffbot_gazebo/launch/diffbot_view.launch#L14-L30). Python's own [`time.monotonic()`](https://docs.python.org/3.8/library/time.html#time.monotonic) and [`time.sleep()`](https://docs.python.org/3.8/library/time.html#time.sleep) always count wall time.

Not every rospy function follows `use_sim_time`. [`rospy.wait_for_message()`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/client.py#L413-L420) counts its [`timeout`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/client.py#L391) on the computer's clock, with [`time.time()`](https://docs.python.org/3.8/library/time.html#time.time). The excerpt shows rospy's lines that count this timeout, from [`client.py` in ros_comm 1.17.4, lines 413–420]({{ ros_comm_src_url }}/clients/rospy/src/rospy/client.py#L413-L420), under the BSD license:

```python title="client.py (rospy 1.17.4): the timeout of wait_for_message()" linenums="413" hl_lines="4 7"
--8<-- "{{ ros_comm_src_raw }}/clients/rospy/src/rospy/client.py:413:420"
```

[`rospy.wait_for_service()`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/impl/tcpros_service.py#L128-L130) counts its `timeout` the same way, in [lines 128–130 of `tcpros_service.py`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/impl/tcpros_service.py#L128-L130).

**Which clock each wait of the tests counts:** the waits in wall time:

- **[`test_imu.py`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py), the first IMU message and the first odometry message**, each at most [`STARTUP_TIMEOUT`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L32), 60 s, in [lines 62 and 65]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L62-L65): while Gazebo starts, the simulation's clock stands still. A limit in simulation time would never run out.
- **`test_imu.py`, the drive controller's connection**, at most `STARTUP_TIMEOUT`, 60 s, with `time.monotonic()` and `time.sleep()` in [lines 66–70]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L66-L70): the same reason.
- **[`test_imu_reset.py`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py), Gazebo's reset service**, at most `STARTUP_TIMEOUT`, 60 s, in [line 59]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L59): while Gazebo starts, the simulation's clock stands still.
- **`test_imu_reset.py`, [`wait_for()`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L91-L98)** with [`RUN_TIMEOUT`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L48), 60 s, and [`RECOVERY_TIMEOUT`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L49), 30 s, with `time.monotonic()` and `time.sleep()` in [lines 90–98]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L90-L98): the test sets the simulation's clock back on purpose. `rospy.sleep()` raises an exception when the clock goes back while it waits, in [lines 161–163 of rospy's `timer.py`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/timer.py#L161-L163).
- **Both files, rostest's [`time-limit`]({{ ros_comm_src_url }}/tools/roslaunch/src/roslaunch/launch.py#L696-L703)**, 120 s for `test_imu.py` and 180 s for `test_imu_reset.py`, in [lines 36 and 40 of `sim_drive_diffbot.test`]({{ imu_sim_src_url }}/diffbot_gazebo/test/sim_drive_diffbot.test#L36-L40): the limit stops a test node that hangs.

The waits in simulation time:

- **`test_imu.py`, [`send_command()`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L72-L78)** for [`SETTLE`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L37), [`SPEED_UP`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L38) and [`MEASURE`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L39), 1 s, 1 s and 2 s, with `rospy.Time.now()` and `rospy.Rate` in [lines 72–78]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L72-L78): the robot's motion and the number of IMU messages depend on simulated seconds. `MEASURE` always covers 2 s of motion and about 100 IMU messages, however fast the computer runs the simulation.
- **`test_imu_reset.py`, [`RUN_BEFORE_RESET`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L39)**, 3 s, and **[`RECOVERY`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L40)**, 1 s, in [line 68]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L68) and [line 81]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L81): the test reads no clock for them. It reads the samples' stamps. They say how long the IMU ran before the reset and how early it came back, in the IMU's own time.

One wait counts no clock:

- **`test_imu_reset.py`, the call of [`/gazebo/reset_simulation`](plugin.md#publishing-again-after-a-reset)**, between the two `wait_for()` waits, in [line 106]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L106): the call waits for the service's answer. rospy's service call returns when the answer has arrived, in [lines 518–523 of `tcpros_service.py`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/impl/tcpros_service.py#L518-L523). The test gives the call no time limit, and `RECOVERY_TIMEOUT` starts only after the call returns. If the answer never came, only rostest's `time-limit` would end the test.

**When a wait in wall time runs out** in `test_imu.py`: without an IMU message or an odometry message in time, `rospy.wait_for_message()` raises an exception, and the test reports an error. When the drive controller doesn't connect in time, [`assertLess()`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L68) fails, and the test reports a failure. In `test_imu_reset.py`, the three timeouts only limit how long a failing run takes. The reset call has no timeout of its own, so rostest's `time-limit` of 180 s is its only bound.
