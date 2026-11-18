# IMU Simulation: Tests

What CI checks on the simulated IMU, how to run the checks yourself, and what a pass proves. This page belongs to [IMU: Simulation](index.md).

Two Python files check the simulated IMU in Gazebo, [`test_imu.py`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py) and [`test_imu_reset.py`]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py), with three tests. Each test reads the topic `/diffbot/imu/data_raw`:

1. **[The test at rest](#the-test-at-rest)**, in `test_imu.py`: the robot stands still.
    - **A pass shows:** the message has the form that its readers need. More than 90 messages arrive in 2 s of simulation time, each stamp is later than the one before, the frame is `imu_link`, the orientation $\mathbf q$ is marked unavailable, the yaw rate $\omega_z$ and the upward reading $f_z$ are finite, the yaw rate's variance is positive, and the mean of $f_z$ lies within 0.5 m/s² of 9.81 m/s², which is gravity.
    - **Why:** the IMU sample validator and the EKF need the message this way, and the firmware publishes it this way.
2. **[The left turn](#the-left-turn)**, in `test_imu.py`: the robot turns left on the spot at 0.5 rad/s.
    - **A pass shows:** the mean of the yaw rate $\omega_z$ over 2 s of the turn lies between 0.35 and 0.65 rad/s.
    - **Why:** the yaw rate is the one value that the EKF fuses, and its sign has to be that of the turn.
3. **[The reset test](#the-reset-test)**, in `test_imu_reset.py`: the test resets the simulation after about 3 s.
    - **A pass shows:** ten samples arrive after the reset with a stamp below 1 s.
    - **Why:** the IMU sample validator and the EKF start over at a reset and need samples right away.

[What the tests don't show](#what-the-tests-dont-show) lists what a pass doesn't prove. Two limits come from the checks themselves: a yaw rate that stays at one value [can pass both tests of `test_imu.py`](#what-a-pass-of-the-left-turn-shows), and ten samples with one repeated stamp [count as a recovery](#which-samples-count-as-fresh).

## Run the tests

You need the [dev container](../../../development/dev-container.md) with its workspace `~/catkin_ws` built and sourced, as the container's setup leaves it. In a terminal of the dev container, each of the two commands runs one robot's simulation test file, which contains the IMU tests:

```console
rostest diffbot_gazebo sim_drive_diffbot.test
rostest diffbot_gazebo sim_drive_remo.test
```

Each command takes about two minutes on our test PC, because rostest starts Gazebo again for every test node. A passing run of the first command ends with these lines, with the test file at the commit that this page shows:

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

The three highlighted lines are the IMU's tests. What to expect from each:

- **The left turn** passes when the mean yaw rate lies between 0.35 and 0.65 rad/s. A failure prints the measured mean. With a yaw rate that stays at 0 rad/s, for example, the message is `0.0 != 0.5 within 0.15 delta (0.5 difference) : turning left at 0.5 rad/s, the IMU measured 0.000 rad/s about z`.
- **The test at rest** passes without output. A failure names the check that failed and the value that it found.
- **The reset test** prints one line, also when it passes. rostest keeps the line in the test node's result file. The next command prints the line of DiffBot's last run:

    ```console
    grep -o 'last stamp before the reset[^<&]*' ~/.ros/test_results/diffbot_gazebo/rosunit-imu_reset_diffbot.xml
    ```

    ```text
    last stamp before the reset 3.000 s; after the reset 10 samples, first stamps ['0.020', '0.040', '0.060'], 10 of them earlier than 1.0 s
    ```

    Remo's result file is `rosunit-imu_reset_remo.xml`. The numbers differ a little from run to run: a run with Remo printed 3.019 s and 11 samples. [The reset test](#the-reset-test) explains the numbers.

Simulation Test also shows how to [run all tests as CI does](../../../development/simulation-test.md#run-it) and how to [read the results](../../../development/simulation-test.md#read-the-results).

### How rostest runs the IMU tests

Both simulation test files, `sim_drive_diffbot.test` and `sim_drive_remo.test`, run each of the two Python files as a test node of its own: `imu_diffbot` and `imu_reset_diffbot` for DiffBot, `imu_remo` and `imu_reset_remo` for Remo. The excerpt shows DiffBot's two test nodes, from [`sim_drive_diffbot.test`, lines 28–34]({{ imu_sim_src_url }}/diffbot_gazebo/test/sim_drive_diffbot.test#L28-L34):

```xml title="sim_drive_diffbot.test: the two IMU test nodes" linenums="28"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/sim_drive_diffbot.test:28:34"
```

rostest starts Gazebo and the robot again for each of these test nodes, as for the other test nodes in [the test file on Simulation Test](../../../development/simulation-test.md#1-the-test-file). So the reset of `test_imu_reset.py` doesn't reach the other tests.

`test_imu.py` has two test methods. unittest runs a test class's methods in [alphabetical order](https://docs.python.org/3/library/unittest.html#unittest.TestLoader.sortTestMethodsUsing). So the left turn, `test_imu_measures_a_positive_yaw_rate_in_a_left_turn`, runs before the test at rest, `test_imu_publishes_the_firmware_message_at_rest`. This page explains the test at rest first, because its checks are the simpler ones.

Before each of the two test methods, `setUp()` waits until the IMU publishes and the drive controller listens to commands. [Waiting until the robot can drive](#waiting-until-the-robot-can-drive) in the reference part of this page explains these waits.

## The test at rest

`test_imu_publishes_the_firmware_message_at_rest` checks that the message on `/diffbot/imu/data_raw` is what its readers need. The EKF runs at 50 Hz, the firmware's rate. The IMU sample validator forwards only samples with the frame `imu_link`, rising stamps, finite values and a positive variance. And the firmware's message has no orientation. The robot stands still during this test, because then the reading on the upward axis, $f_z$, is known: gravity alone.

The figure shows what happens during the test, from left to right over simulation time: the command that the test sends, the robot, the reading that the test checks, and the IMU's messages that the test keeps.

--8<-- "includes/imu-sim-test-at-rest.html"

**What the robot does:** the test sends the command to stand still, first for `SETTLE`, 1 s, and then for `MEASURE`, 2 s. The excerpt shows these two lines at the start of the test method, from [`test_imu.py`, lines 84–87]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L84-L87):

```python title="test_imu.py: stand still, then collect" linenums="84"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:84:87"
```

`stand_still` is `Twist()`, a command with all velocities 0. The test sends this command for `SETTLE` first, because the robot may still be braking after the left turn. Then the test collects the IMU's messages of $T$ = `MEASURE` = 2 s in the list `messages`.

The test uses two helpers for this, which the left turn uses too. The excerpt shows both helpers, from [`test_imu.py`, lines 67–82]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L67-L82):

```python title="test_imu.py: send_command() and collect_messages()" linenums="67"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:67:82"
```

- **`send_command()`** publishes a velocity command for `seconds` of simulation time, 20 times per second (`COMMAND_RATE`). One command isn't enough: the drive controller stops the robot when no command arrives for 0.5 s.
- **`collect_messages()`** returns the IMU's messages of the next `seconds`. It empties the list, sends the command for that time while the subscriber fills the list, and returns a copy of the list.

The checks follow, each with its lines.

**The rate:** the excerpt shows the check of the rate, from [`test_imu.py`, lines 88–91]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L88-L91):

```python title="test_imu.py: the rate" linenums="88"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:88:91"
```

`rate` is $N / T$, with $N$ messages in $T$ = 2 s. The check is $N / T > 0.9 \cdot 50 \text{ Hz} = 45 \text{ Hz}$. So the test needs more than 90 messages, which is at least 91. The firmware publishes at 50 Hz, and the EKF runs at that rate.

**The stamps:** the excerpt shows the check of the stamps, from [`test_imu.py`, lines 92–94]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L92-L94):

```python title="test_imu.py: the stamps" linenums="92"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:92:94"
```

Each stamp must be later than the one before. The IMU sample validator drops a sample that isn't later than the last one it forwarded.

**The frame and the orientation:** the excerpt shows the first two checks that the test makes for each message, from [`test_imu.py`, lines 95–98]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L95-L98):

```python title="test_imu.py: the frame and the orientation" linenums="95"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:95:98"
```

- The frame must be `imu_link`. The EKF looks up this frame to convert the yaw rate into the robot's axes.
- `orientation_covariance[0]` must be −1: the orientation $\mathbf q$, a quaternion with four numbers, is marked unavailable, as in the firmware's message. [The adapter](measurement-to-message.md#part-3-the-adapter) sets it.

**The values and the yaw rate's variance:** the excerpt shows the other checks in the same loop, from [`test_imu.py`, lines 99–104]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L99-L104):

```python title="test_imu.py: the values and the variance" linenums="99"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:99:104"
```

- `angular_velocity.z`, the yaw rate $\omega_z$, and `linear_acceleration.z`, the upward reading $f_z$, must be finite: not NaN, not infinite. The IMU sample validator drops samples with values that aren't finite. This check doesn't ask that the yaw rate is near 0 rad/s, although the robot stands still.
- `variance`, the message's `angular_velocity_covariance[8]`, must be positive and finite. Element 8 is the yaw rate's variance, $\sigma^2 = 2.5 \cdot 10^{-5}$ (rad/s)² in the simulation, as [how the message stores a covariance](index.md#how-the-message-stores-a-covariance) and [filling the covariances](measurement-to-message.md#filling-the-covariances) show. The EKF weighs the yaw rate with it, and the validator drops samples without a positive, finite variance.

**Gravity:** the excerpt shows the check of the upward reading, from [`test_imu.py`, lines 105–109]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L105-L109):

```python title="test_imu.py: gravity" linenums="105"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:105:109"
```

`mean_upward_reading` is the average of the upward axis's $N$ readings, $\bar f_z = \frac{1}{N} \sum_{i=1}^{N} f_{z,i}$. The check needs

$$
|\bar f_z - 9.81 \text{ m/s}^2| \le 0.5 \text{ m/s}^2
$$

At rest, the upward axis reads gravity, $f_z = +9.8$ m/s² in Gazebo, as [the linear acceleration](index.md#the-linear-acceleration) explains. An IMU frame that is upside down reads about −9.8 m/s², and a plugin that leaves gravity out reads about 0 m/s².

## The left turn

`test_imu_measures_a_positive_yaw_rate_in_a_left_turn` checks the yaw rate $\omega_z$, the one value that the EKF fuses, while the robot turns. At rest, the yaw rate is 0 rad/s however the IMU is mounted. Only a turn shows the sign of the yaw rate.

The figure shows what happens during the test: the turn command, the robot's headings that follow from the command at the start, after `SPEED_UP` and after `MEASURE`, the yaw rate $\omega_z$ that the test checks, and the IMU's messages that the test keeps.

--8<-- "includes/imu-sim-test-left-turn.html"

The excerpt shows the whole test method, from [`test_imu.py`, lines 111–121]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L111-L121):

```python title="test_imu.py: the left turn" linenums="111"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:111:121"
```

- **What the robot does:** `turn_left` is the command, a turn on the spot at `TURN_RATE`, 0.5 rad/s, in the command's `angular.z`. [`send_command()`](#the-test-at-rest) sends `turn_left` for `SPEED_UP`, 1 s, so that the robot reaches the turn rate. Then [`collect_messages()`](#the-test-at-rest) keeps the robot turning for `MEASURE`, 2 s, and returns the IMU's messages of that time, which the test keeps in `messages`. Then the test stops the robot with `Twist()`.
- **What the test checks:** `mean_yaw_rate` is the average of the yaw rates of `messages`, $\bar\omega_z$. The check needs $|\bar\omega_z - 0.5 \text{ rad/s}| \le 0.15 \text{ rad/s}$, so a mean between 0.35 and 0.65 rad/s. A left turn, counterclockwise seen from above, is positive about the upward z axis, as [the angular velocity](index.md#the-angular-velocity) shows.

### What a pass of the left turn shows

A pass shows one thing: the mean of the yaw rates that arrived in the 2 s of `MEASURE` lies between 0.35 and 0.65 rad/s. So the test fails when the IMU's frame is upside down, which gives about −0.5 rad/s, and when the yaw rate stays near 0 rad/s.

A pass doesn't show that the yaw rate follows the robot's motion:

- **A yaw rate that stays at 0.5 rad/s passes both tests.** The left turn accepts this value as the mean. The test at rest checks only that each yaw rate is finite. So an IMU whose yaw rate never changes from +0.5 rad/s would pass `test_imu.py`.
- **The mean has no minimum number of messages.** The left turn has no check of the rate. One message with a yaw rate in the accepted range is enough for a pass. With no message at all, the division by `len(yaw_rates)` raises a `ZeroDivisionError`, and the test reports an error, not a failure with the check's message.

## The reset test

`test_imu_reset.py` checks that the IMU publishes again right after a reset of the simulation. A reset sets Gazebo's clock back to zero. A plugin that doesn't notice this stays silent for as long as the simulation ran before the reset, and the EKF gets no yaw rate in that time. [Publishing again after a reset](plugin.md#publishing-again-after-a-reset) explains how the robots' plugin handles a clock that went back.

The figure shows the stamps of the IMU's samples during the test, with the robots' IMU plugin and with a plugin that keeps the time from before the reset. The figure is an idealized example: the reset comes at exactly 3 s, and the IMU publishes exactly 50 samples per second. In a run, the reset comes a little later, for example at 3.019 s. The robot stands still in this test. What changes is the simulation's clock.

--8<-- "includes/imu-sim-test-reset.html"

The excerpt shows the test's constants, from [`test_imu_reset.py`, lines 32–46]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L32-L46):

```python title="test_imu_reset.py: the constants" linenums="32" hl_lines="5 6"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu_reset.py:32:46"
```

`RUN_BEFORE_RESET` and `RECOVERY`, the highlighted lines, are seconds of simulation time. The test compares them with the samples' stamps. The three timeouts and `POLL_INTERVAL` are seconds of wall time.

`setUp()` subscribes to `/diffbot/imu/data_raw` with the callback `note_stamp()`, which notes each sample's stamp in the list `self.stamps`. Then `setUp()` waits for Gazebo's reset service.

??? example "`setUp()`, `note_stamp()` and `imu_ran_before_reset()`"

    The excerpt shows how the test collects the stamps and how it decides that the IMU has run long enough, from [`test_imu_reset.py`, lines 51–64]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L51-L64). `self.stamps[-1]` is the stamp of the sample that arrived last:

    ```python title="test_imu_reset.py: setUp(), note_stamp() and imu_ran_before_reset()" linenums="51"
    --8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu_reset.py:51:64"
    ```

The next excerpt shows the start of the test method, from [`test_imu_reset.py`, lines 89–96]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L89-L96):

```python title="test_imu_reset.py: run, reset, wait for fresh samples" linenums="89" hl_lines="7"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu_reset.py:89:96"
```

1. **Run:** the test waits until the latest stamp is at least `RUN_BEFORE_RESET`, 3 s. The test looks at the stamps every `POLL_INTERVAL`, 0.02 s of wall time, so the last stamp before the reset is 3 s or a little later. In the two runs of [Run the tests](#run-the-tests), the last stamp was 3.000 s and 3.019 s. `last_time` in the plugin then holds about that time.
2. **Reset:** the test notes how many samples have arrived, in `sample_count_at_reset`, and calls `/gazebo/reset_simulation`, the highlighted line. Gazebo's clock starts from zero again.
3. **Wait for fresh samples:** the test waits until `FRESH_SAMPLE_COUNT`, 10, fresh samples have arrived, at most `RECOVERY_TIMEOUT`, 30 s of wall time. The test fails when the samples don't arrive in that time.

### Which samples count as fresh

The excerpt shows which samples count as fresh, from [`test_imu_reset.py`, lines 66–77]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L66-L77):

```python title="test_imu_reset.py: fresh_stamps() and imu_recovered()" linenums="66"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu_reset.py:66:77"
```

`fresh_stamps()` takes the stamps that arrived after the test noted `sample_count_at_reset`, and keeps those below `RECOVERY`, 1 s. Such a stamp can only come from the new simulation time: before the reset, the stamps had reached 3 s. `imu_recovered()` is true when ten stamps are left.

- **With the robots' plugin**, the first sample after the reset is stamped about 0.02 s, one period after the clock went back. Ten fresh samples then take about 0.2 s of simulation time. The test doesn't require these two values: it accepts any ten stamps below 1 s.
- **With a plugin that keeps the old time**, the next sample comes only after the clock has passed 3 s again. Then no sample is stamped below 1 s, and the test fails.
- **What the count doesn't check:** `fresh_stamps()` compares each stamp with `RECOVERY` and not with the stamps before it. Ten samples with one and the same stamp count as ten fresh samples, also with the stamp 0 s. The IMU sample validator would forward one of them at most, because it drops a sample whose stamp isn't later than the last valid sample's. So a pass shows that samples with new stamps arrive, not that their stamps advance. In the runs for this page, the first three stamps after the reset were 0.020 s, 0.040 s and 0.060 s.

After the wait, the test prints one line and then checks the result.

??? example "The printed line and the failure message"

    The excerpt shows the end of the test method, from [`test_imu_reset.py`, lines 97–111]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L97-L111). `print()` writes the line that [Run the tests](#run-the-tests) shows: the last stamp before the reset, the number of samples after the reset, the first three of their stamps, and how many of them are fresh. The failure message of `assertTrue()` adds the limits:

    ```python title="test_imu_reset.py: the printed line and the failure message" linenums="97"
    --8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu_reset.py:97:111"
    ```

## What the tests don't show

Each item says what isn't tested, why not, and what a pass therefore doesn't prove.

- **That the yaw rate follows the robot's motion:** the two tests of `test_imu.py` check the yaw rate's mean in the turn and only its finiteness at rest, as [what a pass of the left turn shows](#what-a-pass-of-the-left-turn-shows) explains. A pass doesn't show that the yaw rate is near 0 rad/s at rest.
- **That the stamps advance after a reset:** `test_imu_reset.py` counts the samples with a stamp below 1 s and doesn't compare their stamps, as [which samples count as fresh](#which-samples-count-as-fresh) explains. The test at rest checks rising stamps, but only before a reset.
- **That the simulation can't stall while the controllers start:** the stall of gazebo_plugins' original plugin depends on timing: it needs a request of the controller spawner in the global callback queue at the moment that the plugin spins, as [one request, with each plugin](plugin.md#one-request-with-each-plugin) explains. Most starts pass, with the original too, and each test node starts the simulation once. So a pass of `test_imu.py` doesn't show that the deadlock is gone. The counts of many starts, with the original and with the copy, are in [diffbot#185](https://github.com/ros-mobile-robots/diffbot/issues/185). If a start did stall, `setUp()` would end with an error, because no odometry comes. A longer `STARTUP_TIMEOUT` wouldn't help, because a stopped clock doesn't start again.
- **Which way the IMU's x and y axes point:** `test_imu.py` checks only the upward axis: the yaw rate and the reading of gravity. A rotation of the IMU about that axis changes neither. So a pass doesn't show that the `imu_rpy` of the robot's URDF model is right. Without the −90° of Remo's mounting, the test would still pass.
- **The size and the direction of the horizontal acceleration:** no test drives the robot forward and compares the reading with the robot's acceleration. A pass says nothing about the size and the sign of $f_x$ and $f_y$, the message's `linear_acceleration.x` and `linear_acceleration.y`. The EKF doesn't fuse the acceleration, but the IMU sample validator still reads it: the validator drops a sample when one of its six values isn't finite, with the sample's yaw rate. The excerpt shows this check, from [`imu_validator.py`, lines 268–273]({{ ekf_src_url }}/diffbot_control/scripts/imu_validator.py#L268-L273):

    ```python title="imu_validator.py: check_finite_values()" linenums="268"
    --8<-- "{{ ekf_src_raw }}/diffbot_control/scripts/imu_validator.py:268:273"
    ```

    The test at rest checks that $\omega_z$ and $f_z$ are finite. It doesn't look at the other four values, $\omega_x$, $\omega_y$, $f_x$ and $f_y$.

- **The real BNO055:** the simulated noise is a setting, $\sigma$ = 0.005 rad/s and 0.005 m/s², and the simulation adds no bias and no sensor delay to the values. So a pass says nothing about the real sensor's noise, bias, delay and mounting. The [bench checks](../driver-bno055.md#bench-checks) measure those on the robot. Without an added delay, a subscriber still gets each sample a little after its stamp. In one run of Remo's simulation, the messages on `/diffbot/imu/data_raw` arrived 1 to 2 ms of simulation time after their stamps.
- **The rest of the simulation after a reset:** `test_imu_reset.py` checks the IMU's topic after one reset, about 3 s after the start. A pass doesn't show that the other publishers come back at once. Two of them don't:
    - **The drive controller's odometry:** [the EKF's limits](../robot_localization.md#limits) on the Localization page explain why, with the times of a run.
    - **The joint states:** `joint_state_controller` publishes only when the clock has passed `last_publish_time_` by one publishing period. A reset sets the clock back to zero, and `last_publish_time_` keeps its value from before the reset. So the controller publishes the joint states again only when the clock has reached that time again. This follows from the controller's code and isn't measured in a run. The excerpt shows the condition in the controller's `update()`, from [`joint_state_controller.cpp` in ros_controllers 0.22.0, lines 104–110](https://github.com/ros-controls/ros_controllers/blob/0.22.0/joint_state_controller/src/joint_state_controller.cpp#L104-L110):

        ```cpp title="joint_state_controller.cpp: when update() publishes the joint states" linenums="104"
        --8<-- "https://raw.githubusercontent.com/ros-controls/ros_controllers/0.22.0/joint_state_controller/src/joint_state_controller.cpp:104:110"
        ```

- **The EKF:** these tests read `/diffbot/imu/data_raw`, the validator's input. Whether the validator forwards the samples and the EKF uses them is the subject of the [EKF's tests](../robot_localization.md#how-its-tested) on the Localization page.

## Reference: how the tests wait

This part explains the code around the checks: the constants, what `setUp()` waits for, and which clock each wait of the tests counts.

??? example "The constants of `test_imu.py`"

    The excerpt shows all constants of the file, from [`test_imu.py`, lines 22–48]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L22-L48). The comments say what each number is. `STARTUP_TIMEOUT` and `POLL_INTERVAL` are seconds of wall time, the computer's clock. `SETTLE`, `SPEED_UP` and `MEASURE` are seconds of simulation time, Gazebo's clock. [Wall time and simulation time](#wall-time-and-simulation-time) explains the difference.

    ```python title="test_imu.py: the constants" linenums="22"
    --8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:22:48"
    ```

### Waiting until the robot can drive

Both tests of `test_imu.py` send velocity commands to the robot. A command that the robot doesn't receive would make the test measure a robot at rest. So before each test, `setUp()` waits until the IMU publishes and the drive controller listens. unittest calls `setUp()` before each test method. The excerpt shows `setUp()`, from [`test_imu.py`, lines 53–65]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L53-L65):

```python title="test_imu.py: setUp()" linenums="53"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/test/test_imu.py:53:65"
```

- **`List[Imu]`:** `Imu` is the Python class of the message type `sensor_msgs/Imu`, imported from `sensor_msgs.msg` in [line 20]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L20). Each IMU message arrives as an `Imu` object. `List[Imu]` is a type hint from Python's [`typing`](https://docs.python.org/3/library/typing.html) module: it says that `self.messages` is a list of `Imu` messages. Python doesn't check it when the test runs. It tells readers and tools such as mypy what the list holds.
- **The subscriber:** `rospy.Subscriber(TOPIC, Imu, self.messages.append, queue_size=QUEUE_SIZE)` subscribes to `/diffbot/imu/data_raw`. Its arguments are the topic, the message class, and the callback that rospy calls for each message. The callback is the list's `append` method, so each message is added to `self.messages`. rospy runs the callback in one of its background threads, so the list fills while the test method waits. The test keeps no variable for the subscriber: rospy keeps the subscription until the node shuts down.
- **The publisher:** `rospy.Publisher(COMMAND, Twist, queue_size=COMMAND_QUEUE_SIZE)` creates a publisher for `geometry_msgs/Twist` velocity commands on the drive controller's topic. `self.command_publisher` keeps it for the test methods.
- **The first IMU message:** `rospy.wait_for_message()` waits for this message. The message comes early, because Gazebo's IMU sensor, the plugin and the adapter publish soon after the robot is in the simulation.
- **An IMU message doesn't mean that the robot can drive:** the controller spawner starts the drive controller, `mobile_base_controller`, later, as [the launch file on Simulation Test](../../../development/simulation-test.md#2-the-launch-file) shows. So `setUp()` waits for the controller's first odometry message, which shows that the controller runs.
- **A running controller can still miss the first commands:** ROS connects a publisher and a subscriber only after both exist, and a message published before the connection is lost. `get_num_connections()` counts the subscribers connected to the test's publisher. The loop waits until the drive controller has connected, at most `STARTUP_TIMEOUT`, 60 s.

### Wall time and simulation time

The tests wait on two clocks:

- **Wall time** is the computer's clock, the time that a clock on the wall shows.
- **Simulation time** is Gazebo's clock. It starts at 0 s when the simulation starts and moves on by 1 ms with every simulation step. It stands still while Gazebo starts, and for good when the simulation stalls.

**Where the two differ:** the two clocks don't run at the same speed. Their ratio is Gazebo's real time factor: 1 when a second of simulation takes a second of wall time, and less than 1 when the computer is too slow for that, as Gazebo's tutorial on [physics parameters](https://classic.gazebosim.org/tutorials?tut=physics_params&cat=physics) explains. The test's world asks for real time, with 1000 steps of 1 ms per second in [lines 14–16 of `diffbot_empty.world`]({{ imu_sim_src_url }}/diffbot_gazebo/worlds/diffbot_empty.world#L14-L16). A busy computer, such as a CI runner, reaches less. So 2 s of simulation time can take 2 s of wall time, or 5 s.

**Which clock a ROS node reads:** a ROS node reads the computer's clock, unless the parameter `use_sim_time` is set. Then `rospy.Time.now()`, `rospy.sleep()` and `rospy.Rate` follow Gazebo's clock, which Gazebo publishes on the topic `/clock`. The ROS wiki's page on the [clock](http://wiki.ros.org/Clock) describes this. The simulation's launch file sets `use_sim_time`, in [lines 14 and 30 of `diffbot_view.launch`]({{ imu_sim_src_url }}/diffbot_gazebo/launch/diffbot_view.launch#L14-L30). Python's own `time.monotonic()` and `time.sleep()` always count wall time.

Not every rospy function follows `use_sim_time`. `rospy.wait_for_message()` counts its `timeout` on the computer's clock, with `time.time()`. The excerpt shows rospy's lines that count this timeout, from [`client.py` in ros_comm 1.17.4, lines 413–420]({{ ros_comm_src_url }}/clients/rospy/src/rospy/client.py#L413-L420), under the BSD license:

```python title="client.py (rospy 1.17.4): the timeout of wait_for_message()" linenums="413" hl_lines="4 7"
--8<-- "{{ ros_comm_src_raw }}/clients/rospy/src/rospy/client.py:413:420"
```

`rospy.wait_for_service()` counts its `timeout` the same way, in [lines 128–130 of `tcpros_service.py`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/impl/tcpros_service.py#L128-L130).

**Which clock each wait of the tests counts.** The waits in wall time:

- **`test_imu.py`, the first IMU message and the first odometry message**, each at most `STARTUP_TIMEOUT`, 60 s, in [lines 57 and 60]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L57-L60): while Gazebo starts, the simulation's clock stands still. A limit in simulation time would never run out.
- **`test_imu.py`, the drive controller's connection**, at most `STARTUP_TIMEOUT`, 60 s, with `time.monotonic()` and `time.sleep()` in [lines 61–65]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L61-L65): the same reason.
- **`test_imu_reset.py`, Gazebo's reset service**, at most `STARTUP_TIMEOUT`, 60 s, in [line 55]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L55): while Gazebo starts, the simulation's clock stands still.
- **`test_imu_reset.py`, `wait_for()`** with `RUN_TIMEOUT`, 60 s, and `RECOVERY_TIMEOUT`, 30 s, with `time.monotonic()` and `time.sleep()` in [lines 79–87]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L79-L87): the test sets the simulation's clock back on purpose. `rospy.sleep()` raises an exception when the clock goes back while it waits, in [lines 161–163 of rospy's `timer.py`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/timer.py#L161-L163).
- **Both files, rostest's `time-limit`**, 120 s for `test_imu.py` and 180 s for `test_imu_reset.py`, in [lines 30 and 34 of `sim_drive_diffbot.test`]({{ imu_sim_src_url }}/diffbot_gazebo/test/sim_drive_diffbot.test#L30-L34): the limit stops a test node that hangs.

The waits in simulation time:

- **`test_imu.py`, `send_command()`** for `SETTLE`, `SPEED_UP` and `MEASURE`, 1 s, 1 s and 2 s, with `rospy.Time.now()` and `rospy.Rate` in [lines 67–73]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu.py#L67-L73): the robot's motion and the number of IMU messages depend on simulated seconds. `MEASURE` always covers 2 s of motion and about 100 IMU messages, however fast the computer runs the simulation.
- **`test_imu_reset.py`, `RUN_BEFORE_RESET`**, 3 s, and **`RECOVERY`**, 1 s, in [lines 64 and 73]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L64-L73): the test reads no clock for them. It reads the samples' stamps. They say how long the IMU ran before the reset and how early it came back, in the IMU's own time.

One wait counts no clock:

- **`test_imu_reset.py`, the call of `/gazebo/reset_simulation`**, between the two `wait_for()` waits, in [line 95]({{ imu_sim_src_url }}/diffbot_gazebo/test/test_imu_reset.py#L95): the call waits for the service's answer. rospy's service call returns when the answer has arrived, in [lines 518–523 of `tcpros_service.py`]({{ ros_comm_src_url }}/clients/rospy/src/rospy/impl/tcpros_service.py#L518-L523). The test gives the call no time limit, and `RECOVERY_TIMEOUT` starts only after the call returns. If the answer never came, only rostest's `time-limit` would end the test.

**When a wait in wall time runs out** in `test_imu.py`: without an IMU message or an odometry message in time, `rospy.wait_for_message()` raises an exception, and the test reports an error. When the drive controller doesn't connect in time, `assertLess()` fails, and the test reports a failure. In `test_imu_reset.py`, the three timeouts only limit how long a failing run takes. The reset call has no timeout of its own, so rostest's `time-limit` of 180 s is its only bound.
