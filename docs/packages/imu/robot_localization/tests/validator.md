# Tests: The IMU Sample Validator

[`test_imu_validator.py`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py) tests the [IMU sample validator](../../robot_localization.md#the-imu-sample-validator) one decision at a time. Its 30 unit tests call the validator's Python class and functions directly, without a ROS master, a node or a topic. They check which samples the validator accepts, the reason for each sample that it rejects, the silence and the level that its diagnostics report, and the counts of a report. The [stepped-clock test](stepped-clock.md) runs the node together with the EKF. This page belongs to the [tests of the Localization](index.md).

## One sample through the validator

Two examples show the validator's decision before the code. Each example gives one sample to a new validator, one that has seen no sample yet, at the ROS time $t$ = 100 s. This is the ROS time of most tests, their constant [`NOW`](#the-constants-and-the-helpers). Both samples are in the frame `imu_link`, with finite values and positive variances, as the firmware sends them. They differ only in their stamp $t_s$:

- **Sample B, stamped $t_s$ = 99.900000 s:** its age $t - t_s$ is 0.100000 s, exactly [`max_age`](../../robot_localization.md#check-4-the-stamp-against-the-ros-time). The validator accepts B, and the node forwards the sample.
- **Sample A, stamped $t_s$ = 99.899999 s:** its age $t - t_s$ is 0.100001 s, 1 µs more than `max_age`. The validator rejects A as [`TOO_OLD`](#dropreason-the-reasons-to-drop-a-sample), and the node drops the sample.

The figure shows the decision's steps in the order of its arrows. A sample that meets a step's condition goes on to the next step. A sample that doesn't takes the arrow "no" to its drop reason, in a dashed red box. The numbers 1 to 5 are those of [the five checks](../../robot_localization.md#the-five-checks) on the Localization page.

--8<-- "includes/localization-test-validator-decision.html"

??? note "Why B passes and A fails, how to read the figure, and the two ends of the decision"

    [Check 4](../../robot_localization.md#check-4-the-stamp-against-the-ros-time) lets a sample through that is at most `max_age`, 0.1 s, old: $t - t_s \le \texttt{max\_age}$. B is exactly that old and passes. [Check 5](../../robot_localization.md#check-5-the-stamp-against-the-last-forwarded-sample) has no stamp of an earlier valid sample to compare B's stamp with, so B passes check 5 too, and the validator accepts B. A is 1 µs older than B and fails check 4. The same two samples appear in the test of [Check 4: the two time limits](#check-4-the-two-time-limits), which gives both to one validator.

    The figure shows every step of the decision, in the order of its arrows. On a wide screen, the steps run from left to right in two rows. On a narrow screen, they run from top to bottom. Each step is a box with a condition. A sample that meets the condition follows the arrow to the next step. A sample that doesn't meet the condition takes the arrow "no" to the reason for which the validator rejects the sample, in a dashed red box. The second step drops no sample: after the ROS time went back, the validator sets its attributes `last_valid_stamp` and `last_valid_arrival_time` to `None`. These two attributes hold the stamp and the arrival time of the last valid sample. Then the decision goes on, as [a clock that goes back](#clear_last_valid_sample_times_if_clock_went_back-a-clock-that-goes-back) explains. The numbers 1 to 5 are those of [the five checks](../../robot_localization.md#the-five-checks) on the Localization page.

    The decision has two ends:

    - **Accepted:** [`drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample) stores the sample's stamp $t_s$ in the validator's attribute `last_valid_stamp` and its arrival time $t$ in `last_valid_arrival_time`, and returns `None`. The node counts the sample as forwarded and publishes the sample for the EKF. The tests check this end with [`assert_accepted()`](#validatortestcase-a-new-validator-and-two-assertions).
    - **Rejected:** `drop_reason_or_record()` returns the reason. The validator stores neither the sample's stamp nor its arrival time in `last_valid_stamp` and `last_valid_arrival_time`. At a ROS time other than 0, the clock check has already stored the call's ROS time in `last_checked_ros_time`. The node counts the sample under this reason, which the [diagnostics](#testsamplecounts-the-numbers-of-a-report) report, and passes a warning to [`rospy.logwarn_throttle()`](https://github.com/ros/ros_comm/blob/1.17.0/clients/rospy/src/rospy/core.py#L242) with the period [`WARNING_PERIOD`](#where-the-ros-time-comes-from), 5 s. The tests check this end with [`assert_rejected()`](#validatortestcase-a-new-validator-and-two-assertions).

## Run the tests

Open a shell in which the workspace is sourced and start the file with Python's [`unittest`](https://docs.python.org/3/library/unittest.html) from its folder:

```console
roscd diffbot_control/test
python3 -m unittest -v test_imu_validator
```

A pass prints one line for each test, with `ok` at its end, and ends with `Ran 30 tests` and `OK`. catkin runs the file with the package's other tests, and CI runs it on every pull request, as [Testing and CI](../../../../development/ci.md#ekf-tests) describes.

??? example "How the package registers its three EKF tests"

    catkin runs the file with nose, a test runner for Python, as part of the package's tests. The excerpt shows how `diffbot_control` registers its three EKF tests, from [`CMakeLists.txt`, lines 226–234]({{ ekf_test_src_url }}/diffbot_control/CMakeLists.txt#L226-L234). [`catkin_add_nosetests()`](https://github.com/ros/catkin/blob/noetic-devel/doc/howto/format2/python_nose_configuration.rst) registers this file. [`add_rostest()`](https://github.com/ros/catkin/blob/noetic-devel/doc/howto/format2/rostest_configuration.rst) registers the two scenarios of the [stepped-clock test](stepped-clock.md), one `.test` file each:

    ```cmake title="diffbot_control/CMakeLists.txt: the package's tests" linenums="226"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/CMakeLists.txt:226:234"
    ```

??? example "The output of a run"

    This is the whole output of a run at the commit [`3882a19`](https://github.com/ros-mobile-robots/diffbot/commit/3882a19cbda8543f5a0f1c5d4f0965caa90d4f3d) of [diffbot#220](https://github.com/ros-mobile-robots/diffbot/pull/220), whose files are those of its merge commit [`7d428c2`](https://github.com/ros-mobile-robots/diffbot/commit/7d428c282c073ad4b99a76cab834276daadb5a6f), in the [dev container](../../../../development/dev-container.md)'s image with ROS Noetic and Python 3.8.10. The run started `python3 -m unittest -v test_imu_validator` in the folder `diffbot_control/test`, the folder that `roscd` changes to. With the option `-v`, unittest prints one line for each test: the test's name, its class in parentheses, and `ok` for a test that passed. unittest runs the classes, and the tests in each class, in the alphabetical order of their names, so the output's order differs from the file's. [`ValidatorTestCase`](#validatortestcase-a-new-validator-and-two-assertions) has no tests, so the output has no line for this class. The last lines give the number of tests and the time that they took, and `OK` when every test passed:

    ```text
    test_health_is_a_warning_with_new_drops (test_imu_validator.TestHealth) ... ok
    test_health_is_an_error_before_the_first_valid_sample (test_imu_validator.TestHealth) ... ok
    test_health_is_an_error_for_silence_also_with_new_drops (test_imu_validator.TestHealth) ... ok
    test_health_is_ok_at_the_silence_limit_and_an_error_beyond_it (test_imu_validator.TestHealth) ... ok
    test_health_is_ok_without_new_drops (test_imu_validator.TestHealth) ... ok
    test_validator_accepts_a_new_sample_after_rejecting_a_future_one (test_imu_validator.TestImuValidator) ... ok
    test_validator_accepts_a_sample_max_age_old_and_rejects_an_older_one (test_imu_validator.TestImuValidator) ... ok
    test_validator_accepts_a_sample_max_future_ahead_and_rejects_a_later_one (test_imu_validator.TestImuValidator) ... ok
    test_validator_accepts_a_stamp_once_after_the_clock_goes_back (test_imu_validator.TestImuValidator) ... ok
    test_validator_accepts_valid_samples (test_imu_validator.TestImuValidator) ... ok
    test_validator_rejects_a_sample_at_ros_time_zero (test_imu_validator.TestImuValidator) ... ok
    test_validator_rejects_a_sample_not_later_than_the_last_valid_one (test_imu_validator.TestImuValidator) ... ok
    test_validator_rejects_a_sample_with_a_nan_or_infinite_value (test_imu_validator.TestImuValidator) ... ok
    test_validator_rejects_a_sample_with_a_wrong_or_empty_frame (test_imu_validator.TestImuValidator) ... ok
    test_validator_rejects_a_sample_with_an_invalid_angular_velocity_covariance (test_imu_validator.TestImuValidator) ... ok
    test_validator_rejects_stamps_outside_time_limits_after_the_clock_goes_back (test_imu_validator.TestImuValidator) ... ok
    test_validator_rejects_the_same_stamp_again_after_a_sample_at_ros_time_zero (test_imu_validator.TestImuValidator) ... ok
    test_snapshot_counts_a_dropped_sample_as_new_once_also_in_an_error_report (test_imu_validator.TestSampleCounts) ... ok
    test_snapshot_counts_a_dropped_sample_as_new_only_once (test_imu_validator.TestSampleCounts) ... ok
    test_snapshot_has_the_forwarded_count_and_the_drops_by_reason (test_imu_validator.TestSampleCounts) ... ok
    test_snapshot_keeps_its_numbers_when_more_samples_arrive (test_imu_validator.TestSampleCounts) ... ok
    test_silence_is_none_after_the_clock_goes_back_less_than_the_silence (test_imu_validator.TestSecondsSinceLastValidSample) ... ok
    test_silence_is_none_after_the_clock_goes_back_without_samples (test_imu_validator.TestSecondsSinceLastValidSample) ... ok
    test_silence_is_none_before_the_first_valid_sample (test_imu_validator.TestSecondsSinceLastValidSample) ... ok
    test_silence_is_none_before_the_last_check_after_a_sample_at_ros_time_zero (test_imu_validator.TestSecondsSinceLastValidSample) ... ok
    test_silence_is_none_when_a_rejected_sample_shows_the_clock_went_back (test_imu_validator.TestSecondsSinceLastValidSample) ... ok
    test_silence_is_the_time_since_the_last_valid_sample (test_imu_validator.TestSecondsSinceLastValidSample) ... ok
    test_silence_keeps_growing_after_a_sample_at_ros_time_zero (test_imu_validator.TestSecondsSinceLastValidSample) ... ok
    test_silence_keeps_growing_while_samples_are_rejected (test_imu_validator.TestSecondsSinceLastValidSample) ... ok
    test_silence_restarts_with_the_first_valid_sample_after_the_clock_goes_back (test_imu_validator.TestSecondsSinceLastValidSample) ... ok

    ----------------------------------------------------------------------
    Ran 30 tests in 0.002s

    OK
    ```

## What a pass proves and what a failure means

A pass means that, for the tests' samples and times, the validator decides as [the five checks](../../robot_localization.md#the-five-checks) describe, and that its diagnostics report the expected silence, level and counts.

A failure names the test and the assertion, with the value that came back. The test's name says which decision went wrong, and its section below shows the test. For example, [`test_validator_accepts_a_sample_max_age_old_and_rejects_an_older_one`](#check-4-the-two-time-limits) expects that the validator accepts a sample exactly 0.1 s old and rejects one 1 µs older as [`TOO_OLD`](#dropreason-the-reasons-to-drop-a-sample). Its failure shows which of the two decisions differed and the value that came back. The tests give the validator their own limits, so a changed launch parameter [`max_age`](../../robot_localization.md#check-4-the-stamp-against-the-ros-time) doesn't make them fail.

### What the tests don't show

- **The node:** no test creates an [`ImuValidatorNode`](#where-the-ros-time-comes-from), with its topics, parameters and lock.
- **The real clock and the real IMU:** whether [`max_age`](../../robot_localization.md#check-4-the-stamp-against-the-ros-time) and [`max_future`](../../robot_localization.md#check-4-the-stamp-against-the-ros-time) fit the robot's USB link is a measurement on the robot.
- **Values that the validator doesn't check:** a finite but implausible value passes by design.

??? note "The details"

    - **The node:** the class [`ImuValidatorNode`](#where-the-ros-time-comes-from) reads the parameters, subscribes to the topic `imu/data_raw`, publishes on the topic `imu/data_validated` and on the topic `/diagnostics`, and reads the ROS time in a lock around each sample. The unit tests don't create an `ImuValidatorNode`: they call the methods of [`ImuValidator`](#what-the-tests-call) directly. The [stepped-clock test](stepped-clock.md) runs the node with the EKF and checks its counts on the topic `/diagnostics`.
    - **The real clock and the real IMU:** every time in these tests is a number that the test chose. Whether [`max_age`](../../robot_localization.md#check-4-the-stamp-against-the-ros-time) and [`max_future`](../../robot_localization.md#check-4-the-stamp-against-the-ros-time) fit the delays of the robot's USB link is a measurement on the robot, as [Limits](../../robot_localization.md#limits) says. The issue [IMU validator: measure the samples' age on the robot, then set max_age and max_future (diffbot#219)](https://github.com/ros-mobile-robots/diffbot/issues/219) measures it.
    - **The fields that the validator doesn't check:** the [overview](../../robot_localization.md#what-the-validator-doesnt-check) lists them. No test sends a sample with an implausible but finite value, because the validator accepts such a sample by design. The EKF doesn't drop such a value either. The first measurement initializes the filter. For each later measurement, robot_localization's EKF computes the difference between the measurement and the filter's prediction and corrects the estimate with this difference, unless a rejection threshold finds the difference too large ([`ekf.cpp`, lines 164–189](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ekf.cpp#L164-L189)), as the same section of the overview explains. The IMU's threshold for the angular velocity, [`imu0_twist_rejection_threshold`](../../robot_localization.md#what-the-validator-doesnt-check), defaults to the largest `double` ([`ros_filter.cpp`, line 1418](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ros_filter.cpp#L1418)), and [`ekf.yaml`]({{ ekf_test_src_url }}/diffbot_control/config/ekf.yaml) sets none. So a large but finite yaw rate in a valid sample moves the estimate.

## The test classes

The file holds the 30 tests in four test classes, one for each part of [`imu_validator.py`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py) that works without ROS communication:

| Test class | Tests | What it calls | What it checks |
|:--|:--|:--|:--|
| [`TestImuValidator`](#testimuvalidator-which-samples-are-valid) | 12 | [`ImuValidator.drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample) | Which samples the validator accepts, and the reason for each sample that it rejects |
| [`TestSecondsSinceLastValidSample`](#testsecondssincelastvalidsample-the-silence) | 9 | [`ImuValidator.seconds_since_last_valid_sample()`](#seconds_since_last_valid_sample-the-silence) | The time since the last valid sample, which the diagnostics report, also when the clock goes back |
| [`TestHealth`](#testhealth-the-status) | 5 | [`health()`](#testhealth-the-status) | The level of the diagnostics status: OK, warning or error |
| [`TestSampleCounts`](#testsamplecounts-the-numbers-of-a-report) | 4 | [`SampleCounts`](#testsamplecounts-the-numbers-of-a-report) | The counts of forwarded and dropped samples in a diagnostics report |

A fifth class, [`ValidatorTestCase`](#validatortestcase-a-new-validator-and-two-assertions), holds no tests. It gives the first two test classes a new validator for each test and two assertions about the validator's decision on a sample.

## What the tests call

Most tests call the class [`ImuValidator`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L203-L356), which holds the validator's checks without the ROS node around them. The sections below explain the class's methods that the tests call or that run inside these calls, and the reasons to drop a sample that the class returns, with the code in closed boxes. [The five checks](../../robot_localization.md#the-five-checks) on the Localization page explain the checks themselves.

### Where the ROS time comes from

[`ImuValidator`](#what-the-tests-call) never reads the ROS clock. Its methods get the ROS time $t$ as their argument `now`: from the node on the robot and in the simulation, and in the tests a time that the test chooses, most often [`NOW`](#the-constants-and-the-helpers), 100 s.

??? note "The node's callback, and what the ROS clock returns"

    [`ImuValidator`](#what-the-tests-call) never reads the ROS clock itself. Its methods [`drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample) and [`seconds_since_last_valid_sample()`](#seconds_since_last_valid_sample-the-silence) get the ROS time $t$ as their argument `now`. The caller decides which time that is:

    - **On the robot and in the simulation:** the node's class [`ImuValidatorNode`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L359) calls the two methods. For each sample, rospy calls the node's callback [`on_sample()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L383-L398), which reads the ROS time with [`rospy.Time.now()`](https://github.com/ros/ros_comm/blob/1.17.0/clients/rospy/src/rospy/rostime.py#L144) and passes it to `drop_reason_or_record()` as `now`. For each diagnostics report, the node's method [`take_snapshot()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L415) reads the ROS time in the same way and passes it to `seconds_since_last_valid_sample()`.
    - **In the tests:** each test passes a ROS time that it chooses, most often [`NOW`](#the-constants-and-the-helpers), 100 s. So a test can put any ROS time next to any stamp $t_s$, and needs no clock and no waiting.

    The excerpt shows `on_sample()`, from [`imu_validator.py`, lines 383–398]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L383-L398). The callback reads the ROS time with `rospy.Time.now()` into `now` and passes it with the sample to `drop_reason_or_record()`:

    ```python title="imu_validator.py: the node reads the ROS time and passes it on" linenums="383"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:383:398"
    ```

    `on_sample()` stores the method's result in the variable `drop_reason`. The result is either `None` or a reason to drop the sample:

    - **`None`:** Python's value for "nothing", here "no reason to drop the sample". The node counts the sample as forwarded and publishes the sample on the topic `imu/data_validated`.
    - **A reason:** a member of [`DropReason`](#dropreason-the-reasons-to-drop-a-sample). The node counts the sample by this reason with [`count_dropped()`](#testsamplecounts-the-numbers-of-a-report). After the lock, the callback passes a warning with the reason's text to [`rospy.logwarn_throttle()`](https://github.com/ros/ros_comm/blob/1.17.0/clients/rospy/src/rospy/core.py#L242) with the period [`WARNING_PERIOD`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L102), 5 s, which leaves out repeated warnings within that period.

    The lock `self.lock` around both calls keeps the diagnostics timer, which runs in another thread, from reading the validator in the middle of a sample.

    What `rospy.Time.now()` returns depends on the ROS parameter [`/use_sim_time`](https://github.com/ros/ros_comm/blob/1.17.0/clients/rospy/src/rospy/impl/simtime.py#L71-L93), as the ROS wiki's page [Clock](http://wiki.ros.org/Clock) explains:

    - **On the robot:** none of the launch files that start the robot sets `/use_sim_time`. [Which launch file starts what](../../robot_localization.md#which-launch-file-starts-what) on the Localization page shows these files. Without the parameter, `rospy.Time.now()` returns the system clock of the robot's single-board computer (SBC), in seconds since 1 January 1970. This clock doesn't read 0.
    - **In the simulation:** gazebo_ros's [`empty_world.launch`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros/launch/empty_world.launch#L27), which [`diffbot_view.launch`]({{ ekf_test_src_url }}/diffbot_gazebo/launch/diffbot_view.launch#L30) includes, sets `/use_sim_time` to true. Gazebo publishes the simulation's time on the topic `/clock`, and `rospy.Time.now()` returns the time of the last message there. Until the node has received the first message on `/clock`, `rospy.Time.now()` returns 0. [Before the checks: no clock](#before-the-checks-no-clock) shows what the validator does then.

    The [stepped-clock test](stepped-clock.md) sets `/use_sim_time` too, and publishes the messages on `/clock` itself.

### `drop_reason_or_record()`: the decision about a sample

[`drop_reason_or_record(message, now)`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L258-L275) decides about one sample, stamped $t_s$, that arrives at the ROS time $t$, `now`. The method returns the first reason to drop the sample, or records the sample's stamp and arrival time and returns `None`.

??? note "The two arguments, and what the name says"

    [`drop_reason_or_record(message, now)`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L258-L275) decides about one sample. The node calls the method once for each sample, and the tests call the method through [two assertions](#validatortestcase-a-new-validator-and-two-assertions). Its two arguments are:

    - **`message`:** the sample, a [`sensor_msgs/Imu`](../../robot_localization.md#the-messages-and-their-fields) message. The sample carries its own time, the stamp $t_s$ in `header.stamp`, which the sender writes into the message. On the robot, the Teensy's firmware stamps each sample with rosserial's ROS time when it starts to read the BNO055. That time estimates when the BNO055 measured, because the sensor doesn't report that time, as the [firmware's IMU publisher]({{ ekf_test_src_url }}/diffbot_firmware/lib/imu/imu_publisher.h#L36-L38) explains. In the simulation, Gazebo's IMU plugin stamps each sample with the simulation time of the measurement. The header also carries the frame of the values, `header.frame_id`. [The messages and their fields](../../robot_localization.md#the-messages-and-their-fields) on the Localization page lists the sample's fields.
    - **`now`:** the ROS time $t$ at which the sample arrives at the validator, a [`rospy.Time`](https://github.com/ros/ros_comm/blob/1.17.0/clients/rospy/src/rospy/rostime.py#L95). The sample's way from the sender to the node takes time, so $t$ is usually a little later than $t_s$. On the robot, [rosserial synchronizes](https://github.com/ros-drivers/rosserial/blob/0.9.1/rosserial_client/src/ros_lib/ros/node_handle.h#L354-L383) the Teensy's clock only approximately with the SBC's clock. The remaining offset can put $t_s$ a little after $t$. Check 4 allows a stamp up to [`max_future`](../../robot_localization.md#check-4-the-stamp-against-the-ros-time), 0.02 s, ahead.

    The validator checks both the frame and the stamp: [check 1](../../robot_localization.md#checks-1-to-3-the-samples-form) the frame, [check 4](../../robot_localization.md#check-4-the-stamp-against-the-ros-time) the stamp $t_s$ against the ROS time $t$, and [check 5](../../robot_localization.md#check-5-the-stamp-against-the-last-forwarded-sample) the stamp $t_s$ against the stamp of the last valid sample.

    The name lists the method's two outcomes in their order: the caller gets the drop reason, the first reason to drop the sample, or, when the sample has no drop reason, the method records the sample's two times and returns `None`. To record means here that the validator stores the sample's stamp in its attribute `last_valid_stamp` and the ROS time of the sample's arrival in `last_valid_arrival_time`, for its next decisions. The validator doesn't keep the message or write it to a file. The return type in the method's first line, `Optional[DropReason]`, says the same: the result is a member of [`DropReason`](#dropreason-the-reasons-to-drop-a-sample) or `None`. So `None` means "no reason to drop": the sample is valid, and the node forwards the sample.

The excerpt shows the method, from [`imu_validator.py`, lines 258–275]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L258-L275):

```python title="imu_validator.py: drop_reason_or_record()" linenums="258"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:258:275"
```

The method runs two steps, each in a method of its own: first [`drop_reason()`](#step-1-drop_reason-the-decision), then, only when the reason is `None`, [`record_valid_sample_times()`](#step-2-record_valid_sample_times-a-valid-sample).

#### Step 1: `drop_reason()`, the decision

[`drop_reason()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L277-L295) returns the first reason to drop the sample, or `None` when the sample passes every check: first the clock, then the clock check, then the six check functions in their order. A sample that would fail two checks gets the reason of the earlier one.

??? example "`drop_reason()` and the order of the checks"

    [`drop_reason()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L277-L295) returns the first reason to drop the sample, or `None` when the sample passes every check. The excerpt shows the method, from [`imu_validator.py`, lines 277–295]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L277-L295):

    ```python title="imu_validator.py: drop_reason()" linenums="277"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:277:295"
    ```

    `drop_reason()` works in three steps:

    1. **No clock:** if a sample arrives at the ROS time 0 s, `drop_reason()` returns [`DropReason.NO_CLOCK`](#dropreason-the-reasons-to-drop-a-sample) at once. The validator reads 0 s as "no clock yet", as the comment above this return says, so the method skips step 2, the clock check. The clock check compares the current ROS time with `last_checked_ros_time`, the ROS time of the last clock check. At startup, before the first message on `/clock`, a new validator has no earlier check. Now suppose that the validator last checked the clock at 100 s and a clock publisher sends 0 s. A sample that arrives at 0 s leaves the validator's attributes as they are: 100 s in `last_checked_ros_time`, and the last valid sample's stamp and arrival time in `last_valid_stamp` and `last_valid_arrival_time`. If the next check reads 0.02 s, the check sets `last_valid_stamp` and `last_valid_arrival_time` to `None`, because 0.02 s is earlier than 100 s. That next check can come from a sample or from a diagnostics report. [A test in Before the checks: no clock](#before-the-checks-no-clock) and [two tests of the silence](#after-a-sample-at-the-ros-time-0) check that a sample at 0 s changes none of the three attributes.
    2. **A clock that went back:** [`clear_last_valid_sample_times_if_clock_went_back(now)`](#clear_last_valid_sample_times_if_clock_went_back-a-clock-that-goes-back) clears `last_valid_stamp` and `last_valid_arrival_time` if `now` is earlier than `last_checked_ros_time`. The method then stores `now` in `last_checked_ros_time` for the next clock check.
    3. **The checks:** the loop calls the checks in the tuple `sample_checks`, one after the other. The first check that returns a reason ends the loop, and the method returns that reason. The later checks don't run. When every check has returned `None`, the method returns `None`.

    The constructor [`__init__()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L211-L230) fills `sample_checks` in the order in which the checks run. The excerpt shows the tuple, from [`imu_validator.py`, lines 222–230]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L222-L230):

    ```python title="imu_validator.py: the checks in their order" linenums="222"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:222:230"
    ```

    The six functions are [the five checks](../../robot_localization.md#the-five-checks) of the Localization page. Check 4 has two functions, one for each time limit: [`check_not_too_far_ahead()`](../../robot_localization.md#check-4-the-stamp-against-the-ros-time) and [`check_not_too_old()`](../../robot_localization.md#check-4-the-stamp-against-the-ros-time). A sample that would fail two checks gets the reason of the earlier one, as sample A in [Check 4: the two time limits](#check-4-the-two-time-limits) shows.

#### Step 2: `record_valid_sample_times()`, a valid sample

Only for a valid sample, [`record_valid_sample_times()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L297-L303) stores the sample's stamp in `last_valid_stamp` and its arrival time in `last_valid_arrival_time`. So the stamp of a dropped sample never becomes the last valid stamp.

??? example "`record_valid_sample_times()`, and what a call changes"

    Only when [`drop_reason()`](#step-1-drop_reason-the-decision) has returned `None`, [`record_valid_sample_times()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L297-L303) stores two times of the sample in attributes of the validator: the sample's stamp in `last_valid_stamp` and its arrival time in `last_valid_arrival_time`. The excerpt shows the method, from [`imu_validator.py`, lines 297–303]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L297-L303):

    ```python title="imu_validator.py: record_valid_sample_times()" linenums="297"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:297:303"
    ```

    `record_valid_sample_times()` stores the two times: the sample's stamp $t_s$ in `last_valid_stamp`, the stamp $t_\text{last}$ that [check 5](../../robot_localization.md#check-5-the-stamp-against-the-last-forwarded-sample) compares the next stamps with, and the ROS time of its arrival, `now`, in `last_valid_arrival_time`, from which [`seconds_since_last_valid_sample()`](#seconds_since_last_valid_sample-the-silence) counts the silence. Each valid sample replaces the values of the two attributes, so the validator keeps no history of samples.

    What a call of [`drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample) changes in the validator's attributes depends on how the call ends. The table lists the three cases:

    | How the call ends | `last_valid_stamp` and `last_valid_arrival_time` | `last_checked_ros_time`, which the next clock check compares with |
    |:--|:--|:--|
    | The sample is valid | replaced by this sample's stamp and `now` | `now` |
    | A check drops the sample | unchanged, or set to `None` when `now` shows that the clock went back | `now` |
    | The ROS time is 0 s ([`NO_CLOCK`](#dropreason-the-reasons-to-drop-a-sample)) | unchanged | unchanged |

    So the stamp of a dropped sample never becomes $t_\text{last}$. The node counts every dropped sample by its reason, `NO_CLOCK` included, and passes a warning to [`rospy.logwarn_throttle()`](https://github.com/ros/ros_comm/blob/1.17.0/clients/rospy/src/rospy/core.py#L242) with the period [`WARNING_PERIOD`](#where-the-ros-time-comes-from), 5 s.

#### Why one method runs both steps

The node and the tests both call [`drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample). So neither can record the times of a dropped sample or forget those of a valid one.

??? note "What each of the two mistakes would do"

    Two places in the code call [`drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample): the node's callback [`on_sample()`](#where-the-ros-time-comes-from), once for each sample that arrives, and the tests, through [`assert_accepted()` and `assert_rejected()`](#validatortestcase-a-new-validator-and-two-assertions). Neither calls [`drop_reason()`](#step-1-drop_reason-the-decision) and [`record_valid_sample_times()`](#step-2-record_valid_sample_times-a-valid-sample) itself. So neither can get the two steps wrong in one of two ways:

    - **Recording a dropped sample's times:** check 5 would compare later stamps with the stamp of a sample that the checks dropped. If a caller recorded the times of a sample from the future, otherwise valid samples with stamps no later than that stored stamp would be dropped as [`NOT_NEWER`](#dropreason-the-reasons-to-drop-a-sample).
    - **Not recording a valid sample's times:** check 5 would still let the same stamp through, and the silence would keep counting from the arrival time that was recorded last. If no arrival time had been recorded, the silence would stay `None`. Such omissions could make the diagnostics report an error while valid samples arrive.

    The tests therefore run the same two steps as the node.

### `clear_last_valid_sample_times_if_clock_went_back()`: a clock that goes back

When a call's ROS time is earlier than `last_checked_ros_time`, the ROS time of the last clock check, the clock went back. The method then sets `last_valid_stamp` and `last_valid_arrival_time` to `None`, so the order check and the silence start over.

??? example "The clock check, its two helpers, and a sample after the jump"

    The ROS time can jump back. In the simulation, a call of the service `/gazebo/reset_simulation` starts Gazebo's clock again near 0 s. On the robot, a correction of the SBC's clock can set the time back. Without a reaction, [check 5](../../robot_localization.md#check-5-the-stamp-against-the-last-forwarded-sample) would still compare new stamps with the stamp $t_\text{last}$ of the last valid sample from before the jump. Samples that pass checks 1 to 4 would then be dropped as [`NOT_NEWER`](#dropreason-the-reasons-to-drop-a-sample) while their stamps are no later than $t_\text{last}$. For example, suppose the last valid stamp is 100 s and the simulation restarts near 0 s. If each new sample's stamp equals its arrival time, those samples would be dropped until their stamps exceed 100 s, about 100 s of simulation time later.

    When the clock went back, the method sets the validator's attributes `last_valid_stamp` and `last_valid_arrival_time` to `None`. These two attributes hold the stamp of the last valid sample and the ROS time of its arrival, the last valid sample's times of the method's name. The validator stores no sample, only these two times. The validator also stores the ROS time of the last clock check in its attribute `last_checked_ros_time`. The excerpt shows the method and its two helpers, from [`imu_validator.py`, lines 232–256]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L232-L256):

    ```python title="imu_validator.py: clear_last_valid_sample_times_if_clock_went_back() and its two helpers" linenums="232"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:232:256"
    ```

    - **[`clock_went_back(now)`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L246-L248):** returns whether `now` is earlier than `last_checked_ros_time`. Before the first clock check, `last_checked_ros_time` is `None`, and the answer is `False`.
    - **[`clear_last_valid_sample_times()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L250-L256):** sets the attributes `last_valid_stamp` and `last_valid_arrival_time` to `None`, as before the first valid sample. `clear_last_valid_sample_times()` undoes what [`record_valid_sample_times()`](#step-2-record_valid_sample_times-a-valid-sample) stores.
    - **[`clear_last_valid_sample_times_if_clock_went_back(now)`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L232-L244):** calls `clear_last_valid_sample_times()` only when `clock_went_back(now)` is true. Then the method stores `now` in `last_checked_ros_time` for the next clock check, whether the clock went back or not.

    After a jump back, the validator starts over as if it had seen no valid sample yet:

    - **Check 5:** with no stamp $t_\text{last}$ to compare with, check 5 lets the next sample through. If checks 1 to 4 pass too, the sample is valid, and its stamp becomes the new $t_\text{last}$.
    - **The silence:** [`seconds_since_last_valid_sample()`](#seconds_since_last_valid_sample-the-silence) returns `None` until a sample is valid again. A diagnostics report in that time shows the error "no IMU sample forwarded yet, or since the clock went back". At 50 Hz, samples are spaced 20 ms apart. The silence becomes a number again only when a sample passes all five checks.

    The table follows two samples through a new validator, one that has seen no sample yet, with the values of the test [`test_validator_accepts_a_stamp_once_after_the_clock_goes_back`](#a-clock-that-goes-back) in [A clock that goes back](#a-clock-that-goes-back):

    - **The first sample:** is stamped 100 s and arrives at the ROS time 100 s. Both values are the test's [`NOW`](#the-constants-and-the-helpers).
    - **Sample J:** arrives after the clock went back, as after a call of `/gazebo/reset_simulation`. J is stamped 0.02 s and arrives at the ROS time 0.02 s. Both values are the test's constant [`RESTART_NEAR_ZERO`](#the-constants-and-the-helpers).

    Each column of the table is one sample. The rows follow the sample's call of [`drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample) from top to bottom. The clock check stores its ROS time in the validator's attribute `last_checked_ros_time`. The row "Result" gives the values of the attributes `last_valid_stamp` and `last_valid_arrival_time` after the call:

    | Step of the call | The first sample | Sample J |
    |:--|:--|:--|
    | Stamp $t_s$ | 100 s | 0.02 s |
    | ROS time $t$ at arrival | 100 s | 0.02 s |
    | Clock check: $t$ against the ROS time of the last clock check | No clock check yet, so nothing to compare with | 100 s. 0.02 s is earlier, so the clock went back: the check sets `last_valid_stamp` and `last_valid_arrival_time` to `None` |
    | `last_checked_ros_time` after the clock check | 100 s | 0.02 s |
    | Check 5: $t_s$ against the last valid stamp | No stamp to compare with, so the check passes | No stamp to compare with, so the check passes |
    | Result | Valid. `last_valid_stamp`: 100 s. `last_valid_arrival_time`: 100 s | Valid. `last_valid_stamp`: 0.02 s. `last_valid_arrival_time`: 0.02 s |

    Without the clock check, `last_valid_stamp` would still be 100 s when J arrives. Check 5 would drop J as `NOT_NEWER`, because J's stamp of 0.02 s is not later than 100 s.

    Two methods call `clear_last_valid_sample_times_if_clock_went_back()` with their own `now`: [`drop_reason()`](#step-1-drop_reason-the-decision) for each sample that arrives at a ROS time other than 0, and `seconds_since_last_valid_sample()` for each diagnostics report. So the validator notices a jump back also while no sample arrives. The tests never call the method directly. They see its effect in [A clock that goes back](#a-clock-that-goes-back) and in the tests of [the silence](#testsecondssincelastvalidsample-the-silence).

### `seconds_since_last_valid_sample()`: the silence

[`seconds_since_last_valid_sample(now)`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L342-L356) checks the clock first. Then it returns the silence $t - t_\text{valid}$ in s, the ROS time since the last valid sample arrived, or `None` when no sample was valid since the clock last went back. The [validator's diagnostics](../../robot_localization.md#the-validators-diagnostics) report this silence.

??? example "`seconds_since_last_valid_sample()`"

    [`seconds_since_last_valid_sample(now)`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L342-L356) returns how long no sample has been valid at the ROS time `now`. The [validator's diagnostics](../../robot_localization.md#the-validators-diagnostics) report this silence. The excerpt shows the method, from [`imu_validator.py`, lines 342–356]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L342-L356):

    ```python title="imu_validator.py: seconds_since_last_valid_sample()" linenums="342"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:342:356"
    ```

    The method first calls [`clear_last_valid_sample_times_if_clock_went_back()`](#clear_last_valid_sample_times_if_clock_went_back-a-clock-that-goes-back), so the validator notices a clock that went back also while no sample arrives. Then the method returns one of two things:

    - **`None`:** no sample was valid since the clock last went back, or none at all.
    - **The silence in s, a [`float`](https://docs.python.org/3/library/functions.html#float):** the ROS time $t$ minus $t_\text{valid}$, the ROS time `last_valid_arrival_time` at which the last valid sample arrived: $t - t_\text{valid}$. The silence counts from the sample's arrival, not from its stamp.

### `DropReason`: the reasons to drop a sample

[`DropReason`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L106-L118) holds one member for each of the seven reasons to drop a sample, each with the text that the log and the diagnostics show, such as [`TOO_OLD`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L117) with `too old`.

??? example "The seven members, and `DropReason` in the code"

    The table lists the seven members with the check that returns each:

    | Member | Returned by | Text in the log and on `/diagnostics` |
    |:--|:--|:--|
    | [`NO_CLOCK`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L112) | [`drop_reason()`](#step-1-drop_reason-the-decision) itself, before the checks, at the ROS time 0 | `no ROS clock` |
    | [`WRONG_FRAME`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L113) | [Check 1](../../robot_localization.md#checks-1-to-3-the-samples-form), [`check_frame()`](../../robot_localization.md#checks-1-to-3-the-samples-form) | `wrong frame` |
    | [`NOT_FINITE`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L114) | [Check 2](../../robot_localization.md#checks-1-to-3-the-samples-form), [`check_finite_values()`](../../robot_localization.md#checks-1-to-3-the-samples-form) | `not finite` |
    | [`INVALID_ANGULAR_VELOCITY_COVARIANCE`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L115) | [Check 3](../../robot_localization.md#checks-1-to-3-the-samples-form), [`check_angular_velocity_covariance()`](../../robot_localization.md#checks-1-to-3-the-samples-form) | `invalid angular velocity covariance` |
    | [`FROM_THE_FUTURE`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L116) | [Check 4](../../robot_localization.md#check-4-the-stamp-against-the-ros-time), [`check_not_too_far_ahead()`](../../robot_localization.md#check-4-the-stamp-against-the-ros-time) | `from the future` |
    | [`TOO_OLD`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L117) | [Check 4](../../robot_localization.md#check-4-the-stamp-against-the-ros-time), [`check_not_too_old()`](../../robot_localization.md#check-4-the-stamp-against-the-ros-time) | `too old` |
    | [`NOT_NEWER`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L118) | [Check 5](../../robot_localization.md#check-5-the-stamp-against-the-last-forwarded-sample), [`check_stamp_later_than_last_valid()`](../../robot_localization.md#check-5-the-stamp-against-the-last-forwarded-sample) | `not newer than the last one` |

    [`DropReason`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L106-L118) is an [enumeration](https://docs.python.org/3/library/enum.html), a fixed set of named members, one for each reason to drop a sample. The excerpt shows the class, from [`imu_validator.py`, lines 105–118]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L105-L118):

    ```python title="imu_validator.py: DropReason" linenums="105"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:105:118"
    ```

    Each member's value is a text. The node writes the text into its warning in the log, as [`on_sample()`](#where-the-ros-time-comes-from) shows. The diagnostics count the dropped samples by reason, each under a key that starts with `Dropped,` and ends with the text, as `Dropped, too old` in the example of the [validator's diagnostics](../../robot_localization.md#the-validators-diagnostics). The table lists the seven members with the check that returns each:

    The tests import the module [`imu_validator`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py) and not its names. So a test writes a member with the module's name in front, as [`imu_validator.DropReason.TOO_OLD`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L117).

## The constants and the helpers

The tests share constants, such as [`NOW`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L23), 100 s, the ROS time of most checks, and two helpers: [`make_imu_sample()`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L68-L84) builds a sample like the firmware's that is valid at `NOW` for a new validator, and [`seconds()`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L87-L89) turns a number of seconds into a [`rospy.Duration`](https://github.com/ros/ros_comm/blob/1.17.0/clients/rospy/src/rospy/rostime.py#L59).

??? example "The constants of `test_imu_validator.py`"

    The excerpt shows all constants of the file, from [`test_imu_validator.py`, lines 23–65]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L23-L65). [`NOW`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L23) is the ROS time $t$ of most checks, 100 s. [`MAX_AGE`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L25) and [`MAX_FUTURE`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L26) are the two time limits that [`ekf.launch`]({{ ekf_test_src_url }}/diffbot_control/launch/ekf.launch) sets on the robot and in the simulation. [`JUST_TOO_OLD`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L33) and [`JUST_TOO_FAR_AHEAD`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L34) lie [`ONE_MICROSECOND`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L29), 1 µs, beyond these limits. The tests that use the other constants explain them in their sections, and the comment above each constant says which condition its value meets. [`ONE_NANOSECOND`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L28) is a [`rospy.Duration`](https://github.com/ros/ros_comm/blob/1.17.0/clients/rospy/src/rospy/rostime.py#L59) of exactly 1 ns, built from whole nanoseconds and not with [`seconds()`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L87-L89). `seconds()` passes a number of seconds to [`rospy.Duration.from_sec()`](https://github.com/ros/genpy/blob/0.6.18/src/genpy/rostime.py#L79-L87), which multiplies the number by 10⁹ as a floating-point number and cuts off the fraction of a nanosecond. So `seconds(3.3)` is 3.299999999 s.

    ```python title="test_imu_validator.py: the constants" linenums="23"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:23:65"
    ```

??? example "`make_imu_sample()` and `seconds()`"

    Every test that gives the validator a sample builds the sample with the helper [`make_imu_sample()`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L68-L84). The excerpt shows `make_imu_sample()` and the helper `seconds()`, from [`test_imu_validator.py`, lines 68–89]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L68-L89):

    ```python title="test_imu_validator.py: make_imu_sample() and seconds()" linenums="68"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:68:89"
    ```

    - **`make_imu_sample()`:** returns a [`sensor_msgs/Imu`](../../robot_localization.md#the-messages-and-their-fields) message like the firmware's. With the defaults of its three arguments, the sample is valid at the ROS time `NOW` for a new validator, one that has seen no sample yet. If the last valid sample has the same stamp, [check 5](../../robot_localization.md#check-5-the-stamp-against-the-last-forwarded-sample) drops the new sample. With the defaults, the sample's stamp $t_s$ is `NOW`, its frame is `imu_link` and its yaw rate $\omega_z$ is [`YAW_RATE`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L57), 0.5 rad/s. The accelerometer's reading on the upward axis, $f_z$, is [`GRAVITY`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L58), 9.81 m/s², as for an IMU at rest. `orientation_covariance[0]` is −1, which marks the orientation as unavailable, as in the firmware's message.
    - **The variances:** each variance on the diagonal of the angular velocity's covariance $\boldsymbol\Sigma_{\boldsymbol\omega}$, such as $\sigma^2_{\omega_z}$, is [`ANGULAR_VELOCITY_VARIANCE`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L59), 4 · 10⁻⁴ (rad/s)². Each variance on the diagonal of `linear_acceleration_covariance` is [`LINEAR_ACCELERATION_VARIANCE`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L60), 10⁻² (m/s²)². Both have the size of the firmware's variances.
    - **Valid and invalid samples:** the tests override the defaults of `stamp`, `frame_id` or `yaw_rate`, or change a field of the returned message, to build the samples that they need. A later stamp gives the next valid sample, for example. An invalid sample differs from the valid one in one property, so that the sample aims at one check. A sample can fail a later check too, as sample A of [the time limits](#check-4-the-two-time-limits) does.
    - **`seconds()`:** turns a number of seconds into a `rospy.Duration`, which a test adds to `NOW` or subtracts from it.

### `ValidatorTestCase`: a new validator and two assertions

[`TestImuValidator`](#testimuvalidator-which-samples-are-valid) and [`TestSecondsSinceLastValidSample`](#testsecondssincelastvalidsample-the-silence) derive from [`ValidatorTestCase`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L92-L110). Its [`setUp()`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L100-L101) gives each test a new validator. Its assertions [`assert_accepted()`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L103-L105) and [`assert_rejected()`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L107-L110) call [`drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample) once, as the node does for each sample, and check that the validator accepts the sample or rejects it with the expected reason.

??? example "`ValidatorTestCase` in the code"

    [`TestImuValidator`](#testimuvalidator-which-samples-are-valid) and [`TestSecondsSinceLastValidSample`](#testsecondssincelastvalidsample-the-silence) derive from the class [`ValidatorTestCase`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L92-L110), which derives from [`unittest.TestCase`](https://docs.python.org/3/library/unittest.html#unittest.TestCase). A class that derives from another class inherits the other class's methods. So the tests of both classes can use the three methods of `ValidatorTestCase`: [`setUp()`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L100-L101) and the two assertions [`assert_accepted()`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L103-L105) and [`assert_rejected()`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L107-L110). `ValidatorTestCase` itself has no tests. The excerpt shows the class, from [`test_imu_validator.py`, lines 92–110]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L92-L110):

    ```python title="test_imu_validator.py: ValidatorTestCase" linenums="92"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:92:110"
    ```

    - **`setUp()`:** unittest calls `setUp()` before each test method. So each test of a class that derives from `ValidatorTestCase` starts with a new [`ImuValidator`](#what-the-tests-call) in `self.validator`, one that has seen no sample yet. No test depends on the samples of another test. [`FRAME`](#the-constants-and-the-helpers), [`MAX_AGE`](#the-constants-and-the-helpers) and [`MAX_FUTURE`](#the-constants-and-the-helpers) are the values that [`ekf.launch`]({{ ekf_test_src_url }}/diffbot_control/launch/ekf.launch) gives the validator's node.
    - **`assert_accepted(message, now)`:** calls [`drop_reason_or_record(message, now)`](#drop_reason_or_record-the-decision-about-a-sample) once, the same method that the node calls once for each sample, and checks the result with [`assertIsNone()`](https://docs.python.org/3/library/unittest.html#unittest.TestCase.assertIsNone). A result of `None` means that the sample has no drop reason: the validator accepts the sample and stores the sample's stamp in `last_valid_stamp` and its arrival time in `last_valid_arrival_time`. For any other result, `assertIsNone()` stops the test, and unittest reports the test as failed, with the drop reason that came back instead of `None`.
    - **`assert_rejected(reason, message, now)`:** calls `drop_reason_or_record(message, now)` once too, and compares the result with the expected `reason` by [`assertEqual()`](https://docs.python.org/3/library/unittest.html#unittest.TestCase.assertEqual). `assertEqual()` compares its first two arguments and fails the test when they differ. The tests of this file give `assertEqual()` the expected value first and the result second, and `assert_rejected()` does the same. So the assertion holds when the validator rejects the sample with the expected reason. The validator doesn't store a rejected sample's stamp or arrival time in `last_valid_stamp` and `last_valid_arrival_time`. Only the clock check, which runs before the checks at any ROS time other than 0, stores the call's ROS time in `last_checked_ros_time`.
    - **`msg`:** the optional last argument of both assertions. The type `Optional[str]` says that `msg` is a text or `None`, and `= None` makes `None` the default. When the assertion fails, unittest adds the text to its report of the failure. The tests of [check 1](#check-1-the-frame), [check 2](#check-2-the-values) and [check 3](#check-3-the-variances) run a loop over several cases and give each assertion a text that names the case, so that a failure says which case the validator decided wrongly.

    `assertIsNone()` and `assertEqual()` are two of the [assert methods](https://docs.python.org/3/library/unittest.html#assert-methods) that every test class inherits from `unittest.TestCase`, which is why the two assertions call these methods on `self`. The tests call `assert_accepted()` and `assert_rejected()` on `self` in the same way.

    Accepted and rejected are the validator's decisions, and the tests' names say "accepts" and "rejects". The node forwards an accepted sample and drops a rejected one. The tests themselves publish nothing, as [What the tests don't show](#what-the-tests-dont-show) says.

## `TestImuValidator`: which samples are valid

The class tests [`drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample), one reason to drop a sample at a time, in the order of the checks. Each test starts with a new validator.

??? example "The class in the code"

    The class tests [`drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample), one reason to drop a sample at a time. The excerpt shows the class's first two lines, from [`test_imu_validator.py`, lines 113–114]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L113-L114). The name in parentheses, [`ValidatorTestCase`](#validatortestcase-a-new-validator-and-two-assertions), is the class that [`TestImuValidator`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L113-L213) derives from. So each test starts with a new validator and checks each decision of the validator with [`assert_accepted()`](#validatortestcase-a-new-validator-and-two-assertions) or [`assert_rejected()`](#validatortestcase-a-new-validator-and-two-assertions):

    ```python title="test_imu_validator.py: the class derives from ValidatorTestCase" linenums="113"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:113:114"
    ```

    The sections below follow the order of the checks, with the numbers and names of [the five checks](../../robot_localization.md#the-five-checks) on the Localization page.

### A valid sample

[`test_validator_accepts_valid_samples`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L116-L118) is the class's baseline. The validator accepts a sample stamped [`NOW`](#the-constants-and-the-helpers) at `NOW`, and then the IMU's next sample at 50 Hz, stamped and arriving 0.02 s later.

??? example "The test"

    [`test_validator_accepts_valid_samples`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L116-L118) is the baseline of the class. The excerpt shows the test, from [`test_imu_validator.py`, lines 116–118]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L116-L118):

    ```python title="test_imu_validator.py: valid samples" linenums="116"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:116:118"
    ```

    The test's first line holds two calls, one inside the other. Python runs the inner call first:

    1. **The inner call:** [`make_imu_sample()`](#the-constants-and-the-helpers) returns a valid sample, stamped [`NOW`](#the-constants-and-the-helpers).
    2. **The outer call:** [`self.assert_accepted(make_imu_sample(), NOW)`](#validatortestcase-a-new-validator-and-two-assertions) gets two arguments. The first is the sample from the inner call. The second is the ROS time $t$ of the sample's arrival, `NOW`, 100 s. The assertion [`assert_accepted()`](#validatortestcase-a-new-validator-and-two-assertions) calls [`drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample) with both and checks that the method returns `None`, no drop reason. Otherwise the test stops there, and unittest reports the test as failed, with the drop reason that came back instead of `None`.

    So the first line checks that the validator accepts the sample. In the node, [`on_sample()`](#where-the-ros-time-comes-from) forwards an accepted sample.

    The second line checks a second sample in the same way. The sample is stamped [`MAX_FUTURE`](#the-constants-and-the-helpers), 0.02 s, later than the first, and arrives at a ROS time that is 0.02 s later too. This sample is the next one of the IMU at 50 Hz. The validator accepts the sample, because its stamp is later than the first sample's.

### Before the checks: no clock

At the ROS time 0, the validator has no clock yet. It rejects every sample with [`NO_CLOCK`](#dropreason-the-reasons-to-drop-a-sample) before the checks, because a stamp can't be judged against this time. A second test checks that such a sample leaves `last_valid_stamp` as it is.

??? example "The two tests"

    [`test_validator_rejects_a_sample_at_ros_time_zero`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L120-L121) gives the validator a valid sample at the ROS time 0, the test's constant [`ROS_TIME_ZERO`](#the-constants-and-the-helpers), [`rospy.Time(0)`](https://github.com/ros/ros_comm/blob/1.17.0/clients/rospy/src/rospy/rostime.py#L95). The excerpt shows this test and the next one, from [`test_imu_validator.py`, lines 120–134]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L120-L134):

    ```python title="test_imu_validator.py: no clock, and what a sample at 0 s leaves" linenums="120"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:120:134"
    ```

    [`assert_rejected()`](#validatortestcase-a-new-validator-and-two-assertions) gets the expected reason first, here [`imu_validator.DropReason.NO_CLOCK`](#dropreason-the-reasons-to-drop-a-sample), then the sample and the ROS time. So the first test checks that the validator rejects even a valid sample at the ROS time 0, with the reason [`NO_CLOCK`](#dropreason-the-reasons-to-drop-a-sample).

    The validator treats the ROS time 0 as an unavailable clock. At startup with simulation time, [`rospy.Time.now()`](https://github.com/ros/ros_comm/blob/1.17.0/clients/rospy/src/rospy/rostime.py#L144) returns 0 until the node has received the first message on the topic `/clock`, as [Where the ROS time comes from](#where-the-ros-time-comes-from) explains. A stamp can't be judged against this time. At $t = 0$, check 4 would allow only the stamps between −0.1 s and 0.02 s:

    $$
    t - \texttt{max\_age} \le t_s \le t + \texttt{max\_future}
    \quad\Rightarrow\quad
    -0.1\ \text{s} \le t_s \le 0.02\ \text{s}
    $$

    - **A sample from Gazebo:** a valid sample stamped, for example, 0.5 s would be rejected as from the future. The log and the diagnostics would blame the sample's stamp, although the node's clock is missing.
    - **A sample without a stamp:** a message whose stamp was never set has the stamp 0 s, and would pass check 4.

    So the validator rejects every sample at the ROS time 0 before the checks, with its own reason `NO_CLOCK`, which [`drop_reason()`](#step-1-drop_reason-the-decision) returns. On the robot, the ROS time is the SBC's system clock, which doesn't read 0. So the robot's validator doesn't reject samples for this reason.

    [`test_validator_rejects_the_same_stamp_again_after_a_sample_at_ros_time_zero`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L123-L134) checks that a sample at the ROS time 0 leaves `last_valid_stamp` as it is, because [`drop_reason()`](#step-1-drop_reason-the-decision) returns `NO_CLOCK` before the clock check. The test gives one validator three samples:

    1. **The 1st call, at the ROS time [`NOW`](#the-constants-and-the-helpers), 100 s:** a sample stamped `NOW` is valid. The validator stores 100 s in its attributes `last_valid_stamp` and `last_checked_ros_time`.
    2. **The 2nd call, at `ROS_TIME_ZERO`, 0 s:** a sample stamped [`SHORTLY_BEFORE`](#the-constants-and-the-helpers) before `NOW`, 99.99 s. The validator returns `NO_CLOCK` before the clock check. Both attributes stay 100 s.
    3. **The 3rd call, at `NOW` plus [`ONE_MILLISECOND`](#the-constants-and-the-helpers), 100.001 s:** a sample stamped `NOW` again. The clock check finds 100.001 s later than `last_checked_ros_time`, 100 s, and stores 100.001 s. `last_valid_stamp` is still 100 s, so [check 5](#check-5-the-order-of-the-stamps) rejects the sample as [`NOT_NEWER`](#dropreason-the-reasons-to-drop-a-sample).

    The third call's own clock check finds no clock that went back, because its ROS time is not earlier than `NOW`. So only the second call could have changed `last_valid_stamp`. A validator that checked the clock before it returned `NO_CLOCK` would take 0 s for a clock that went back and set its attributes `last_valid_stamp` and `last_valid_arrival_time` to `None`. A validator that stored the second sample's stamp would compare the third stamp with 99.99 s. Either would accept the third sample, and the test fails for both. [Two tests of the silence](#after-a-sample-at-the-ros-time-0) check the other two attributes.

### Check 1: the frame

A sample in the frame `base_link` or in an empty frame is rejected as [`WRONG_FRAME`](#dropreason-the-reasons-to-drop-a-sample).

??? example "The test"

    Check 1 compares the sample's `header.frame_id` with the frame that the validator expects, [`FRAME`](#the-constants-and-the-helpers), `imu_link`. The excerpt shows the test, from [`test_imu_validator.py`, lines 136–140]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L136-L140):

    ```python title="test_imu_validator.py: a wrong frame" linenums="136"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:136:140"
    ```

    The loop runs twice, with the frame `base_link` and with an empty frame. In each pass, [`assert_rejected()`](#validatortestcase-a-new-validator-and-two-assertions) expects [`DropReason.WRONG_FRAME`](#dropreason-the-reasons-to-drop-a-sample). The last argument is the assertion's `msg`, the text that unittest adds to its report when the assertion fails. The text names the frame of the loop's current pass, so a failure says which of the two frames the validator didn't reject.

### Check 2: the values

A sample with NaN, +∞ or −∞ as its yaw rate or as its acceleration is rejected as [`NOT_FINITE`](#dropreason-the-reasons-to-drop-a-sample).

??? example "The test"

    Check 2 needs finite values in the angular velocity $\boldsymbol\omega$, the field `angular_velocity`, and in the accelerometer's reading $\mathbf f$, the field `linear_acceleration`. The excerpt shows the test, from [`test_imu_validator.py`, lines 142–151]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L142-L151):

    ```python title="test_imu_validator.py: NaN and infinite values" linenums="142"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:142:151"
    ```

    The loop runs three times, for NaN, +∞ and −∞. Each pass builds two samples. The first has the value as its yaw rate $\omega_z$, `angular_velocity.z`. The second has the value as $f_x$, `linear_acceleration.x`. The validator has to reject all six samples with [`DropReason.NOT_FINITE`](#dropreason-the-reasons-to-drop-a-sample).

### Check 3: the variances

Three samples, each with one defect in the angular velocity's covariance, are rejected as [`INVALID_ANGULAR_VELOCITY_COVARIANCE`](#dropreason-the-reasons-to-drop-a-sample): an unavailable angular velocity, a zero variance of the yaw rate, and NaN off the diagonal.

??? example "The test"

    Check 3 looks at the covariance $\boldsymbol\Sigma_{\boldsymbol\omega}$ of the angular velocity $\boldsymbol\omega$, the field [`angular_velocity_covariance`](../../robot_localization.md#the-messages-and-their-fields). Its nine values have to be finite, and its three variances on the diagonal, $\sigma^2_{\omega_x}$, $\sigma^2_{\omega_y}$ and $\sigma^2_{\omega_z}$, have to be positive. [The variances](../../robot_localization.md#the-variances) on the Localization page shows how the message stores the 3 × 3 matrix as nine numbers, row by row. The excerpt shows the test, from [`test_imu_validator.py`, lines 153–164]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L153-L164):

    ```python title="test_imu_validator.py: three invalid covariances" linenums="153"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:153:164"
    ```

    The test builds three samples, each with one defect in $\boldsymbol\Sigma_{\boldsymbol\omega}$, and expects [`DropReason.INVALID_ANGULAR_VELOCITY_COVARIANCE`](#dropreason-the-reasons-to-drop-a-sample) for each. The text in each pair of `cases` is the `msg` of a failure:

    - **`unavailable`:** the first element is −1 and the others are 0. This is how a [`sensor_msgs/Imu`](../../robot_localization.md#the-messages-and-their-fields) message says that it carries no angular velocity. Check 3 reads the three variances $\sigma^2_{\omega_x}$, $\sigma^2_{\omega_y}$ and $\sigma^2_{\omega_z}$ as −1, 0 and 0 (rad/s)², so none of them is positive.
    - **`zero_variance`:** the variance of the yaw rate $\sigma^2_{\omega_z}$, the element with the index [`YAW_RATE_VARIANCE_INDEX`](#the-constants-and-the-helpers), 8, is 0 (rad/s)².
    - **`nan_off_the_diagonal`:** the covariance $\sigma_{\omega_x \omega_y}$, the element with the index [`OFF_DIAGONAL_INDEX`](#the-constants-and-the-helpers), 1, is NaN. This element is not a variance, so the test shows that the check looks at all nine values.

### Check 4: the two time limits

Check 4 compares the stamp $t_s$ with the ROS time $t$: $t - \texttt{max\_age} \le t_s \le t + \texttt{max\_future}$. Two tests probe each limit from both sides. The excerpt shows both tests, from [`test_imu_validator.py`, lines 166–174]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L166-L174):

```python title="test_imu_validator.py: at the limits and 1 µs beyond" linenums="166"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:166:174"
```

The figure shows the samples of these two tests on a time axis. The sections on check 5 and on a clock that goes back have figures of the same kind:

--8<-- "includes/localization-test-validator-limits.html"

The validator has to accept B and C, so each limit itself is still valid, and reject A as [`TOO_OLD`](#dropreason-the-reasons-to-drop-a-sample) and D as [`FROM_THE_FUTURE`](#dropreason-the-reasons-to-drop-a-sample).

??? note "How to read the three time-axis figures"

    The figure above shows the samples of these two tests on a time axis. The sections on [check 5](#check-5-the-order-of-the-stamps) and on [a clock that goes back](#a-clock-that-goes-back) have figures of the same kind. Read the three figures as follows:

    - **The axis:** time runs to the right. One tick is 0.02 s, the IMU's sample period at 50 Hz. Each axis has its own scale.
    - **A mark:** stands at the stamp $t_s$ of its sample. Its letter is the sample's name in the text. A solid green mark is a sample that the validator has to accept, a dashed red mark one that the validator has to reject.
    - **The bands:** a stamp in a green band passes the check. A stamp in a red band is rejected for the reason that the band names.
    - **Two slanted strokes:** the axis leaves out a stretch of time there.
    - **The list below a figure:** says for each sample what the node does with the sample, and why. "Forwarded" stands for a sample that the validator accepts, "dropped" for one that the validator rejects.

    Two samples that lie only 1 µs or 1 ns apart would share one spot at the figure's scale, so the figures draw them apart.

??? note "The stamps from the constants, and why A is too old and not "not newer""

    The first test sends B and then A, the second test D and then C, all at the ROS time [`NOW`](#the-constants-and-the-helpers). The tests build the four stamps from the constants: B is `NOW - seconds(MAX_AGE)`, A is `NOW - seconds(JUST_TOO_OLD)`, D is `NOW + seconds(JUST_TOO_FAR_AHEAD)` and C is `NOW + seconds(MAX_FUTURE)`. The validator has to accept B and C, so each limit itself is still valid. The validator has to reject A with [`DropReason.TOO_OLD`](#dropreason-the-reasons-to-drop-a-sample) and D with [`DropReason.FROM_THE_FUTURE`](#dropreason-the-reasons-to-drop-a-sample).

    In the first test, sample A is also older than the valid sample B before it. The validator reports `DropReason.TOO_OLD` and not [`DropReason.NOT_NEWER`](#dropreason-the-reasons-to-drop-a-sample), because the checks run [in their order](#step-1-drop_reason-the-decision) and check 4 comes before check 5. The second test sends the rejected sample D first. For a rejected sample at a ROS time other than 0, the validator stores only the ROS time of the call in `last_checked_ros_time`, for its clock check, so D doesn't change what the validator decides for C.

### Check 5: the order of the stamps

Check 5 lets a stamp through only when it is later than $t_\text{last}$, the stamp of the last valid sample. After a valid sample stamped [`NOW`](#the-constants-and-the-helpers), a sample with the same stamp or an earlier one is rejected as [`NOT_NEWER`](#dropreason-the-reasons-to-drop-a-sample), a sample 1 ns later is accepted, and a rejected sample from the future doesn't become $t_\text{last}$:

--8<-- "includes/localization-test-validator-order.html"

??? example "The two tests"

    Check 5 compares the stamp $t_s$ with $t_\text{last}$, the stamp of the last valid sample: $t_s > t_\text{last}$. The excerpt shows the two tests of this check, from [`test_imu_validator.py`, lines 176–194]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L176-L194):

    ```python title="test_imu_validator.py: the order check" linenums="176"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:176:194"
    ```

    [`test_validator_rejects_a_sample_not_later_than_the_last_valid_one`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L176-L186) sends four samples to one validator:

    1. A valid sample stamped [`NOW`](#the-constants-and-the-helpers). The validator stores `NOW` in `last_valid_stamp`, as $t_\text{last}$.
    2. Sample F, stamped `NOW` again. The validator rejects the sample with [`DropReason.NOT_NEWER`](#dropreason-the-reasons-to-drop-a-sample), because F's stamp is not later than $t_\text{last}$. A stamp is valid only once, until the clock goes back.
    3. Sample E, stamped [`SHORTLY_BEFORE`](#the-constants-and-the-helpers), 0.01 s, before `NOW`. `SHORTLY_BEFORE` is a tenth of [`MAX_AGE`](#the-constants-and-the-helpers), so check 4 lets E through, and check 5 rejects the sample with the same reason.
    4. Sample G, stamped [`ONE_NANOSECOND`](#the-constants-and-the-helpers), 1 ns, after `NOW`, the smallest step of a ROS stamp. G arrives [`ONE_MILLISECOND`](#the-constants-and-the-helpers), 1 ms, after `NOW`, so after its own stamp. A ROS time before `NOW` would be a clock that went back and would clear `last_valid_stamp`. The validator accepts the sample.

    [`test_validator_accepts_a_new_sample_after_rejecting_a_future_one`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L188-L194) shows what $t_\text{last}$ is not. After a valid sample stamped `NOW`, the test sends sample H, stamped [`FAR_AHEAD`](#the-constants-and-the-helpers), 5 s, ahead. Check 4 rejects H with [`DropReason.FROM_THE_FUTURE`](#dropreason-the-reasons-to-drop-a-sample). Then the test sends sample I, stamped 0.02 s after `NOW`. The validator accepts I. If the validator stored the stamp of the rejected sample H in `last_valid_stamp`, every sample of the next 5 s would count as not newer.

### A clock that goes back

After the ROS time went back, the order check starts over, but check 4 judges the stamps against the new ROS time. The figure shows the two tests, each on its own axis at the new ROS time:

--8<-- "includes/localization-test-validator-clock-back.html"

??? example "The two tests"

    As [`clear_last_valid_sample_times_if_clock_went_back()`](#clear_last_valid_sample_times_if_clock_went_back-a-clock-that-goes-back) explains, the validator sets its attributes `last_valid_stamp` and `last_valid_arrival_time` to `None` when a call checks the clock and the call's `now` is earlier than `last_checked_ros_time`, the ROS time of the last clock check. Every diagnostics report checks the clock, and so does every sample that arrives at a ROS time other than 0. A sample at the ROS time 0 returns [`NO_CLOCK`](#dropreason-the-reasons-to-drop-a-sample) before the clock check and changes nothing. After the validator has cleared `last_valid_stamp` and `last_valid_arrival_time`, the order check, [check 5](#check-5-the-order-of-the-stamps), starts over. The excerpt shows the two tests, from [`test_imu_validator.py`, lines 196–213]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L196-L213):

    ```python title="test_imu_validator.py: the clock goes back" linenums="196"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:196:213"
    ```

    - **[`test_validator_accepts_a_stamp_once_after_the_clock_goes_back`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L196-L204):** the first sample is stamped [`NOW`](#the-constants-and-the-helpers), 100 s, and arrives at the ROS time `NOW`. The validator accepts this sample. Then the ROS time is [`RESTART_NEAR_ZERO`](#the-constants-and-the-helpers), 0.02 s, as after a reset of the simulation. This test is the first axis of the figure. Sample J is stamped `RESTART_NEAR_ZERO` and arrives at the ROS time `RESTART_NEAR_ZERO`. This stamp is far older than $t_\text{last}$, 100 s. The validator accepts J all the same, because the order check started over. The order check works again from this sample on: the test sends the same stamp once more, [`ONE_MILLISECOND`](#the-constants-and-the-helpers), 1 ms, later, and expects [`DropReason.NOT_NEWER`](#dropreason-the-reasons-to-drop-a-sample). This second sample is K.
    - **[`test_validator_rejects_stamps_outside_time_limits_after_the_clock_goes_back`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L206-L213):** starting over applies to the order check only. The first sample is again stamped `NOW`, 100 s, and arrives at the ROS time `NOW`. Then the ROS time is [`RESTART`](#the-constants-and-the-helpers), 50 s, and both of the next samples arrive at this ROS time. This test is the second axis of the figure. Sample M is stamped 100.02 s, `NOW` plus [`MAX_FUTURE`](#the-constants-and-the-helpers). This stamp was valid before the clock went back and is 50.02 s ahead of the new ROS time. The validator rejects M with [`DropReason.FROM_THE_FUTURE`](#dropreason-the-reasons-to-drop-a-sample). Sample L is stamped 49 s, [`ONE_SECOND`](#the-constants-and-the-helpers), 1 s, before `RESTART`, and the validator rejects L with [`DropReason.TOO_OLD`](#dropreason-the-reasons-to-drop-a-sample).

## `TestSecondsSinceLastValidSample`: the silence

The [validator's diagnostics](../../robot_localization.md#the-validators-diagnostics) report an error when no sample was forwarded for too long. The time behind this error comes from [`seconds_since_last_valid_sample()`](#seconds_since_last_valid_sample-the-silence), which returns the silence $t - t_\text{valid}$ in s, or `None` when no sample was valid since the clock last went back.

Nine tests in four groups check the silence. Each figure draws each test's calls as rows from top to bottom, above the ROS time.

??? note "How to read the silence figures"

    Read the drawings as follows:

    - **A row:** one call of the test, in the order of the test's lines, from top to bottom. The list below the drawing gives each call with the same number, the test's names for its times and their values.
    - **A circle:** a sample that the test gives the validator. A solid green circle is a sample that the validator accepts. A dashed red circle is a sample that the validator rejects, with the drop reason beside it.
    - **A square:** a call of `seconds_since_last_valid_sample()`, with `None` beside it or the silence that it returns.
    - **The axis:** the ROS time $t$ of the calls. Each mark stands at the ROS time of its call. Two slanted strokes leave out a stretch of time, and each part of the axis between them has its own scale.
    - **A triangle:** a sample's stamp $t_s$, where it differs from the ROS time of the sample's arrival. A dotted line joins the triangle to the sample's mark.
    - **A green line:** `last_valid_arrival_time`, $t_\text{valid}$. The line runs down from the valid sample that stored it. A cross ends the line above the row of the call that sets the attribute to `None`.
    - **A dashed gray line:** `last_checked_ros_time`, the ROS time of the last clock check. The line runs down from each call that checks the clock to the next call that checks it. A sample at the ROS time 0 doesn't check the clock, so the line passes its row. Where the dashed line and the green line stand at the same time, the dashed line lies on the green one.
    - **An arrow to the left, "back":** the call's ROS time is earlier than `last_checked_ros_time`, so the clock went back.
    - **A dimension line:** the silence $t - t_\text{valid}$ that a call of `seconds_since_last_valid_sample()` returns, with the test's name for the expected value.

??? example "The class and its helper `assert_silence()`"

    The class derives from [`ValidatorTestCase`](#validatortestcase-a-new-validator-and-two-assertions) too. So each test starts with a new validator, and the tests give the validator their samples through [`assert_accepted()`](#validatortestcase-a-new-validator-and-two-assertions) and [`assert_rejected()`](#validatortestcase-a-new-validator-and-two-assertions). The class adds a helper of its own, [`assert_silence()`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L219-L224), for the comparison of the silence. The excerpt shows the class's first lines and the helper, from [`test_imu_validator.py`, lines 216–224]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L216-L224):

    ```python title="test_imu_validator.py: the class and assert_silence()" linenums="216"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:216:224"
    ```

    `assert_silence(expected, now)` calls `seconds_since_last_valid_sample(now)` and compares the result with `expected`. If the method returns `None`, `assert_silence()` fails the test with a message that names the expected silence. Otherwise [`assertAlmostEqual()`](https://docs.python.org/3/library/unittest.html#unittest.TestCase.assertAlmostEqual) rounds the difference of the two numbers to seven decimal places, its default, and expects zero.

### Before and after a valid sample

The first three tests show where the silence counts from. The first test asks a new validator for the silence. The second asks after one valid sample, and the third after a valid sample and a rejected one:

--8<-- "includes/localization-test-validator-silence-grows.html"

??? example "The three tests"

    The excerpt shows the three tests, from [`test_imu_validator.py`, lines 226–242]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L226-L242):

    ```python title="test_imu_validator.py: the silence grows" linenums="226"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:226:242"
    ```

    - **Before the first valid sample:** the method returns `None`.
    - **After a valid sample:** the valid sample is stamped [`SHORTLY_BEFORE`](#the-constants-and-the-helpers), 0.01 s, before it arrives at [`NOW`](#the-constants-and-the-helpers). [`SILENCE`](#the-constants-and-the-helpers), 3.5 s, after the arrival, the method returns 3.5 s and not 3.51 s, because the silence counts from the arrival and not from the stamp. `SILENCE` has a fraction of a second, so a silence counted in whole seconds would fail too.
    - **While samples are rejected:** a sample that the validator rejects with [`DropReason.TOO_OLD`](#dropreason-the-reasons-to-drop-a-sample) doesn't end the silence. The rejected sample is stamped [`ONE_SECOND`](#the-constants-and-the-helpers) after `NOW`, later than the valid sample's stamp, so [check 5](#check-5-the-order-of-the-stamps) would let it through. The sample arrives `SILENCE` after `NOW`, when its stamp is 2.5 s old, more than [`MAX_AGE`](#the-constants-and-the-helpers). So check 4 rejects the sample before check 5 runs. At the same ROS time, `SILENCE`, 3.5 s, after the valid sample, the method returns 3.5 s.

### After a sample at the ROS time 0

A sample at the ROS time 0 returns [`NO_CLOCK`](#dropreason-the-reasons-to-drop-a-sample) before the clock check and leaves the validator's attributes as they are. The two tests check `last_valid_arrival_time` and `last_checked_ros_time`:

--8<-- "includes/localization-test-validator-silence-zero.html"

??? example "The two tests"

    A sample at the ROS time 0 returns [`NO_CLOCK`](#dropreason-the-reasons-to-drop-a-sample) before the clock check and leaves the validator's attributes as they are. A test in [Before the checks: no clock](#before-the-checks-no-clock) checks this for `last_valid_stamp`. The next two tests check the other two attributes, `last_valid_arrival_time` and `last_checked_ros_time`. Both give the validator the same two samples, a valid one at [`NOW`](#the-constants-and-the-helpers) and one at the ROS time 0. They differ in the ROS time at which they ask for the silence:

    The excerpt shows the two tests, from [`test_imu_validator.py`, lines 244–260]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L244-L260):

    ```python title="test_imu_validator.py: the silence after a sample at 0 s" linenums="244"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:244:260"
    ```

    - **The arrival time:** a valid sample arrives at `NOW`, and a sample stamped 99.99 s, [`SHORTLY_BEFORE`](#the-constants-and-the-helpers) before `NOW`, returns `NO_CLOCK` at [`ROS_TIME_ZERO`](#the-constants-and-the-helpers). At 100.1 s, [`RECENT`](#the-constants-and-the-helpers) after `NOW`, the method returns 0.1 s, counted from the arrival at `NOW`. A validator that cleared `last_valid_arrival_time` at 0 s would return `None`, and one that stored the stamp 99.99 s there would return 0.11 s.
    - **The last clock check:** the same two calls, then the method is asked at 99.999 s, [`ONE_MILLISECOND`](#the-constants-and-the-helpers) before `NOW`. `last_checked_ros_time` is still 100 s, so the clock went back, and the method returns `None`. A validator that stored 0 s or the stamp 99.99 s as the last clock check would find no clock that went back and return −0.001 s.

### After the clock goes back

The next three tests are about a clock that goes back while no valid sample arrives. In each test, a valid sample arrives at [`NOW`](#the-constants-and-the-helpers), and a later call has an earlier ROS time:

--8<-- "includes/localization-test-validator-silence-clock-back.html"

??? example "The three tests"

    The excerpt shows the three tests, from [`test_imu_validator.py`, lines 262–288]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L262-L288):

    ```python title="test_imu_validator.py: the silence after the clock goes back" linenums="262"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:262:288"
    ```

    - **Without samples:** a valid sample arrives at the ROS time [`NOW`](#the-constants-and-the-helpers), 100 s. Then the method is asked at the ROS time [`RESTART`](#the-constants-and-the-helpers), 50 s, and [`SILENCE`](#the-constants-and-the-helpers) later, at 53.5 s, and returns `None` both times. The method itself notices that the clock went back, because the method checks the clock with [`clear_last_valid_sample_times_if_clock_went_back()`](#clear_last_valid_sample_times_if_clock_went_back-a-clock-that-goes-back) in each call.
    - **Back by less than the silence:** a valid sample arrives at the ROS time `NOW`, 100 s. The method is asked at the ROS time 103.5 s, `SILENCE` after `NOW`, and returns a silence of 3.5 s. Then the method is asked at 100.1 s, `NOW` plus [`RECENT`](#the-constants-and-the-helpers). The clock went back by 3.4 s, less than the silence, so the new ROS time is still later than the arrival of the valid sample, and $t - t_\text{valid}$ would be 0.1 s, a silence that [`health()`](#testhealth-the-status) reports as OK. The method returns `None` all the same, and at 101 s, [`ONE_SECOND`](#the-constants-and-the-helpers) after `NOW`, as well. A sample from before the clock went back says nothing about the IMU in the new time.
    - **Shown by a rejected sample:** a valid sample arrives at the ROS time `NOW`, 100 s, and the method returns a silence of 3.5 s at 103.5 s. Then the clock goes back. The first call in the new time is a sample that arrives at 100.1 s, `NOW` plus `RECENT`, stamped 99 s, `ONE_SECOND` before `NOW`, which the validator rejects as too old. The validator checks the clock also for a sample that a check rejects, so in this call the validator sets its attributes `last_valid_stamp` and `last_valid_arrival_time` to `None`. The method returns `None` at 104.5 s, `SILENCE` plus `ONE_SECOND` after `NOW`. This last call is later than the call at 103.5 s, so its own clock check finds no clock that went back. Only the rejected sample's call can have cleared the two attributes. A sample at the ROS time 0 would not show that the clock went back: the validator rejects such a sample before the clock check.

### The first valid sample after the clock went back

The last test of the class follows the silence through a whole restart, from a valid sample before the clock goes back to the first valid sample after it:

--8<-- "includes/localization-test-validator-silence-restart.html"

??? example "The test"

    The excerpt shows the test, from [`test_imu_validator.py`, lines 290–303]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L290-L303):

    ```python title="test_imu_validator.py: the silence starts again" linenums="290"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:290:303"
    ```

    A valid sample arrives at the ROS time [`NOW`](#the-constants-and-the-helpers), 100 s. Then the ROS time is [`RESTART`](#the-constants-and-the-helpers), 50 s. At this ROS time, the validator rejects a sample from the future and a sample with the wrong frame. At 51 s, [`ONE_SECOND`](#the-constants-and-the-helpers) after `RESTART`, the method still returns `None`. At 53.5 s, [`SILENCE`](#the-constants-and-the-helpers) after `RESTART`, a valid sample arrives. At 53.6 s, [`RECENT`](#the-constants-and-the-helpers), 0.1 s, after that sample, the method returns 0.1 s.

## `TestHealth`: the status

[`health()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L143-L160) turns the [silence](#seconds_since_last_valid_sample-the-silence) and the number of samples dropped since the report before into the level of the diagnostics status. A silence error comes before the warning about new drops. The table shows the level that each of the five tests of [`TestHealth`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L306-L331) expects:

| Silence | New drops | Level that the test expects |
|:--|:--|:--|
| `None`: no valid sample yet | 0 | [`ERROR`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/diagnostic_msgs/msg/DiagnosticStatus.msg#L7) |
| [`SILENCE_LIMIT`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L101), 1 s | 0 | [`OK`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/diagnostic_msgs/msg/DiagnosticStatus.msg#L5) |
| `SILENCE_LIMIT` plus [`ONE_MILLISECOND`](#the-constants-and-the-helpers), 1.001 s | 0 | `ERROR` |
| [`RECENT`](#the-constants-and-the-helpers), 0.1 s | [`NEW_DROP_COUNT`](#the-constants-and-the-helpers), 3 | [`WARN`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/diagnostic_msgs/msg/DiagnosticStatus.msg#L6), and the message contains the number 3 |
| `RECENT`, 0.1 s | 0 | `OK` |
| `None`, and 1.001 s | 3 | `ERROR` for both: silence comes before the warning |

One test checks the limit of the silence. The excerpt shows it, from [`test_imu_validator.py`, lines 312–316]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L312-L316):

```python title="test_imu_validator.py: the silence limit and 1 ms beyond" linenums="312"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:312:316"
```

With no new drops, a silence of exactly `SILENCE_LIMIT`, 1 s, is still `OK`, and 1 ms more is `ERROR`. The test reads the limit from [`imu_validator`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py), so it doesn't repeat its value. [The validator's diagnostics](../../robot_localization.md#the-validators-diagnostics) says what the levels mean on the robot.

??? example "The function `health()`"

    `health()` returns a [`Health`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L126-L130), a named tuple of the level and the message. The node calls the function once for each diagnostics report, with the numbers of one [snapshot](#testsamplecounts-the-numbers-of-a-report). The excerpt shows the function, from [`imu_validator.py`, lines 143–160]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L143-L160):

    ```python title="imu_validator.py: health()" linenums="143"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:143:160"
    ```

    The function looks at three conditions in their order and returns at the first one that holds:

    1. **No valid sample:** the silence is `None`, before the first valid sample or since the clock went back. The function returns `ERROR`.
    2. **A silence that is too long:** the silence is longer than `SILENCE_LIMIT`, 1 s. The function returns `ERROR`, with the silence in the message.
    3. **New drops:** `new_drop_count` is more than 0. The function returns `WARN`, with the number in the message.

    When none of the three holds, the function returns `OK`. The function reads no clock and keeps no state, so each call is independent of the others.

??? example "All five tests"

    The excerpt shows the class's five tests, from [`test_imu_validator.py`, lines 309–331]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L309-L331):

    ```python title="test_imu_validator.py: the level of the status" linenums="309"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:309:331"
    ```

## `TestSampleCounts`: the numbers of a report

[`SampleCounts`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L163-L190) counts the forwarded and the dropped samples. For each diagnostics report, [`take_snapshot()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L180-L190) returns the numbers of one report, among them `new_drop_count`, the samples dropped since the report before. The four tests:

- **[`test_snapshot_has_the_forwarded_count_and_the_drops_by_reason`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L340-L350):** the counts by reason, and the silence that the test passed.
- **[`test_snapshot_counts_a_dropped_sample_as_new_only_once`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L352-L358):** a dropped sample is new in the next snapshot and not in the one after.
- **[`test_snapshot_counts_a_dropped_sample_as_new_once_also_in_an_error_report`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L360-L368):** the same when [`health()`](#testhealth-the-status) reports an error.
- **[`test_snapshot_keeps_its_numbers_when_more_samples_arrive`]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L370-L376):** a report's numbers all come from one moment.

??? example "`SampleCounts` and `take_snapshot()`"

    [`SampleCounts`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L163-L190) counts the forwarded samples and the dropped samples by their reason, without ROS communication. The node's callback [`on_sample()`](#where-the-ros-time-comes-from) calls [`count_forwarded()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L172-L174) for each forwarded sample and [`count_dropped()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L176-L178) for each dropped one. For each diagnostics report, the node calls the method [`take_snapshot()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L180-L190), which returns the numbers of one report as a [`DiagnosticSnapshot`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L133-L140). This named tuple has four fields: `forwarded_count`, `drops_by_reason`, `new_drop_count` and `seconds_since_last_valid_sample`. The excerpt shows the class, from [`imu_validator.py`, lines 163–190]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L163-L190):

    ```python title="imu_validator.py: SampleCounts" linenums="163"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:163:190"
    ```

    `take_snapshot()` counts the new drops: the method adds up all dropped samples, subtracts `drop_count_at_last_snapshot`, the number that the snapshot before counted, and stores the new total for the next snapshot. The method copies `drops_by_reason` into a new dictionary, so a snapshot keeps its numbers when more samples arrive.

??? example "The four tests"

    The excerpt shows the first two tests, from [`test_imu_validator.py`, lines 340–358]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L340-L358):

    ```python title="test_imu_validator.py: the counts of a snapshot" linenums="340"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:340:358"
    ```

    - **The counts:** after two forwarded samples, two samples dropped as too old and one dropped for its frame, the snapshot has `forwarded_count` 2 and `drops_by_reason` with 2 and 1. The snapshot also carries the [silence](#seconds_since_last_valid_sample-the-silence) that the test passed to `take_snapshot()`.
    - **New drops:** `new_drop_count` is the number of samples dropped since the snapshot before. One dropped sample makes this number 1 in the next snapshot and 0 in the one after. Two more dropped samples, for two different reasons, make the number 2. [`health()`](#testhealth-the-status) uses this number for the report's level: a new drop can raise the next report to [`WARN`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/diagnostic_msgs/msg/DiagnosticStatus.msg#L6) when the IMU is otherwise healthy. The report after it is [`OK`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/diagnostic_msgs/msg/DiagnosticStatus.msg#L5) again if no further sample was dropped and the IMU is still healthy. A silence that is too long, or no valid sample yet, takes priority: the report is then [`ERROR`](https://github.com/ros/common_msgs/blob/092bc230c8153dd4cfee0c84c9796cc7e6b09fdb/diagnostic_msgs/msg/DiagnosticStatus.msg#L7), as the last row of the table in [`TestHealth`](#testhealth-the-status) shows.

    The other two tests cover two cases in which this count could go wrong. The excerpt shows them, from [`test_imu_validator.py`, lines 360–376]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L360-L376):

    ```python title="test_imu_validator.py: a snapshot in an error report, and a snapshot that stays" linenums="360"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:360:376"
    ```

    - **In an error report:** a sample is dropped before any sample was valid. The first snapshot gets the silence `None`, so `health()` reports an error, not the warning. The dropped sample counts as new in this snapshot all the same, and not again in the next one.
    - **A snapshot stays as it is:** after `take_snapshot()`, the test counts one more forwarded sample and one more dropped sample. The snapshot from before still has its numbers. A report is built from one snapshot, so its numbers all come from one moment.
