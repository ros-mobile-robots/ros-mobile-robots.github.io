# Tests: The IMU Sample Validator

What the unit tests of the IMU sample validator check, and how they do it. This page belongs to the [tests of the Localization](index.md).

[`test_imu_validator.py`]({{ diffbot_repo_url }}/diffbot_control/test/test_imu_validator.py) tests the [IMU sample validator](../../robot_localization.md#the-imu-sample-validator) one decision at a time. Its 27 unit tests call the validator's Python class and functions directly. No ROS master runs, no node starts and no message travels over a topic. The tests prove what the validator decides for a given IMU sample at a given ROS time $t$, and what its diagnostics report. They don't prove that the node forwards the samples on its topic or that the EKF handles them. For that, the [stepped-clock test](stepped-clock.md) runs the node together with the EKF.

The file holds the 27 tests in four test classes, one for each part of [`imu_validator.py`]({{ diffbot_repo_url }}/diffbot_control/scripts/imu_validator.py) that works without ROS communication:

| Test class | Tests | What it calls | What it checks |
|:--|:--|:--|:--|
| [`TestImuValidator`](#testimuvalidator-which-samples-are-valid) | 11 | [`ImuValidator.drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample) | Which samples the validator forwards, and the reason for each sample that it drops |
| [`TestSecondsSinceLastValidSample`](#testsecondssincelastvalidsample-the-silence) | 7 | [`ImuValidator.seconds_since_last_valid_sample()`](#seconds_since_last_valid_sample-the-silence) | The time since the last valid sample, which the diagnostics report, also when the clock goes back |
| [`TestHealth`](#testhealth-the-status) | 5 | `health()` | The level of the diagnostics status: OK, warning or error |
| [`TestSampleCounts`](#testsamplecounts-the-numbers-of-a-report) | 4 | `SampleCounts` | The counts of forwarded and dropped samples in a diagnostics report |

A fifth class, [`ValidatorTestCase`](#validatortestcase-a-new-validator-and-two-assertions), holds no tests. It gives the first two test classes a new validator for each test and two assertions about the validator's decision on a sample.

[What the tests call](#what-the-tests-call) explains the validator's methods and its reasons to drop a sample before the tests use them. [The constants and the helpers](#the-constants-and-the-helpers) explains what the tests share. [What the tests don't show](#what-the-tests-dont-show) lists what a pass doesn't prove.

## How the tests run

catkin runs the file with nose, a test runner for Python, as part of the package's tests. The excerpt shows how `diffbot_control` registers its three EKF tests, from [`CMakeLists.txt`, lines 226–234]({{ ekf_test_src_url }}/diffbot_control/CMakeLists.txt#L226-L234). [`catkin_add_nosetests()`](https://github.com/ros/catkin/blob/noetic-devel/doc/howto/format2/python_nose_configuration.rst) registers this file. [`add_rostest()`](https://github.com/ros/catkin/blob/noetic-devel/doc/howto/format2/rostest_configuration.rst) registers the two scenarios of the [stepped-clock test](stepped-clock.md), one `.test` file each:

```cmake title="diffbot_control/CMakeLists.txt: the package's tests" linenums="226"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/CMakeLists.txt:226:234"
```

CI runs all three with the workspace's other tests, as [Testing and CI](../../../../development/ci.md#ekf-tests) describes. To run only this file, open a shell in which the workspace is sourced and start the file with Python's `unittest` from its folder:

```console
roscd diffbot_control/test
python3 -m unittest -v test_imu_validator
```

The collapsed box shows the whole output of a run at the commit [`545fc53`](https://github.com/ros-mobile-robots/diffbot/commit/545fc533ea8fcb7ed6965dfab9cc48fa6c0d1130) of [diffbot#220](https://github.com/ros-mobile-robots/diffbot/pull/220), in the [dev container](../../../../development/dev-container.md)'s image with ROS Noetic and Python 3.8.10. The run started `python3 -m unittest -v test_imu_validator` in the folder `diffbot_control/test`, the folder that `roscd` changes to. With the option `-v`, unittest prints one line for each test: the test's name, its class in parentheses, and `ok` for a test that passed. unittest runs the classes, and the tests in each class, in the alphabetical order of their names, so the output's order differs from the file's. `ValidatorTestCase` has no tests, so the output has no line for this class. The last lines give the number of tests and the time that they took, and `OK` when every test passed:

??? example "The output of `python3 -m unittest -v test_imu_validator`"

    ```text
    test_health_is_a_warning_with_new_drops (test_imu_validator.TestHealth) ... ok
    test_health_is_an_error_before_the_first_valid_sample (test_imu_validator.TestHealth) ... ok
    test_health_is_an_error_for_silence_also_with_new_drops (test_imu_validator.TestHealth) ... ok
    test_health_is_ok_at_the_silence_limit_and_an_error_beyond_it (test_imu_validator.TestHealth) ... ok
    test_health_is_ok_without_new_drops (test_imu_validator.TestHealth) ... ok
    test_validator_drops_a_sample_at_ros_time_zero (test_imu_validator.TestImuValidator) ... ok
    test_validator_drops_a_sample_not_later_than_the_last_valid_one (test_imu_validator.TestImuValidator) ... ok
    test_validator_drops_a_sample_with_a_nan_or_infinite_value (test_imu_validator.TestImuValidator) ... ok
    test_validator_drops_a_sample_with_a_wrong_or_empty_frame (test_imu_validator.TestImuValidator) ... ok
    test_validator_drops_a_sample_with_an_invalid_angular_velocity_covariance (test_imu_validator.TestImuValidator) ... ok
    test_validator_forwards_a_new_sample_after_dropping_a_future_one (test_imu_validator.TestImuValidator) ... ok
    test_validator_forwards_a_sample_max_age_old_and_drops_an_older_one (test_imu_validator.TestImuValidator) ... ok
    test_validator_forwards_a_sample_max_future_ahead_and_drops_a_later_one (test_imu_validator.TestImuValidator) ... ok
    test_validator_forwards_new_samples_after_the_clock_goes_back (test_imu_validator.TestImuValidator) ... ok
    test_validator_forwards_valid_samples (test_imu_validator.TestImuValidator) ... ok
    test_validator_still_drops_old_and_future_samples_after_the_clock_goes_back (test_imu_validator.TestImuValidator) ... ok
    test_snapshot_counts_a_dropped_sample_as_new_once_also_in_an_error_report (test_imu_validator.TestSampleCounts) ... ok
    test_snapshot_counts_a_dropped_sample_as_new_only_once (test_imu_validator.TestSampleCounts) ... ok
    test_snapshot_has_the_forwarded_count_and_the_drops_by_reason (test_imu_validator.TestSampleCounts) ... ok
    test_snapshot_keeps_its_numbers_when_more_samples_arrive (test_imu_validator.TestSampleCounts) ... ok
    test_silence_is_none_after_the_clock_goes_back_a_little (test_imu_validator.TestSecondsSinceLastValidSample) ... ok
    test_silence_is_none_after_the_clock_goes_back_without_samples (test_imu_validator.TestSecondsSinceLastValidSample) ... ok
    test_silence_is_none_before_the_first_valid_sample (test_imu_validator.TestSecondsSinceLastValidSample) ... ok
    test_silence_is_none_when_a_dropped_sample_shows_the_clock_went_back (test_imu_validator.TestSecondsSinceLastValidSample) ... ok
    test_silence_is_the_time_since_the_last_valid_sample (test_imu_validator.TestSecondsSinceLastValidSample) ... ok
    test_silence_keeps_growing_while_samples_are_dropped (test_imu_validator.TestSecondsSinceLastValidSample) ... ok
    test_silence_restarts_with_the_first_valid_sample_after_the_clock_goes_back (test_imu_validator.TestSecondsSinceLastValidSample) ... ok

    ----------------------------------------------------------------------
    Ran 27 tests in 0.002s

    OK
    ```

## What the tests call

Most tests call the class [`ImuValidator`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L203), which holds the validator's checks without the ROS node around them. This section explains the class's methods that the tests call or that run inside these calls, and the reasons to drop a sample that the class returns. [The five checks](../../robot_localization.md#the-five-checks) on the Localization page explain the checks themselves. `health()` and `SampleCounts` have their own sections, next to their tests.

### Where the ROS time comes from

`ImuValidator` never reads the ROS clock itself. Its methods `drop_reason_or_record()` and `seconds_since_last_valid_sample()` get the ROS time $t$ as their argument `now`. The caller decides which time that is:

- **On the robot and in the simulation:** the node's class [`ImuValidatorNode`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L331) calls the two methods. For each sample, rospy calls the node's callback `on_sample()`, which reads the ROS time with `rospy.Time.now()` and passes it to `drop_reason_or_record()` as `now`. For each diagnostics report, the node's method [`take_snapshot()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L387) reads the ROS time in the same way and passes it to `seconds_since_last_valid_sample()`.
- **In the tests:** each test passes a ROS time that it chooses, most often `NOW`, 100 s. So a test can put any ROS time next to any stamp $t_s$, and needs no clock and no waiting.

The excerpt shows `on_sample()`, from [`imu_validator.py`, lines 355–370]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L355-L370). Line 358 reads the ROS time, and line 359 passes the time with the sample to `drop_reason_or_record()`:

```python title="imu_validator.py: the node reads the ROS time and passes it on" linenums="355"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:355:370"
```

Line 359 stores the method's result in the variable `drop_reason`. The result is either `None` or a reason to drop the sample:

- **`None`:** Python's value for "nothing", here "no reason to drop the sample". The node counts the sample as forwarded and publishes the sample on the topic `imu/data_validated`, in lines 360–364.
- **A reason:** a member of [`DropReason`](#dropreason-the-reasons-to-drop-a-sample). The node counts the sample by this reason in line 366. Lines 367–370 log a warning with the reason's text, at most once every `WARNING_PERIOD`, 5 s.

The lock of line 357 keeps the diagnostics timer, which runs in another thread, from reading the validator in the middle of a sample.

What `rospy.Time.now()` returns depends on the ROS parameter `/use_sim_time`, as the ROS wiki's page [Clock](http://wiki.ros.org/Clock) explains:

- **On the robot:** none of the launch files that start the robot sets `/use_sim_time`. [Which launch file starts what](../../robot_localization.md#which-launch-file-starts-what) on the Localization page shows these files. Without the parameter, `rospy.Time.now()` returns the system clock of the robot's single-board computer (SBC), in seconds since 1 January 1970. This clock doesn't read 0.
- **In the simulation:** gazebo_ros's [`empty_world.launch`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros/launch/empty_world.launch#L27), which [`diffbot_view.launch`]({{ ekf_test_src_url }}/diffbot_gazebo/launch/diffbot_view.launch#L30) includes, sets `/use_sim_time` to true. Gazebo publishes the simulation's time on the topic `/clock`, and `rospy.Time.now()` returns the time of the last message there. Until the node has received the first message on `/clock`, `rospy.Time.now()` returns 0. [Before the checks: no clock](#before-the-checks-no-clock) shows what the validator does then.

The [stepped-clock test](stepped-clock.md) sets `/use_sim_time` too, and publishes the messages on `/clock` itself.

### `drop_reason_or_record()`: the decision about a sample

`drop_reason_or_record(message, now)` decides about one sample. The node calls the method once for each sample, and the tests call the method through [two assertions](#validatortestcase-a-new-validator-and-two-assertions). Its two arguments are:

- **`message`:** the sample, a `sensor_msgs/Imu` message.
- **`now`:** the ROS time $t$ at which the sample arrives, a `rospy.Time`.

The name lists the method's two outcomes in their order: the caller gets the drop reason, the first reason to drop the sample, or, when the sample has no drop reason, the method records the sample and returns `None`. The return type in the method's first line, `Optional[DropReason]`, says the same: the result is a member of [`DropReason`](#dropreason-the-reasons-to-drop-a-sample) or `None`. So `None` means "no reason to drop": the sample is valid, and the node forwards the sample. The excerpt shows the method, from [`imu_validator.py`, lines 244–256]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L244-L256):

```python title="imu_validator.py: drop_reason_or_record()" linenums="244"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:244:256"
```

The method runs two steps, each in a method of its own:

1. **The decision:** `drop_reason()` returns the first reason to drop the sample, or `None` when the sample passes every check.
2. **A valid sample:** only when the reason is `None`, `record_valid_sample()` records the sample as the last valid one.

Because the node and the tests' assertions both call `drop_reason_or_record()`, the tests run the same two steps as the node. No caller can record a sample that the checks dropped, or forget to record a valid one. The excerpt shows the two methods, from [`imu_validator.py`, lines 258–277]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L258-L277):

```python title="imu_validator.py: drop_reason() and record_valid_sample()" linenums="258"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:258:277"
```

`drop_reason()` works in three steps:

1. **No clock:** at the ROS time 0, the method returns `DropReason.NO_CLOCK` at once, before any check. Line 265 tests the time with `now.is_zero()`. The comment in line 264 says why this step comes before `observe_time()`: the time 0 means that there is no clock yet, not that the clock went back.
2. **The time of the call:** `observe_time()` records `now`, as [the next section](#observe_time-the-ros-time-of-each-call) explains.
3. **The checks:** the loop calls the checks in the tuple `sample_checks`, one after the other. The first check that returns a reason ends the loop, and the method returns that reason. The later checks don't run. When every check has returned `None`, the method returns `None`.

`record_valid_sample()` records the sample's stamp $t_s$ as `last_valid_stamp`, the stamp $t_\text{last}$ that check 5 compares the next stamps with. The method records `now` as `last_valid_at`, from which `seconds_since_last_valid_sample()` counts.

For a sample that a check drops, the validator records only the ROS time of the call, in step 2 of `drop_reason()`. So the stamp of a dropped sample never becomes $t_\text{last}$. Step 2 can still make the validator forget the last valid sample, when the call's `now` shows that the clock went back, as the next section explains. A sample that arrives at the ROS time 0 leaves no trace at all, because step 1 returns before step 2.

The constructor `__init__()` fills `sample_checks` in the order in which the checks run. The excerpt shows the tuple, from [`imu_validator.py`, lines 221–229]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L221-L229):

```python title="imu_validator.py: the checks in their order" linenums="221"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:221:229"
```

The six functions are the five checks of the Localization page. Check 4 has two functions, one for each time limit: `check_not_from_the_future()` and `check_not_too_old()`. A sample that would fail two checks gets the reason of the earlier one, as sample A in [Check 4: the two time limits](#check-4-the-two-time-limits) shows.

### `observe_time()`: the ROS time of each call

`observe_time(now)` records the ROS time of each call and notices a clock that goes back. `drop_reason()` and `seconds_since_last_valid_sample()` call the method with their own `now`. The tests never call `observe_time()` directly. They see its effect through these two methods, in [A clock that goes back](#a-clock-that-goes-back) and in the tests of [the silence](#testsecondssincelastvalidsample-the-silence). The excerpt shows the method, from [`imu_validator.py`, lines 231–242]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L231-L242):

```python title="imu_validator.py: observe_time()" linenums="231"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:231:242"
```

- **A clock that goes back:** `last_observed_time` holds the ROS time of the call before. When `now` is earlier, the clock went back, and the method sets `last_valid_stamp` and `last_valid_at` to `None`. The validator forgets the last valid sample: check 5 has no stamp $t_\text{last}$ to compare with, and `seconds_since_last_valid_sample()` returns `None` until a sample is valid again.
- **Every call:** the method records `now` as `last_observed_time`.

### `seconds_since_last_valid_sample()`: the silence

`seconds_since_last_valid_sample(now)` returns how long no sample has been valid at the ROS time `now`. The [validator's diagnostics](../../robot_localization.md#the-validators-diagnostics) report this silence. The excerpt shows the method, from [`imu_validator.py`, lines 315–328]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L315-L328):

```python title="imu_validator.py: seconds_since_last_valid_sample()" linenums="315"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:315:328"
```

The method first calls `observe_time()`, so the method notices a clock that went back also while no sample arrives. Then the method returns one of two things:

- **`None`:** no sample was valid since the clock last went back, or none at all.
- **The silence in s, a `float`:** the ROS time $t$ minus $t_\text{valid}$, the ROS time `last_valid_at` of the call that recorded the last valid sample: $t - t_\text{valid}$. The silence counts from the sample's arrival, not from its stamp.

### `DropReason`: the reasons to drop a sample

`DropReason` is an [enumeration](https://docs.python.org/3/library/enum.html), a fixed set of named members, one for each reason to drop a sample. The excerpt shows the class, from [`imu_validator.py`, lines 105–118]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L105-L118):

```python title="imu_validator.py: DropReason" linenums="105"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:105:118"
```

Each member's value is a text. The node writes the text into its warning in the log, as in `on_sample()` above. The diagnostics count the dropped samples by reason, each under a key that starts with `Dropped,` and ends with the text, as `Dropped, too old` in the example of the [validator's diagnostics](../../robot_localization.md#the-validators-diagnostics). The table lists the seven members with the check that returns each:

| Member | Returned by | Text in the log and on `/diagnostics` |
|:--|:--|:--|
| `NO_CLOCK` | [`drop_reason()`](#drop_reason_or_record-the-decision-about-a-sample) itself, before the checks, at the ROS time 0 | `no ROS clock` |
| `WRONG_FRAME` | [Check 1](../../robot_localization.md#checks-1-to-3-the-samples-form), `check_frame()` | `wrong frame` |
| `NOT_FINITE` | [Check 2](../../robot_localization.md#checks-1-to-3-the-samples-form), `check_finite_values()` | `not finite` |
| `INVALID_ANGULAR_VELOCITY_COVARIANCE` | [Check 3](../../robot_localization.md#checks-1-to-3-the-samples-form), `check_angular_velocity_covariance()` | `invalid angular velocity covariance` |
| `FROM_THE_FUTURE` | [Check 4](../../robot_localization.md#check-4-the-stamp-against-the-ros-time), `check_not_from_the_future()` | `from the future` |
| `TOO_OLD` | [Check 4](../../robot_localization.md#check-4-the-stamp-against-the-ros-time), `check_not_too_old()` | `too old` |
| `NOT_NEWER` | [Check 5](../../robot_localization.md#check-5-the-stamp-against-the-last-forwarded-sample), `check_newer_than_the_last()` | `not newer than the last one` |

The tests import the module `imu_validator` and not its names. So a test writes a member with the module's name in front, as `imu_validator.DropReason.TOO_OLD`.

## The constants and the helpers

The tests share a few constants, two helper functions that build samples and durations, and a base class with two assertions.

??? example "The constants of `test_imu_validator.py`, lines 23–48"

    The excerpt shows all constants of the file, from [`test_imu_validator.py`, lines 23–48]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L23-L48). `NOW` is the ROS time $t$ of most checks, 100 s. `MAX_AGE` and `MAX_FUTURE` are the two time limits that [`ekf.launch`]({{ diffbot_repo_url }}/diffbot_control/launch/ekf.launch) sets on the robot and in the simulation. `JUST_TOO_OLD` and `JUST_TOO_FAR_AHEAD` lie `ONE_MICROSECOND`, 1 µs, beyond these limits.

    ```python title="test_imu_validator.py: the constants" linenums="23"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:23:48"
    ```

Every test that gives the validator a sample builds the sample with the helper `make_imu_sample()`. The excerpt shows `make_imu_sample()` and the helper `seconds()`, from [`test_imu_validator.py`, lines 51–72]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L51-L72):

```python title="test_imu_validator.py: make_imu_sample() and seconds()" linenums="51"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:51:72"
```

- **`make_imu_sample()`:** returns a `sensor_msgs/Imu` message like the firmware's. With the defaults of its three arguments, the sample is valid at the ROS time `NOW` for a new validator, one that has seen no sample yet. If the last valid sample has the same stamp, check 5 drops the new sample. With the defaults, the sample's stamp $t_s$ is `NOW`, its frame is `imu_link` and its yaw rate $\omega_z$ is `YAW_RATE`, 0.5 rad/s. The accelerometer's reading on the upward axis, $f_z$, is `GRAVITY`, 9.81 m/s², as for an IMU at rest. `orientation_covariance[0]` is −1, which marks the orientation as unavailable, as in the firmware's message.
- **The variances:** each variance on the diagonal of the angular velocity's covariance $\boldsymbol\Sigma_{\boldsymbol\omega}$, such as $\sigma^2_{\omega_z}$, is `ANGULAR_VELOCITY_VARIANCE`, 4 · 10⁻⁴ (rad/s)². Each variance on the diagonal of `linear_acceleration_covariance` is `LINEAR_ACCELERATION_VARIANCE`, 10⁻² (m/s²)². Both have the size of the firmware's variances.
- **Valid and invalid samples:** the tests override the defaults of `stamp`, `frame_id` or `yaw_rate`, or change a field of the returned message, to build the samples that they need. A later stamp gives the next valid sample, for example. An invalid sample differs from the valid one in one property, so that the sample aims at one check. A sample can fail a later check too, as sample A of the time limits does.
- **`seconds()`:** turns a number of seconds into a `rospy.Duration`, which a test adds to `NOW` or subtracts from it.

### `ValidatorTestCase`: a new validator and two assertions

`TestImuValidator` and `TestSecondsSinceLastValidSample` derive from the class `ValidatorTestCase`, which derives from `unittest.TestCase`. A class that derives from another class inherits the other class's methods. So the tests of both classes can use the three methods of `ValidatorTestCase`: `setUp()` and the two assertions `assert_accepted()` and `assert_rejected()`. `ValidatorTestCase` itself has no tests. The excerpt shows the class, from [`test_imu_validator.py`, lines 75–92]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L75-L92):

```python title="test_imu_validator.py: ValidatorTestCase" linenums="75"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:75:92"
```

- **`setUp()`:** unittest calls `setUp()` before each test method. So each test of a class that derives from `ValidatorTestCase` starts with a new `ImuValidator` in `self.validator`, one that has seen no sample yet. No test depends on the samples of another test. `FRAME`, `MAX_AGE` and `MAX_FUTURE` are the values that [`ekf.launch`]({{ diffbot_repo_url }}/diffbot_control/launch/ekf.launch) gives the validator's node.
- **`assert_accepted(message, now)`:** calls [`drop_reason_or_record(message, now)`](#drop_reason_or_record-the-decision-about-a-sample) once, the same method that the node calls once for each sample, and checks the result with `assertIsNone()`. A result of `None` means that the sample has no drop reason: the validator accepts the sample and records the sample as the last valid one. For any other result, `assertIsNone()` stops the test, and unittest reports the test as failed, with the drop reason that came back instead of `None`.
- **`assert_rejected(reason, message, now)`:** calls `drop_reason_or_record(message, now)` once too, and compares the result with the expected `reason` by `assertEqual()`. `assertEqual()` compares its first two arguments and fails the test when they differ. The tests of this file give `assertEqual()` the expected value first and the result second, and `assert_rejected()` does the same. So the assertion holds when the validator rejects the sample with the expected reason. A rejected sample doesn't become the last valid one.
- **`msg`:** the optional last argument of both assertions. The type `Optional[str]` says that `msg` is a text or `None`, and `= None` makes `None` the default. When the assertion fails, unittest adds the text to its report of the failure. The tests of checks 1 to 3 run a loop over several cases and give each assertion a text that names the case, so that a failure says which case the validator decided wrongly.

`assertIsNone()` and `assertEqual()` are two of the [assert methods](https://docs.python.org/3/library/unittest.html#assert-methods) that every test class inherits from `unittest.TestCase`, which is why the two assertions call these methods on `self`. The tests call `assert_accepted()` and `assert_rejected()` on `self` in the same way.

Accepted and rejected are the validator's decisions. The tests' names say "forwards" and "drops", which is what the node does with an accepted and with a rejected sample. The tests themselves publish nothing, as [What the tests don't show](#what-the-tests-dont-show) says.

## `TestImuValidator`: which samples are valid

The class tests [`drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample), one reason to drop a sample at a time. The excerpt shows the class's first two lines, from [`test_imu_validator.py`, lines 95–96]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L95-L96). The name in parentheses, `ValidatorTestCase`, is the class that `TestImuValidator` derives from. So each test starts with a new validator and checks each decision of the validator with [`assert_accepted()` or `assert_rejected()`](#validatortestcase-a-new-validator-and-two-assertions):

```python title="test_imu_validator.py: the class derives from ValidatorTestCase" linenums="95"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:95:96"
```

The sections below follow the order of the checks, with the numbers and names of [the five checks](../../robot_localization.md#the-five-checks) on the Localization page.

### A valid sample

`test_validator_forwards_valid_samples` is the baseline of the class. The excerpt shows the test, from [`test_imu_validator.py`, lines 98–100]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L98-L100):

```python title="test_imu_validator.py: valid samples" linenums="98"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:98:100"
```

Line 99 holds two calls, one inside the other. Python runs the inner call first:

1. **The inner call:** `make_imu_sample()` returns a valid sample, stamped `NOW`.
2. **The outer call:** `self.assert_accepted(make_imu_sample(), NOW)` gets two arguments. The first is the sample from the inner call. The second is the ROS time $t$ of the sample's arrival, `NOW`, 100 s. The assertion calls `drop_reason_or_record()` with both and checks that the method returns `None`, no drop reason. Otherwise the test stops there, and unittest reports the test as failed, with the drop reason that came back instead of `None`.

So line 99 checks that the validator accepts the sample. In the node, `on_sample()` forwards an accepted sample.

Line 100 checks a second sample in the same way. The sample is stamped `MAX_FUTURE`, 0.02 s, later than the first, and arrives at a ROS time that is 0.02 s later too. This sample is the next one of the IMU at 50 Hz. The validator accepts the sample, because its stamp is later than the first sample's.

### Before the checks: no clock

`test_validator_drops_a_sample_at_ros_time_zero` gives the validator a valid sample at the ROS time 0, `rospy.Time(0)`. The excerpt shows the test, from [`test_imu_validator.py`, lines 102–104]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L102-L104):

```python title="test_imu_validator.py: no clock" linenums="102"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:102:104"
```

`assert_rejected()` gets the expected reason first, here `imu_validator.DropReason.NO_CLOCK`, then the sample and the ROS time. So the test checks that the validator rejects even a valid sample at the ROS time 0, with the reason `NO_CLOCK`.

The ROS time 0 means that the node has no clock yet. With simulation time, `rospy.Time.now()` returns 0 until the node has received the first message on the topic `/clock`, as [Where the ROS time comes from](#where-the-ros-time-comes-from) explains. A stamp can't be judged against this time. At $t = 0$, check 4 would allow only the stamps between −0.1 s and 0.02 s:

$$
t - \texttt{max\_age} \le t_s \le t + \texttt{max\_future}
\quad\Rightarrow\quad
-0.1\ \text{s} \le t_s \le 0.02\ \text{s}
$$

- **A sample from Gazebo:** a valid sample stamped, for example, 0.5 s would be dropped as from the future. The log and the diagnostics would blame the sample's stamp, although the node's clock is missing.
- **A sample without a stamp:** a message whose stamp was never set has the stamp 0 s, and would pass check 4.

So the validator drops every sample at the ROS time 0 before the checks, with its own reason `NO_CLOCK`, which `drop_reason()` returns. On the robot, the ROS time is the SBC's system clock, which doesn't read 0. So the robot's validator doesn't drop samples for this reason.

### Check 1: the frame

Check 1 compares the sample's `header.frame_id` with the frame that the validator expects, `FRAME`, `imu_link`. The excerpt shows the test, from [`test_imu_validator.py`, lines 106–110]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L106-L110):

```python title="test_imu_validator.py: a wrong frame" linenums="106"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:106:110"
```

The loop runs twice, with the frame `base_link` and with an empty frame. In each pass, `assert_rejected()` expects `DropReason.WRONG_FRAME`. The last argument is the assertion's `msg`, the text that unittest adds to its report when the assertion fails. The text names the frame of the loop's current pass, so a failure says which of the two frames the validator didn't drop.

### Check 2: the values

Check 2 needs finite values in the angular velocity $\boldsymbol\omega$, the field `angular_velocity`, and in the accelerometer's reading $\mathbf f$, the field `linear_acceleration`. The excerpt shows the test, from [`test_imu_validator.py`, lines 112–121]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L112-L121):

```python title="test_imu_validator.py: NaN and infinite values" linenums="112"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:112:121"
```

The loop runs three times, for NaN, +∞ and −∞. Each pass builds two samples. The first has the value as its yaw rate $\omega_z$, `angular_velocity.z`. The second has the value as $f_x$, `linear_acceleration.x`. The validator has to drop all six samples with `DropReason.NOT_FINITE`.

### Check 3: the variances

Check 3 looks at the covariance $\boldsymbol\Sigma_{\boldsymbol\omega}$ of the angular velocity $\boldsymbol\omega$, the field `angular_velocity_covariance`. Its nine values have to be finite, and its three variances on the diagonal, $\sigma^2_{\omega_x}$, $\sigma^2_{\omega_y}$ and $\sigma^2_{\omega_z}$, have to be positive. [The variances](../../robot_localization.md#the-variances) on the Localization page shows how the message stores the 3 × 3 matrix as nine numbers, row by row. The excerpt shows the test, from [`test_imu_validator.py`, lines 123–134]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L123-L134):

```python title="test_imu_validator.py: three invalid covariances" linenums="123"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:123:134"
```

The test builds three samples, each with one defect in $\boldsymbol\Sigma_{\boldsymbol\omega}$, and expects `DropReason.INVALID_ANGULAR_VELOCITY_COVARIANCE` for each. The text in each pair of `cases` is the `msg` of a failure:

- **`unavailable`:** the first element is −1 and the others are 0. This is how a `sensor_msgs/Imu` message says that it carries no angular velocity. Check 3 reads the three variances $\sigma^2_{\omega_x}$, $\sigma^2_{\omega_y}$ and $\sigma^2_{\omega_z}$ as −1, 0 and 0 (rad/s)², so none of them is positive.
- **`zero_variance`:** the variance of the yaw rate $\sigma^2_{\omega_z}$, the element with the index `YAW_RATE_VARIANCE_INDEX`, 8, is 0 (rad/s)².
- **`nan_off_the_diagonal`:** the covariance $\sigma_{\omega_x \omega_y}$, the element with the index `OFF_DIAGONAL_INDEX`, 1, is NaN. This element is not a variance, so the test shows that the check looks at all nine values.

### Check 4: the two time limits

Check 4 compares the stamp $t_s$ with the ROS time $t$: $t - \texttt{max\_age} \le t_s \le t + \texttt{max\_future}$. Two tests probe each limit from both sides. The excerpt shows both tests, from [`test_imu_validator.py`, lines 136–144]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L136-L144):

```python title="test_imu_validator.py: at the limits and 1 µs beyond" linenums="136"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:136:144"
```

The figure below shows the samples of these two tests on a time axis. The sections on check 5 and on a clock that goes back have figures of the same kind. Read the three figures as follows:

- **The axis:** time runs to the right. One tick is 0.02 s, the IMU's sample period at 50 Hz. Each axis has its own scale.
- **A mark:** stands at the stamp $t_s$ of its sample. Its letter is the sample's name in the text. A solid green mark is a sample that the validator has to forward, a dashed red mark one that the validator has to drop.
- **The bands:** a stamp in a green band passes the check. A stamp in a red band is dropped for the reason that the band names.
- **Two slanted strokes:** the axis leaves out a stretch of time there.
- **The list below a figure:** says what the validator has to do with each sample, and why.

Two samples that lie only 1 µs or 1 ns apart would share one spot at the figure's scale, so the figures draw them apart.

--8<-- "includes/localization-test-validator-limits.html"

The first test sends B and then A, the second test D and then C, all at the ROS time `NOW`. The tests build the four stamps from the constants: B is `NOW - seconds(MAX_AGE)`, A is `NOW - seconds(JUST_TOO_OLD)`, D is `NOW + seconds(JUST_TOO_FAR_AHEAD)` and C is `NOW + seconds(MAX_FUTURE)`. The validator has to forward B and C, so each limit itself is still valid. The validator has to drop A with `DropReason.TOO_OLD` and D with `DropReason.FROM_THE_FUTURE`.

In the first test, sample A is also older than the valid sample B before it. The validator reports `DropReason.TOO_OLD` and not `DropReason.NOT_NEWER`, because the checks run [in their order](#drop_reason_or_record-the-decision-about-a-sample) and check 4 comes before check 5. The second test sends the dropped sample D first. For a dropped sample, the validator records only the ROS time of the call, so D doesn't change what the validator decides for C.

### Check 5: the order of the stamps

Check 5 compares the stamp $t_s$ with $t_\text{last}$, the stamp of the last valid sample: $t_s > t_\text{last}$. The excerpt shows the two tests of this check, from [`test_imu_validator.py`, lines 146–158]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L146-L158):

```python title="test_imu_validator.py: the order check" linenums="146"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:146:158"
```

The figure shows the samples of the two tests around $t_\text{last}$, 100 s:

--8<-- "includes/localization-test-validator-order.html"

`test_validator_drops_a_sample_not_later_than_the_last_valid_one` sends four samples to one validator:

1. A valid sample stamped `NOW`. The validator records `NOW` as $t_\text{last}$.
2. Sample F, stamped `NOW` again. The validator drops the sample with `DropReason.NOT_NEWER`, because a sample is valid only once.
3. Sample E, stamped 0.01 s before `NOW`. The validator drops the sample with the same reason.
4. Sample G, stamped 1 ns after `NOW`, the smallest step of a ROS stamp, at a ROS time 1 ms later. The validator forwards the sample.

`test_validator_forwards_a_new_sample_after_dropping_a_future_one` shows what $t_\text{last}$ is not. After a valid sample stamped `NOW`, the test sends sample H, stamped 5 s ahead. Check 4 drops H with `DropReason.FROM_THE_FUTURE`. Then the test sends sample I, stamped 0.02 s after `NOW`. The validator forwards I. If the validator kept the stamp of the dropped sample H as $t_\text{last}$, every sample of the next 5 s would count as not newer.

### A clock that goes back

The ROS time goes back after a call of the service `/gazebo/reset_simulation` in the simulation, or when the SBC's clock is corrected on the robot. The validator notices this in [`observe_time()`](#observe_time-the-ros-time-of-each-call), when a call's `now` is earlier than the `now` of the call before. The validator then forgets the last valid sample, so that the order check starts over. The excerpt shows the two tests, from [`test_imu_validator.py`, lines 160–175]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L160-L175):

```python title="test_imu_validator.py: the clock goes back" linenums="160"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:160:175"
```

The figure shows the two tests, each on its own axis at the new ROS time:

--8<-- "includes/localization-test-validator-clock-back.html"

- **`test_validator_forwards_new_samples_after_the_clock_goes_back`:** after a valid sample at 100 s, the ROS time is `restart`, 0.02 s, as after a reset of the simulation. This test is the first axis of the figure. Sample J is stamped `restart`, which is far older than $t_\text{last}$, 100 s. The validator forwards J all the same, because the order check started over. The order check works again from this sample on: the test sends the same stamp once more, 1 ms later, and expects `DropReason.NOT_NEWER`. This second sample is K.
- **`test_validator_still_drops_old_and_future_samples_after_the_clock_goes_back`:** starting over applies to the order check only. After a valid sample at 100 s, the ROS time is `restart`, 50 s. This test is the second axis of the figure. Sample M is stamped 100.02 s, which was valid before the clock went back and is 50.02 s ahead of the new ROS time. The validator drops M with `DropReason.FROM_THE_FUTURE`. Sample L is stamped 1 s before `restart`, and the validator drops L with `DropReason.TOO_OLD`.

## `TestSecondsSinceLastValidSample`: the silence

The [validator's diagnostics](../../robot_localization.md#the-validators-diagnostics) report an error when no sample was forwarded for too long. The time behind this error comes from [`seconds_since_last_valid_sample()`](#seconds_since_last_valid_sample-the-silence), which returns the silence $t - t_\text{valid}$ in s, or `None` when no sample was valid since the clock last went back.

The class derives from [`ValidatorTestCase`](#validatortestcase-a-new-validator-and-two-assertions) too. So each test starts with a new validator, and the tests give the validator their samples through `assert_accepted()` and `assert_rejected()`. The class adds a helper of its own, `assert_silence()`, for the comparison of the silence. The excerpt shows the class's first lines, the helper and the first three tests, from [`test_imu_validator.py`, lines 178–200]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L178-L200):

```python title="test_imu_validator.py: the silence grows" linenums="178"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:178:200"
```

- **`assert_silence()`:** calls `seconds_since_last_valid_sample(now)` and compares the result with `expected`. `assertAlmostEqual()` rounds the difference of the two numbers to seven decimal places, its default, and expects zero.
- **Before the first valid sample:** the method returns `None`.
- **After a valid sample:** 1.5 s after the sample was recorded, the method returns 1.5 s.
- **While samples are dropped:** a sample that the validator drops with `DropReason.TOO_OLD` doesn't end the silence. 3 s after the valid sample, the method returns 3 s.

The next three tests are about a clock that goes back while no valid sample arrives. The excerpt shows them, from [`test_imu_validator.py`, lines 202–219]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L202-L219):

```python title="test_imu_validator.py: the silence after the clock goes back" linenums="202"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:202:219"
```

- **Without samples:** after a valid sample at 100 s, the method is asked at 50 s and at 55 s and returns `None` both times. The method itself notices that the clock went back, because the method records the ROS time of each call with `observe_time()`.
- **A little back:** the method is asked 3.3 s after the valid sample and then 0.2 s after the valid sample. The clock went back by less than the silence, so $t - t_\text{valid}$ would still be positive. The method returns `None` here too, and 1 s after the valid sample as well. A sample from before the clock went back says nothing about the IMU in the new time.
- **Shown by a dropped sample:** the clock goes back between two calls, and the first call in the new time is a sample that the validator drops. The validator records the ROS time of dropped samples too, so the method returns `None` afterwards.

The last test of the class follows the silence through a whole restart. The excerpt shows the test, from [`test_imu_validator.py`, lines 221–230]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L221-L230):

```python title="test_imu_validator.py: the silence starts again" linenums="221"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:221:230"
```

After a valid sample at 100 s, the ROS time is `restart`, 50 s. The validator drops a sample from the future and a sample with the wrong frame. 1 s later, the method still returns `None`. 2 s after `restart`, a valid sample arrives. 0.5 s after that sample, the method returns 0.5 s.

## `TestHealth`: the status

`health()` turns two numbers into the level and the message of the diagnostics status: the silence $t - t_\text{valid}$ in s, or `None`, and the number of samples dropped since the report before. The function reads no clock and keeps no state, so each call is independent of the others. The excerpt shows the class's five tests, from [`test_imu_validator.py`, lines 236–257]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L236-L257):

```python title="test_imu_validator.py: the level of the status" linenums="236"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:236:257"
```

| Silence | New drops | Level that the test expects |
|:--|:--|:--|
| `None`: no valid sample yet | 0 | `ERROR` |
| `SILENCE_LIMIT`, 1 s | 0 | `OK` |
| `SILENCE_LIMIT + JUST_BEYOND`, 1.001 s | 0 | `ERROR` |
| `RECENT`, 0.1 s | `NEW_DROP_COUNT`, 3 | `WARN`, and the message contains the number 3 |
| `RECENT`, 0.1 s | 0 | `OK` |
| `None`, and 1.001 s | 3 | `ERROR` for both: silence comes before the warning |

`SILENCE_LIMIT` is the validator's own constant, which the tests read from `imu_validator`. So a test doesn't repeat the limit's value. The test checks that the limit itself is still OK and that 1 ms more is an error.

## `TestSampleCounts`: the numbers of a report

`SampleCounts` counts the forwarded samples and the dropped samples by their reason. `take_snapshot()` returns the numbers of one diagnostics report as a `DiagnosticSnapshot`. The excerpt shows the first two tests, from [`test_imu_validator.py`, lines 266–284]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L266-L284):

```python title="test_imu_validator.py: the counts of a snapshot" linenums="266"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:266:284"
```

- **The counts:** after two forwarded samples, two samples dropped as too old and one dropped for its frame, the snapshot has `forwarded_count` 2 and `drops_by_reason` with 2 and 1. The snapshot also carries the silence that the test passed to `take_snapshot()`.
- **New drops:** `new_drop_count` is the number of samples dropped since the snapshot before. One dropped sample makes this number 1 in the next snapshot and 0 in the one after. Two more dropped samples, for two different reasons, make the number 2. The warning of `health()` depends on this number, so a dropped sample causes one warning, not a warning in every report from then on.

The other two tests cover two cases in which this count could go wrong. The excerpt shows them, from [`test_imu_validator.py`, lines 286–302]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L286-L302):

```python title="test_imu_validator.py: a snapshot in an error report, and a snapshot that stays" linenums="286"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:286:302"
```

- **In an error report:** a sample is dropped before any sample was valid. The first snapshot gets the silence `None`, so `health()` reports an error, not the warning. The dropped sample counts as new in this snapshot all the same, and not again in the next one.
- **A snapshot stays as it is:** after `take_snapshot()`, the test counts one more forwarded sample and one more dropped sample. The snapshot from before still has its numbers. A report is built from one snapshot, so its numbers all come from one moment.

## What the tests don't show

- **The node:** the class `ImuValidatorNode` reads the parameters, subscribes to the topic `imu/data_raw`, publishes on the topic `imu/data_validated` and on the topic `/diagnostics`, and reads the ROS time in a lock around each sample. These tests don't call the class. The [stepped-clock test](stepped-clock.md) runs the node with the EKF and checks its counts on the topic `/diagnostics`.
- **The real clock and the real IMU:** every time in these tests is a number that the test chose. Whether `max_age` and `max_future` fit the delays of the robot's USB link is a measurement on the robot, as [Limits](../../robot_localization.md#limits) says.
- **The fields that the validator doesn't check:** the [overview](../../robot_localization.md#what-the-validator-doesnt-check) lists them. No test sends a sample with an implausible but finite value, because the validator forwards such a sample by design.
