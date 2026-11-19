# Tests: The IMU Sample Validator

What the unit tests of the IMU sample validator check, and how they do it. This page belongs to the [tests of the Localization](index.md), which list the five tests by the question that each answers.

[`test_imu_validator.py`]({{ diffbot_repo_url }}/diffbot_control/test/test_imu_validator.py) tests the [IMU sample validator](../../robot_localization.md#the-imu-sample-validator) one decision at a time. Its 30 unit tests call the validator's Python class and functions directly. No ROS master runs, no node starts and no message travels over a topic. The tests prove what the validator decides for a given IMU sample at a given ROS time $t$, and what its diagnostics report. They don't prove that the node forwards the samples on its topic or that the EKF handles them. For that, the [stepped-clock test](stepped-clock.md) runs the node together with the EKF.

## One sample through the validator

Two examples show the validator's decision before the code. Each example gives one sample to a new validator, one that has seen no sample yet, at the ROS time $t$ = 100 s. This is the ROS time of most tests, their constant [`NOW`](#the-constants-and-the-helpers). Both samples are in the frame `imu_link`, with finite values and positive variances, as the firmware sends them. They differ only in their stamp $t_s$:

- **Sample B, stamped $t_s$ = 99.900000 s:** its age $t - t_s$ is 0.100000 s, exactly `max_age`. The validator accepts B, and the node forwards the sample.
- **Sample A, stamped $t_s$ = 99.899999 s:** its age $t - t_s$ is 0.100001 s, 1 µs more than `max_age`. The validator rejects A as [`TOO_OLD`](#dropreason-the-reasons-to-drop-a-sample), and the node drops the sample.

[Check 4](../../robot_localization.md#check-4-the-stamp-against-the-ros-time) lets a sample through that is at most `max_age`, 0.1 s, old: $t - t_s \le \texttt{max\_age}$. B is exactly that old and passes. [Check 5](../../robot_localization.md#check-5-the-stamp-against-the-last-forwarded-sample) has no stamp of an earlier valid sample to compare B's stamp with, so B passes check 5 too, and the validator accepts B. A is 1 µs older than B and fails check 4. The same two samples appear in the test of [Check 4: the two time limits](#check-4-the-two-time-limits), which gives both to one validator.

The figure shows every step of the decision, in the order of its arrows. On a wide screen, the steps run from left to right in two rows. On a narrow screen, they run from top to bottom. Each step is a box with a condition. A sample that meets the condition follows the arrow to the next step. A sample that doesn't meet the condition takes the arrow "no" to the reason for which the validator rejects the sample, in a dashed red box. The second step drops no sample: after the ROS time went back, the validator sets its attributes `last_valid_stamp` and `last_valid_arrival_time` to `None`. These two attributes hold the stamp and the arrival time of the last valid sample. Then the decision goes on, as [a clock that goes back](#clear_last_valid_sample_times_if_clock_went_back-a-clock-that-goes-back) explains. The numbers 1 to 5 are those of [the five checks](../../robot_localization.md#the-five-checks) on the Localization page.

--8<-- "includes/localization-test-validator-decision.html"

The decision has two ends:

- **Accepted:** [`drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample) stores the sample's stamp $t_s$ in the validator's attribute `last_valid_stamp` and its arrival time $t$ in `last_valid_arrival_time`, and returns `None`. The node counts the sample as forwarded and publishes the sample for the EKF. The tests check this end with [`assert_accepted()`](#validatortestcase-a-new-validator-and-two-assertions).
- **Rejected:** `drop_reason_or_record()` returns the reason. The validator stores neither the sample's stamp nor its arrival time in `last_valid_stamp` and `last_valid_arrival_time`. At a ROS time other than 0, the clock check has already stored the call's ROS time in `last_checked_ros_time`. The node counts the sample under this reason, which the [diagnostics](#testsamplecounts-the-numbers-of-a-report) report, and passes a warning to `rospy.logwarn_throttle()` with the period `WARNING_PERIOD`, 5 s. The tests check this end with [`assert_rejected()`](#validatortestcase-a-new-validator-and-two-assertions).

## The test classes

The file holds the 30 tests in four test classes, one for each part of [`imu_validator.py`]({{ diffbot_repo_url }}/diffbot_control/scripts/imu_validator.py) that works without ROS communication:

| Test class | Tests | What it calls | What it checks |
|:--|:--|:--|:--|
| [`TestImuValidator`](#testimuvalidator-which-samples-are-valid) | 12 | [`ImuValidator.drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample) | Which samples the validator accepts, and the reason for each sample that it rejects |
| [`TestSecondsSinceLastValidSample`](#testsecondssincelastvalidsample-the-silence) | 9 | [`ImuValidator.seconds_since_last_valid_sample()`](#seconds_since_last_valid_sample-the-silence) | The time since the last valid sample, which the diagnostics report, also when the clock goes back |
| [`TestHealth`](#testhealth-the-status) | 5 | [`health()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L143) | The level of the diagnostics status: OK, warning or error |
| [`TestSampleCounts`](#testsamplecounts-the-numbers-of-a-report) | 4 | [`SampleCounts`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L163) | The counts of forwarded and dropped samples in a diagnostics report |

A fifth class, [`ValidatorTestCase`](#validatortestcase-a-new-validator-and-two-assertions), holds no tests. It gives the first two test classes a new validator for each test and two assertions about the validator's decision on a sample.

[What the tests call](#what-the-tests-call) explains the validator's methods and its reasons to drop a sample before the tests use them. [The constants and the helpers](#the-constants-and-the-helpers) explains what the tests share. [Run the tests](#run-the-tests) shows how to run the file and its output. [What the tests don't show](#what-the-tests-dont-show) lists what a pass doesn't prove.

## What the tests call

Most tests call the class [`ImuValidator`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L203), which holds the validator's checks without the ROS node around them. This section explains the class's methods that the tests call or that run inside these calls, and the reasons to drop a sample that the class returns. [The five checks](../../robot_localization.md#the-five-checks) on the Localization page explain the checks themselves. [`health()`](#testhealth-the-status) and [`SampleCounts`](#testsamplecounts-the-numbers-of-a-report) have their own sections, next to their tests.

### Where the ROS time comes from

`ImuValidator` never reads the ROS clock itself. Its methods [`drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample) and [`seconds_since_last_valid_sample()`](#seconds_since_last_valid_sample-the-silence) get the ROS time $t$ as their argument `now`. The caller decides which time that is:

- **On the robot and in the simulation:** the node's class [`ImuValidatorNode`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L359) calls the two methods. For each sample, rospy calls the node's callback `on_sample()`, which reads the ROS time with `rospy.Time.now()` and passes it to `drop_reason_or_record()` as `now`. For each diagnostics report, the node's method [`take_snapshot()`]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L415) reads the ROS time in the same way and passes it to `seconds_since_last_valid_sample()`.
- **In the tests:** each test passes a ROS time that it chooses, most often [`NOW`](#the-constants-and-the-helpers), 100 s. So a test can put any ROS time next to any stamp $t_s$, and needs no clock and no waiting.

The excerpt shows `on_sample()`, from [`imu_validator.py`, lines 383–398]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L383-L398). Line 386 reads the ROS time, and line 387 passes the time with the sample to `drop_reason_or_record()`:

```python title="imu_validator.py: the node reads the ROS time and passes it on" linenums="383"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:383:398"
```

Line 387 stores the method's result in the variable `drop_reason`. The result is either `None` or a reason to drop the sample:

- **`None`:** Python's value for "nothing", here "no reason to drop the sample". The node counts the sample as forwarded and publishes the sample on the topic `imu/data_validated`, in lines 388–392.
- **A reason:** a member of [`DropReason`](#dropreason-the-reasons-to-drop-a-sample). The node counts the sample by this reason in line 394. Lines 395–398 pass a warning with the reason's text to `rospy.logwarn_throttle()` with the period `WARNING_PERIOD`, 5 s, which leaves out repeated warnings within that period.

The lock of line 385 keeps the diagnostics timer, which runs in another thread, from reading the validator in the middle of a sample.

What `rospy.Time.now()` returns depends on the ROS parameter `/use_sim_time`, as the ROS wiki's page [Clock](http://wiki.ros.org/Clock) explains:

- **On the robot:** none of the launch files that start the robot sets `/use_sim_time`. [Which launch file starts what](../../robot_localization.md#which-launch-file-starts-what) on the Localization page shows these files. Without the parameter, `rospy.Time.now()` returns the system clock of the robot's single-board computer (SBC), in seconds since 1 January 1970. This clock doesn't read 0.
- **In the simulation:** gazebo_ros's [`empty_world.launch`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros/launch/empty_world.launch#L27), which [`diffbot_view.launch`]({{ ekf_test_src_url }}/diffbot_gazebo/launch/diffbot_view.launch#L30) includes, sets `/use_sim_time` to true. Gazebo publishes the simulation's time on the topic `/clock`, and `rospy.Time.now()` returns the time of the last message there. Until the node has received the first message on `/clock`, `rospy.Time.now()` returns 0. [Before the checks: no clock](#before-the-checks-no-clock) shows what the validator does then.

The [stepped-clock test](stepped-clock.md) sets `/use_sim_time` too, and publishes the messages on `/clock` itself.

### `drop_reason_or_record()`: the decision about a sample

`drop_reason_or_record(message, now)` decides about one sample. The node calls the method once for each sample, and the tests call the method through [two assertions](#validatortestcase-a-new-validator-and-two-assertions). Its two arguments are:

- **`message`:** the sample, a `sensor_msgs/Imu` message. The sample carries its own time, the stamp $t_s$ in `header.stamp`, which the sender writes into the message. On the robot, the Teensy's firmware stamps each sample with rosserial's ROS time when it starts to read the BNO055. That time estimates when the BNO055 measured, because the sensor doesn't report that time, as the [firmware's IMU publisher]({{ ekf_test_src_url }}/diffbot_firmware/lib/imu/imu_publisher.h#L36-L38) explains. In the simulation, Gazebo's IMU plugin stamps each sample with the simulation time of the measurement. The header also carries the frame of the values, `header.frame_id`. [The messages and their fields](../../robot_localization.md#the-messages-and-their-fields) on the Localization page lists the sample's fields.
- **`now`:** the ROS time $t$ at which the sample arrives at the validator, a `rospy.Time`. The sample's way from the sender to the node takes time, so $t$ is usually a little later than $t_s$. On the robot, [rosserial synchronizes](https://github.com/ros-drivers/rosserial/blob/0.9.1/rosserial_client/src/ros_lib/ros/node_handle.h#L354-L383) the Teensy's clock only approximately with the SBC's clock. The remaining offset can put $t_s$ a little after $t$. Check 4 allows a stamp up to `max_future`, 0.02 s, ahead.

The validator checks both the frame and the stamp: [check 1](../../robot_localization.md#checks-1-to-3-the-samples-form) the frame, [check 4](../../robot_localization.md#check-4-the-stamp-against-the-ros-time) the stamp $t_s$ against the ROS time $t$, and [check 5](../../robot_localization.md#check-5-the-stamp-against-the-last-forwarded-sample) the stamp $t_s$ against the stamp of the last valid sample.

The name lists the method's two outcomes in their order: the caller gets the drop reason, the first reason to drop the sample, or, when the sample has no drop reason, the method records the sample's two times and returns `None`. To record means here that the validator stores the sample's stamp in its attribute `last_valid_stamp` and the ROS time of the sample's arrival in `last_valid_arrival_time`, for its next decisions. The validator doesn't keep the message or write it to a file. The return type in the method's first line, `Optional[DropReason]`, says the same: the result is a member of [`DropReason`](#dropreason-the-reasons-to-drop-a-sample) or `None`. So `None` means "no reason to drop": the sample is valid, and the node forwards the sample. The excerpt shows the method, from [`imu_validator.py`, lines 258–275]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L258-L275):

```python title="imu_validator.py: drop_reason_or_record()" linenums="258"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:258:275"
```

The method runs two steps, each in a method of its own: first [`drop_reason()`](#step-1-drop_reason-the-decision), then, only when the reason is `None`, [`record_valid_sample_times()`](#step-2-record_valid_sample_times-a-valid-sample).

#### Step 1: `drop_reason()`, the decision

`drop_reason()` returns the first reason to drop the sample, or `None` when the sample passes every check. The excerpt shows the method, from [`imu_validator.py`, lines 277–295]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L277-L295):

```python title="imu_validator.py: drop_reason()" linenums="277"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:277:295"
```

`drop_reason()` works in three steps:

1. **No clock:** if a sample arrives at the ROS time 0 s, lines 288 and 289 return [`DropReason.NO_CLOCK`](#dropreason-the-reasons-to-drop-a-sample) at once. The validator reads 0 s as "no clock yet", as the comment in lines 286 and 287 says, so the method skips step 2, the clock check. The clock check compares the current ROS time with `last_checked_ros_time`, the ROS time of the last clock check. At startup, before the first message on `/clock`, a new validator has no earlier check. Now suppose that the validator last checked the clock at 100 s and a clock publisher sends 0 s. A sample that arrives at 0 s leaves the validator's attributes as they are: 100 s in `last_checked_ros_time`, and the last valid sample's stamp and arrival time in `last_valid_stamp` and `last_valid_arrival_time`. If the next check reads 0.02 s, the check sets `last_valid_stamp` and `last_valid_arrival_time` to `None`, because 0.02 s is earlier than 100 s. That next check can come from a sample or from a diagnostics report. [A test in Before the checks: no clock](#before-the-checks-no-clock) and [two tests of the silence](#after-a-sample-at-the-ros-time-0) check that a sample at 0 s changes none of the three attributes.
2. **A clock that went back:** [`clear_last_valid_sample_times_if_clock_went_back(now)`](#clear_last_valid_sample_times_if_clock_went_back-a-clock-that-goes-back) clears `last_valid_stamp` and `last_valid_arrival_time` if `now` is earlier than `last_checked_ros_time`. The method then stores `now` in `last_checked_ros_time` for the next clock check.
3. **The checks:** the loop calls the checks in the tuple `sample_checks`, one after the other. The first check that returns a reason ends the loop, and the method returns that reason. The later checks don't run. When every check has returned `None`, the method returns `None`.

The constructor `__init__()` fills `sample_checks` in the order in which the checks run. The excerpt shows the tuple, from [`imu_validator.py`, lines 222–230]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L222-L230):

```python title="imu_validator.py: the checks in their order" linenums="222"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:222:230"
```

The six functions are [the five checks](../../robot_localization.md#the-five-checks) of the Localization page. Check 4 has two functions, one for each time limit: `check_not_too_far_ahead()` and `check_not_too_old()`. A sample that would fail two checks gets the reason of the earlier one, as sample A in [Check 4: the two time limits](#check-4-the-two-time-limits) shows.

#### Step 2: `record_valid_sample_times()`, a valid sample

Only when [`drop_reason()`](#step-1-drop_reason-the-decision) has returned `None`, `record_valid_sample_times()` stores two times of the sample in attributes of the validator: the sample's stamp in `last_valid_stamp` and its arrival time in `last_valid_arrival_time`. The excerpt shows the method, from [`imu_validator.py`, lines 297–303]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L297-L303):

```python title="imu_validator.py: record_valid_sample_times()" linenums="297"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:297:303"
```

Lines 302 and 303 store the two times: the sample's stamp $t_s$ in `last_valid_stamp`, the stamp $t_\text{last}$ that [check 5](../../robot_localization.md#check-5-the-stamp-against-the-last-forwarded-sample) compares the next stamps with, and the ROS time of its arrival, `now`, in `last_valid_arrival_time`, from which [`seconds_since_last_valid_sample()`](#seconds_since_last_valid_sample-the-silence) counts the silence. Each valid sample replaces the values of the two attributes, so the validator keeps no history of samples.

What a call of `drop_reason_or_record()` changes in the validator's attributes depends on how the call ends. The table lists the three cases:

| How the call ends | `last_valid_stamp` and `last_valid_arrival_time` | `last_checked_ros_time`, which the next clock check compares with |
|:--|:--|:--|
| The sample is valid | replaced by this sample's stamp and `now` | `now` |
| A check drops the sample | unchanged, or set to `None` when `now` shows that the clock went back | `now` |
| The ROS time is 0 s ([`NO_CLOCK`](#dropreason-the-reasons-to-drop-a-sample)) | unchanged | unchanged |

So the stamp of a dropped sample never becomes $t_\text{last}$. The node counts every dropped sample by its reason, `NO_CLOCK` included, and passes a warning to `rospy.logwarn_throttle()` with the period `WARNING_PERIOD`, 5 s.

#### Why one method runs both steps

Two places in the code call `drop_reason_or_record()`: the node's callback [`on_sample()`](#where-the-ros-time-comes-from), once for each sample that arrives, and the tests, through [`assert_accepted()` and `assert_rejected()`](#validatortestcase-a-new-validator-and-two-assertions). Neither calls [`drop_reason()`](#step-1-drop_reason-the-decision) and [`record_valid_sample_times()`](#step-2-record_valid_sample_times-a-valid-sample) itself. So neither can get the two steps wrong in one of two ways:

- **Recording a dropped sample's times:** check 5 would compare later stamps with the stamp of a sample that the checks dropped. If a caller recorded the times of a sample from the future, otherwise valid samples with stamps no later than that stored stamp would be dropped as [`NOT_NEWER`](#dropreason-the-reasons-to-drop-a-sample).
- **Not recording a valid sample's times:** check 5 would still let the same stamp through, and the silence would keep counting from the arrival time that was recorded last. If no arrival time had been recorded, the silence would stay `None`. Such omissions could make the diagnostics report an error while valid samples arrive.

The tests therefore run the same two steps as the node.

### `clear_last_valid_sample_times_if_clock_went_back()`: a clock that goes back

The ROS time can jump back. In the simulation, a call of the service `/gazebo/reset_simulation` starts Gazebo's clock again near 0 s. On the robot, a correction of the SBC's clock can set the time back. Without a reaction, [check 5](../../robot_localization.md#check-5-the-stamp-against-the-last-forwarded-sample) would still compare new stamps with the stamp $t_\text{last}$ of the last valid sample from before the jump. Samples that pass checks 1 to 4 would then be dropped as [`NOT_NEWER`](#dropreason-the-reasons-to-drop-a-sample) while their stamps are no later than $t_\text{last}$. For example, suppose the last valid stamp is 100 s and the simulation restarts near 0 s. If each new sample's stamp equals its arrival time, those samples would be dropped until their stamps exceed 100 s, about 100 s of simulation time later.

When the clock went back, the method sets the validator's attributes `last_valid_stamp` and `last_valid_arrival_time` to `None`. These two attributes hold the stamp of the last valid sample and the ROS time of its arrival, the last valid sample's times of the method's name. The validator stores no sample, only these two times. The validator also stores the ROS time of the last clock check in its attribute `last_checked_ros_time`. The excerpt shows the method and its two helpers, from [`imu_validator.py`, lines 232–256]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L232-L256):

```python title="imu_validator.py: clear_last_valid_sample_times_if_clock_went_back() and its two helpers" linenums="232"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:232:256"
```

- **`clock_went_back(now)`:** returns whether `now` is earlier than `last_checked_ros_time`, in line 248. Before the first clock check, `last_checked_ros_time` is `None`, and the answer is `False`.
- **`clear_last_valid_sample_times()`:** sets the attributes `last_valid_stamp` and `last_valid_arrival_time` to `None`, in lines 255 and 256, as before the first valid sample. `clear_last_valid_sample_times()` undoes what [`record_valid_sample_times()`](#step-2-record_valid_sample_times-a-valid-sample) stores.
- **`clear_last_valid_sample_times_if_clock_went_back(now)`:** calls `clear_last_valid_sample_times()` only when `clock_went_back(now)` is true, in lines 242 and 243. Then line 244 stores `now` in `last_checked_ros_time` for the next clock check, whether the clock went back or not.

After a jump back, the validator starts over as if it had seen no valid sample yet:

- **Check 5:** with no stamp $t_\text{last}$ to compare with, check 5 lets the next sample through. If checks 1 to 4 pass too, the sample is valid, and its stamp becomes the new $t_\text{last}$.
- **The silence:** [`seconds_since_last_valid_sample()`](#seconds_since_last_valid_sample-the-silence) returns `None` until a sample is valid again. A diagnostics report in that time shows the error "no IMU sample forwarded yet, or since the clock went back". At 50 Hz, samples are spaced 20 ms apart. The silence becomes a number again only when a sample passes all five checks.

The table follows two samples through a new validator, one that has seen no sample yet, with the values of the test `test_validator_accepts_a_stamp_once_after_the_clock_goes_back` in [A clock that goes back](#a-clock-that-goes-back):

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

`seconds_since_last_valid_sample(now)` returns how long no sample has been valid at the ROS time `now`. The [validator's diagnostics](../../robot_localization.md#the-validators-diagnostics) report this silence. The excerpt shows the method, from [`imu_validator.py`, lines 342–356]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L342-L356):

```python title="imu_validator.py: seconds_since_last_valid_sample()" linenums="342"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:342:356"
```

The method first calls [`clear_last_valid_sample_times_if_clock_went_back()`](#clear_last_valid_sample_times_if_clock_went_back-a-clock-that-goes-back), so the validator notices a clock that went back also while no sample arrives. Then the method returns one of two things:

- **`None`:** no sample was valid since the clock last went back, or none at all.
- **The silence in s, a `float`:** the ROS time $t$ minus $t_\text{valid}$, the ROS time `last_valid_arrival_time` at which the last valid sample arrived: $t - t_\text{valid}$. The silence counts from the sample's arrival, not from its stamp.

### `DropReason`: the reasons to drop a sample

`DropReason` is an [enumeration](https://docs.python.org/3/library/enum.html), a fixed set of named members, one for each reason to drop a sample. The excerpt shows the class, from [`imu_validator.py`, lines 105–118]({{ ekf_test_src_url }}/diffbot_control/scripts/imu_validator.py#L105-L118):

```python title="imu_validator.py: DropReason" linenums="105"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/scripts/imu_validator.py:105:118"
```

Each member's value is a text. The node writes the text into its warning in the log, as [`on_sample()`](#where-the-ros-time-comes-from) shows. The diagnostics count the dropped samples by reason, each under a key that starts with `Dropped,` and ends with the text, as `Dropped, too old` in the example of the [validator's diagnostics](../../robot_localization.md#the-validators-diagnostics). The table lists the seven members with the check that returns each:

| Member | Returned by | Text in the log and on `/diagnostics` |
|:--|:--|:--|
| `NO_CLOCK` | [`drop_reason()`](#step-1-drop_reason-the-decision) itself, before the checks, at the ROS time 0 | `no ROS clock` |
| `WRONG_FRAME` | [Check 1](../../robot_localization.md#checks-1-to-3-the-samples-form), `check_frame()` | `wrong frame` |
| `NOT_FINITE` | [Check 2](../../robot_localization.md#checks-1-to-3-the-samples-form), `check_finite_values()` | `not finite` |
| `INVALID_ANGULAR_VELOCITY_COVARIANCE` | [Check 3](../../robot_localization.md#checks-1-to-3-the-samples-form), `check_angular_velocity_covariance()` | `invalid angular velocity covariance` |
| `FROM_THE_FUTURE` | [Check 4](../../robot_localization.md#check-4-the-stamp-against-the-ros-time), `check_not_too_far_ahead()` | `from the future` |
| `TOO_OLD` | [Check 4](../../robot_localization.md#check-4-the-stamp-against-the-ros-time), `check_not_too_old()` | `too old` |
| `NOT_NEWER` | [Check 5](../../robot_localization.md#check-5-the-stamp-against-the-last-forwarded-sample), `check_stamp_later_than_last_valid()` | `not newer than the last one` |

The tests import the module `imu_validator` and not its names. So a test writes a member with the module's name in front, as `imu_validator.DropReason.TOO_OLD`.

## The constants and the helpers

The tests share a few constants, two helper functions that build samples and durations, and a base class with two assertions, [`ValidatorTestCase`](#validatortestcase-a-new-validator-and-two-assertions).

??? example "The constants of `test_imu_validator.py`, lines 23–65"

    The excerpt shows all constants of the file, from [`test_imu_validator.py`, lines 23–65]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L23-L65). `NOW` is the ROS time $t$ of most checks, 100 s. `MAX_AGE` and `MAX_FUTURE` are the two time limits that [`ekf.launch`]({{ diffbot_repo_url }}/diffbot_control/launch/ekf.launch) sets on the robot and in the simulation. `JUST_TOO_OLD` and `JUST_TOO_FAR_AHEAD` lie `ONE_MICROSECOND`, 1 µs, beyond these limits. The tests that use the other constants explain them in their sections, and the comment above each constant says which condition its value meets. `ONE_NANOSECOND` is a `rospy.Duration` of exactly 1 ns, built from whole nanoseconds and not with `seconds()`. `seconds()` passes a number of seconds to `rospy.Duration.from_sec()`, which multiplies the number by 10⁹ as a floating-point number and cuts off the fraction of a nanosecond. So `seconds(3.3)` is 3.299999999 s.

    ```python title="test_imu_validator.py: the constants" linenums="23"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:23:65"
    ```

Every test that gives the validator a sample builds the sample with the helper `make_imu_sample()`. The excerpt shows `make_imu_sample()` and the helper `seconds()`, from [`test_imu_validator.py`, lines 68–89]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L68-L89):

```python title="test_imu_validator.py: make_imu_sample() and seconds()" linenums="68"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:68:89"
```

- **`make_imu_sample()`:** returns a `sensor_msgs/Imu` message like the firmware's. With the defaults of its three arguments, the sample is valid at the ROS time `NOW` for a new validator, one that has seen no sample yet. If the last valid sample has the same stamp, [check 5](../../robot_localization.md#check-5-the-stamp-against-the-last-forwarded-sample) drops the new sample. With the defaults, the sample's stamp $t_s$ is `NOW`, its frame is `imu_link` and its yaw rate $\omega_z$ is `YAW_RATE`, 0.5 rad/s. The accelerometer's reading on the upward axis, $f_z$, is `GRAVITY`, 9.81 m/s², as for an IMU at rest. `orientation_covariance[0]` is −1, which marks the orientation as unavailable, as in the firmware's message.
- **The variances:** each variance on the diagonal of the angular velocity's covariance $\boldsymbol\Sigma_{\boldsymbol\omega}$, such as $\sigma^2_{\omega_z}$, is `ANGULAR_VELOCITY_VARIANCE`, 4 · 10⁻⁴ (rad/s)². Each variance on the diagonal of `linear_acceleration_covariance` is `LINEAR_ACCELERATION_VARIANCE`, 10⁻² (m/s²)². Both have the size of the firmware's variances.
- **Valid and invalid samples:** the tests override the defaults of `stamp`, `frame_id` or `yaw_rate`, or change a field of the returned message, to build the samples that they need. A later stamp gives the next valid sample, for example. An invalid sample differs from the valid one in one property, so that the sample aims at one check. A sample can fail a later check too, as sample A of [the time limits](#check-4-the-two-time-limits) does.
- **`seconds()`:** turns a number of seconds into a `rospy.Duration`, which a test adds to `NOW` or subtracts from it.

### `ValidatorTestCase`: a new validator and two assertions

[`TestImuValidator`](#testimuvalidator-which-samples-are-valid) and [`TestSecondsSinceLastValidSample`](#testsecondssincelastvalidsample-the-silence) derive from the class `ValidatorTestCase`, which derives from `unittest.TestCase`. A class that derives from another class inherits the other class's methods. So the tests of both classes can use the three methods of `ValidatorTestCase`: `setUp()` and the two assertions `assert_accepted()` and `assert_rejected()`. `ValidatorTestCase` itself has no tests. The excerpt shows the class, from [`test_imu_validator.py`, lines 92–110]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L92-L110):

```python title="test_imu_validator.py: ValidatorTestCase" linenums="92"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:92:110"
```

- **`setUp()`:** unittest calls `setUp()` before each test method. So each test of a class that derives from `ValidatorTestCase` starts with a new `ImuValidator` in `self.validator`, one that has seen no sample yet. No test depends on the samples of another test. `FRAME`, `MAX_AGE` and `MAX_FUTURE` are the values that [`ekf.launch`]({{ diffbot_repo_url }}/diffbot_control/launch/ekf.launch) gives the validator's node.
- **`assert_accepted(message, now)`:** calls [`drop_reason_or_record(message, now)`](#drop_reason_or_record-the-decision-about-a-sample) once, the same method that the node calls once for each sample, and checks the result with `assertIsNone()`. A result of `None` means that the sample has no drop reason: the validator accepts the sample and stores the sample's stamp in `last_valid_stamp` and its arrival time in `last_valid_arrival_time`. For any other result, `assertIsNone()` stops the test, and unittest reports the test as failed, with the drop reason that came back instead of `None`.
- **`assert_rejected(reason, message, now)`:** calls `drop_reason_or_record(message, now)` once too, and compares the result with the expected `reason` by `assertEqual()`. `assertEqual()` compares its first two arguments and fails the test when they differ. The tests of this file give `assertEqual()` the expected value first and the result second, and `assert_rejected()` does the same. So the assertion holds when the validator rejects the sample with the expected reason. The validator doesn't store a rejected sample's stamp or arrival time in `last_valid_stamp` and `last_valid_arrival_time`. Only the clock check, which runs before the checks at any ROS time other than 0, stores the call's ROS time in `last_checked_ros_time`.
- **`msg`:** the optional last argument of both assertions. The type `Optional[str]` says that `msg` is a text or `None`, and `= None` makes `None` the default. When the assertion fails, unittest adds the text to its report of the failure. The tests of [check 1](#check-1-the-frame), [check 2](#check-2-the-values) and [check 3](#check-3-the-variances) run a loop over several cases and give each assertion a text that names the case, so that a failure says which case the validator decided wrongly.

`assertIsNone()` and `assertEqual()` are two of the [assert methods](https://docs.python.org/3/library/unittest.html#assert-methods) that every test class inherits from `unittest.TestCase`, which is why the two assertions call these methods on `self`. The tests call `assert_accepted()` and `assert_rejected()` on `self` in the same way.

Accepted and rejected are the validator's decisions, and the tests' names say "accepts" and "rejects". The node forwards an accepted sample and drops a rejected one. The tests themselves publish nothing, as [What the tests don't show](#what-the-tests-dont-show) says.

## `TestImuValidator`: which samples are valid

The class tests [`drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample), one reason to drop a sample at a time. The excerpt shows the class's first two lines, from [`test_imu_validator.py`, lines 113–114]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L113-L114). The name in parentheses, [`ValidatorTestCase`](#validatortestcase-a-new-validator-and-two-assertions), is the class that `TestImuValidator` derives from. So each test starts with a new validator and checks each decision of the validator with `assert_accepted()` or `assert_rejected()`:

```python title="test_imu_validator.py: the class derives from ValidatorTestCase" linenums="113"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:113:114"
```

The sections below follow the order of the checks, with the numbers and names of [the five checks](../../robot_localization.md#the-five-checks) on the Localization page.

### A valid sample

`test_validator_accepts_valid_samples` is the baseline of the class. The excerpt shows the test, from [`test_imu_validator.py`, lines 116–118]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L116-L118):

```python title="test_imu_validator.py: valid samples" linenums="116"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:116:118"
```

Line 117 holds two calls, one inside the other. Python runs the inner call first:

1. **The inner call:** [`make_imu_sample()`](#the-constants-and-the-helpers) returns a valid sample, stamped `NOW`.
2. **The outer call:** `self.assert_accepted(make_imu_sample(), NOW)` gets two arguments. The first is the sample from the inner call. The second is the ROS time $t$ of the sample's arrival, `NOW`, 100 s. The assertion [`assert_accepted()`](#validatortestcase-a-new-validator-and-two-assertions) calls [`drop_reason_or_record()`](#drop_reason_or_record-the-decision-about-a-sample) with both and checks that the method returns `None`, no drop reason. Otherwise the test stops there, and unittest reports the test as failed, with the drop reason that came back instead of `None`.

So line 117 checks that the validator accepts the sample. In the node, [`on_sample()`](#where-the-ros-time-comes-from) forwards an accepted sample.

Line 118 checks a second sample in the same way. The sample is stamped `MAX_FUTURE`, 0.02 s, later than the first, and arrives at a ROS time that is 0.02 s later too. This sample is the next one of the IMU at 50 Hz. The validator accepts the sample, because its stamp is later than the first sample's.

### Before the checks: no clock

`test_validator_rejects_a_sample_at_ros_time_zero` gives the validator a valid sample at the ROS time 0, the test's constant [`ROS_TIME_ZERO`](#the-constants-and-the-helpers), `rospy.Time(0)`. The excerpt shows this test and the next one, from [`test_imu_validator.py`, lines 120–134]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L120-L134):

```python title="test_imu_validator.py: no clock, and what a sample at 0 s leaves" linenums="120"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:120:134"
```

[`assert_rejected()`](#validatortestcase-a-new-validator-and-two-assertions) gets the expected reason first, here [`imu_validator.DropReason.NO_CLOCK`](#dropreason-the-reasons-to-drop-a-sample), then the sample and the ROS time. So the first test checks that the validator rejects even a valid sample at the ROS time 0, with the reason `NO_CLOCK`.

The validator treats the ROS time 0 as an unavailable clock. At startup with simulation time, `rospy.Time.now()` returns 0 until the node has received the first message on the topic `/clock`, as [Where the ROS time comes from](#where-the-ros-time-comes-from) explains. A stamp can't be judged against this time. At $t = 0$, check 4 would allow only the stamps between −0.1 s and 0.02 s:

$$
t - \texttt{max\_age} \le t_s \le t + \texttt{max\_future}
\quad\Rightarrow\quad
-0.1\ \text{s} \le t_s \le 0.02\ \text{s}
$$

- **A sample from Gazebo:** a valid sample stamped, for example, 0.5 s would be rejected as from the future. The log and the diagnostics would blame the sample's stamp, although the node's clock is missing.
- **A sample without a stamp:** a message whose stamp was never set has the stamp 0 s, and would pass check 4.

So the validator rejects every sample at the ROS time 0 before the checks, with its own reason `NO_CLOCK`, which [`drop_reason()`](#step-1-drop_reason-the-decision) returns. On the robot, the ROS time is the SBC's system clock, which doesn't read 0. So the robot's validator doesn't reject samples for this reason.

`test_validator_rejects_the_same_stamp_again_after_a_sample_at_ros_time_zero` checks that a sample at the ROS time 0 leaves `last_valid_stamp` as it is, because [`drop_reason()`](#step-1-drop_reason-the-decision) returns `NO_CLOCK` before the clock check. The test gives one validator three samples:

1. **The 1st call, line 127, at the ROS time `NOW`, 100 s:** a sample stamped `NOW` is valid. The validator stores 100 s in its attributes `last_valid_stamp` and `last_checked_ros_time`.
2. **The 2nd call, lines 128 and 129, at `ROS_TIME_ZERO`, 0 s:** a sample stamped `SHORTLY_BEFORE` before `NOW`, 99.99 s. The validator returns `NO_CLOCK` before the clock check. Both attributes stay 100 s.
3. **The 3rd call, lines 133 and 134, at `NOW` plus `ONE_MILLISECOND`, 100.001 s:** a sample stamped `NOW` again. The clock check finds 100.001 s later than `last_checked_ros_time`, 100 s, and stores 100.001 s. `last_valid_stamp` is still 100 s, so [check 5](#check-5-the-order-of-the-stamps) rejects the sample as `NOT_NEWER`.

The third call's own clock check finds no clock that went back, because its ROS time is not earlier than `NOW`. So only the second call could have changed `last_valid_stamp`. A validator that checked the clock before it returned `NO_CLOCK` would take 0 s for a clock that went back and set its attributes `last_valid_stamp` and `last_valid_arrival_time` to `None`. A validator that stored the second sample's stamp would compare the third stamp with 99.99 s. Either would accept the third sample, and the test fails for both. [Two tests of the silence](#after-a-sample-at-the-ros-time-0) check the other two attributes.

### Check 1: the frame

Check 1 compares the sample's `header.frame_id` with the frame that the validator expects, [`FRAME`](#the-constants-and-the-helpers), `imu_link`. The excerpt shows the test, from [`test_imu_validator.py`, lines 136–140]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L136-L140):

```python title="test_imu_validator.py: a wrong frame" linenums="136"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:136:140"
```

The loop runs twice, with the frame `base_link` and with an empty frame. In each pass, [`assert_rejected()`](#validatortestcase-a-new-validator-and-two-assertions) expects [`DropReason.WRONG_FRAME`](#dropreason-the-reasons-to-drop-a-sample). The last argument is the assertion's `msg`, the text that unittest adds to its report when the assertion fails. The text names the frame of the loop's current pass, so a failure says which of the two frames the validator didn't reject.

### Check 2: the values

Check 2 needs finite values in the angular velocity $\boldsymbol\omega$, the field `angular_velocity`, and in the accelerometer's reading $\mathbf f$, the field `linear_acceleration`. The excerpt shows the test, from [`test_imu_validator.py`, lines 142–151]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L142-L151):

```python title="test_imu_validator.py: NaN and infinite values" linenums="142"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:142:151"
```

The loop runs three times, for NaN, +∞ and −∞. Each pass builds two samples. The first has the value as its yaw rate $\omega_z$, `angular_velocity.z`. The second has the value as $f_x$, `linear_acceleration.x`. The validator has to reject all six samples with [`DropReason.NOT_FINITE`](#dropreason-the-reasons-to-drop-a-sample).

### Check 3: the variances

Check 3 looks at the covariance $\boldsymbol\Sigma_{\boldsymbol\omega}$ of the angular velocity $\boldsymbol\omega$, the field `angular_velocity_covariance`. Its nine values have to be finite, and its three variances on the diagonal, $\sigma^2_{\omega_x}$, $\sigma^2_{\omega_y}$ and $\sigma^2_{\omega_z}$, have to be positive. [The variances](../../robot_localization.md#the-variances) on the Localization page shows how the message stores the 3 × 3 matrix as nine numbers, row by row. The excerpt shows the test, from [`test_imu_validator.py`, lines 153–164]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L153-L164):

```python title="test_imu_validator.py: three invalid covariances" linenums="153"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:153:164"
```

The test builds three samples, each with one defect in $\boldsymbol\Sigma_{\boldsymbol\omega}$, and expects [`DropReason.INVALID_ANGULAR_VELOCITY_COVARIANCE`](#dropreason-the-reasons-to-drop-a-sample) for each. The text in each pair of `cases` is the `msg` of a failure:

- **`unavailable`:** the first element is −1 and the others are 0. This is how a `sensor_msgs/Imu` message says that it carries no angular velocity. Check 3 reads the three variances $\sigma^2_{\omega_x}$, $\sigma^2_{\omega_y}$ and $\sigma^2_{\omega_z}$ as −1, 0 and 0 (rad/s)², so none of them is positive.
- **`zero_variance`:** the variance of the yaw rate $\sigma^2_{\omega_z}$, the element with the index [`YAW_RATE_VARIANCE_INDEX`](#the-constants-and-the-helpers), 8, is 0 (rad/s)².
- **`nan_off_the_diagonal`:** the covariance $\sigma_{\omega_x \omega_y}$, the element with the index `OFF_DIAGONAL_INDEX`, 1, is NaN. This element is not a variance, so the test shows that the check looks at all nine values.

### Check 4: the two time limits

Check 4 compares the stamp $t_s$ with the ROS time $t$: $t - \texttt{max\_age} \le t_s \le t + \texttt{max\_future}$. Two tests probe each limit from both sides. The excerpt shows both tests, from [`test_imu_validator.py`, lines 166–174]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L166-L174):

```python title="test_imu_validator.py: at the limits and 1 µs beyond" linenums="166"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:166:174"
```

The figure below shows the samples of these two tests on a time axis. The sections on [check 5](#check-5-the-order-of-the-stamps) and on [a clock that goes back](#a-clock-that-goes-back) have figures of the same kind. Read the three figures as follows:

- **The axis:** time runs to the right. One tick is 0.02 s, the IMU's sample period at 50 Hz. Each axis has its own scale.
- **A mark:** stands at the stamp $t_s$ of its sample. Its letter is the sample's name in the text. A solid green mark is a sample that the validator has to accept, a dashed red mark one that the validator has to reject.
- **The bands:** a stamp in a green band passes the check. A stamp in a red band is rejected for the reason that the band names.
- **Two slanted strokes:** the axis leaves out a stretch of time there.
- **The list below a figure:** says for each sample what the node does with the sample, and why. "Forwarded" stands for a sample that the validator accepts, "dropped" for one that the validator rejects.

Two samples that lie only 1 µs or 1 ns apart would share one spot at the figure's scale, so the figures draw them apart.

--8<-- "includes/localization-test-validator-limits.html"

The first test sends B and then A, the second test D and then C, all at the ROS time [`NOW`](#the-constants-and-the-helpers). The tests build the four stamps from the constants: B is `NOW - seconds(MAX_AGE)`, A is `NOW - seconds(JUST_TOO_OLD)`, D is `NOW + seconds(JUST_TOO_FAR_AHEAD)` and C is `NOW + seconds(MAX_FUTURE)`. The validator has to accept B and C, so each limit itself is still valid. The validator has to reject A with [`DropReason.TOO_OLD`](#dropreason-the-reasons-to-drop-a-sample) and D with `DropReason.FROM_THE_FUTURE`.

In the first test, sample A is also older than the valid sample B before it. The validator reports `DropReason.TOO_OLD` and not `DropReason.NOT_NEWER`, because the checks run [in their order](#step-1-drop_reason-the-decision) and check 4 comes before check 5. The second test sends the rejected sample D first. For a rejected sample at a ROS time other than 0, the validator stores only the ROS time of the call in `last_checked_ros_time`, for its clock check, so D doesn't change what the validator decides for C.

### Check 5: the order of the stamps

Check 5 compares the stamp $t_s$ with $t_\text{last}$, the stamp of the last valid sample: $t_s > t_\text{last}$. The excerpt shows the two tests of this check, from [`test_imu_validator.py`, lines 176–194]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L176-L194):

```python title="test_imu_validator.py: the order check" linenums="176"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:176:194"
```

The figure shows the samples of the two tests around $t_\text{last}$, 100 s:

--8<-- "includes/localization-test-validator-order.html"

`test_validator_rejects_a_sample_not_later_than_the_last_valid_one` sends four samples to one validator:

1. A valid sample stamped [`NOW`](#the-constants-and-the-helpers). The validator stores `NOW` in `last_valid_stamp`, as $t_\text{last}$.
2. Sample F, stamped `NOW` again. The validator rejects the sample with [`DropReason.NOT_NEWER`](#dropreason-the-reasons-to-drop-a-sample), because F's stamp is not later than $t_\text{last}$. A stamp is valid only once, until the clock goes back.
3. Sample E, stamped `SHORTLY_BEFORE`, 0.01 s, before `NOW`. `SHORTLY_BEFORE` is a tenth of `MAX_AGE`, so check 4 lets E through, and check 5 rejects the sample with the same reason.
4. Sample G, stamped `ONE_NANOSECOND`, 1 ns, after `NOW`, the smallest step of a ROS stamp. G arrives `ONE_MILLISECOND`, 1 ms, after `NOW`, so after its own stamp. A ROS time before `NOW` would be a clock that went back and would clear `last_valid_stamp`. The validator accepts the sample.

`test_validator_accepts_a_new_sample_after_rejecting_a_future_one` shows what $t_\text{last}$ is not. After a valid sample stamped `NOW`, the test sends sample H, stamped `FAR_AHEAD`, 5 s, ahead. Check 4 rejects H with `DropReason.FROM_THE_FUTURE`. Then the test sends sample I, stamped 0.02 s after `NOW`. The validator accepts I. If the validator stored the stamp of the rejected sample H in `last_valid_stamp`, every sample of the next 5 s would count as not newer.

### A clock that goes back

As [`clear_last_valid_sample_times_if_clock_went_back()`](#clear_last_valid_sample_times_if_clock_went_back-a-clock-that-goes-back) explains, the validator sets its attributes `last_valid_stamp` and `last_valid_arrival_time` to `None` when a call checks the clock and the call's `now` is earlier than `last_checked_ros_time`, the ROS time of the last clock check. Every diagnostics report checks the clock, and so does every sample that arrives at a ROS time other than 0. A sample at the ROS time 0 returns [`NO_CLOCK`](#dropreason-the-reasons-to-drop-a-sample) before the clock check and changes nothing. After the validator has cleared `last_valid_stamp` and `last_valid_arrival_time`, the order check, [check 5](#check-5-the-order-of-the-stamps), starts over. The excerpt shows the two tests, from [`test_imu_validator.py`, lines 196–213]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L196-L213):

```python title="test_imu_validator.py: the clock goes back" linenums="196"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:196:213"
```

The figure shows the two tests, each on its own axis at the new ROS time:

--8<-- "includes/localization-test-validator-clock-back.html"

- **`test_validator_accepts_a_stamp_once_after_the_clock_goes_back`:** the first sample is stamped [`NOW`](#the-constants-and-the-helpers), 100 s, and arrives at the ROS time `NOW`. The validator accepts this sample. Then the ROS time is [`RESTART_NEAR_ZERO`](#the-constants-and-the-helpers), 0.02 s, as after a reset of the simulation. This test is the first axis of the figure. Sample J is stamped `RESTART_NEAR_ZERO` and arrives at the ROS time `RESTART_NEAR_ZERO`. This stamp is far older than $t_\text{last}$, 100 s. The validator accepts J all the same, because the order check started over. The order check works again from this sample on: the test sends the same stamp once more, `ONE_MILLISECOND`, 1 ms, later, and expects `DropReason.NOT_NEWER`. This second sample is K.
- **`test_validator_rejects_stamps_outside_time_limits_after_the_clock_goes_back`:** starting over applies to the order check only. The first sample is again stamped `NOW`, 100 s, and arrives at the ROS time `NOW`. Then the ROS time is `RESTART`, 50 s, and both of the next samples arrive at this ROS time. This test is the second axis of the figure. Sample M is stamped 100.02 s, `NOW` plus `MAX_FUTURE`. This stamp was valid before the clock went back and is 50.02 s ahead of the new ROS time. The validator rejects M with `DropReason.FROM_THE_FUTURE`. Sample L is stamped 49 s, `ONE_SECOND`, 1 s, before `RESTART`, and the validator rejects L with `DropReason.TOO_OLD`.

## `TestSecondsSinceLastValidSample`: the silence

The [validator's diagnostics](../../robot_localization.md#the-validators-diagnostics) report an error when no sample was forwarded for too long. The time behind this error comes from [`seconds_since_last_valid_sample()`](#seconds_since_last_valid_sample-the-silence), which returns the silence $t - t_\text{valid}$ in s, or `None` when no sample was valid since the clock last went back.

The class derives from [`ValidatorTestCase`](#validatortestcase-a-new-validator-and-two-assertions) too. So each test starts with a new validator, and the tests give the validator their samples through `assert_accepted()` and `assert_rejected()`. The class adds a helper of its own, `assert_silence()`, for the comparison of the silence. The excerpt shows the class's first lines and the helper, from [`test_imu_validator.py`, lines 216–224]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L216-L224):

```python title="test_imu_validator.py: the class and assert_silence()" linenums="216"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:216:224"
```

`assert_silence(expected, now)` calls `seconds_since_last_valid_sample(now)` and compares the result with `expected`. If the method returns `None`, line 223 fails the test with a message that names the expected silence. Otherwise `assertAlmostEqual()` rounds the difference of the two numbers to seven decimal places, its default, and expects zero.

The class's nine tests come in four groups, each with a figure. A figure draws each test of its group in a drawing of its own. Read the drawings as follows:

- **A row:** one call of the test, in the order of the test's lines, from top to bottom. The list below the drawing gives each call with the same number, the test's names for its times and their values.
- **A circle:** a sample that the test gives the validator. A solid green circle is a sample that the validator accepts. A dashed red circle is a sample that the validator rejects, with the drop reason beside it.
- **A square:** a call of `seconds_since_last_valid_sample()`, with `None` beside it or the silence that it returns.
- **The axis:** the ROS time $t$ of the calls. Each mark stands at the ROS time of its call. Two slanted strokes leave out a stretch of time, and each part of the axis between them has its own scale.
- **A triangle:** a sample's stamp $t_s$, where it differs from the ROS time of the sample's arrival. A dotted line joins the triangle to the sample's mark.
- **A green line:** `last_valid_arrival_time`, $t_\text{valid}$. The line runs down from the valid sample that stored it. A cross ends the line above the row of the call that sets the attribute to `None`.
- **A dashed gray line:** `last_checked_ros_time`, the ROS time of the last clock check. The line runs down from each call that checks the clock to the next call that checks it. A sample at the ROS time 0 doesn't check the clock, so the line passes its row. Where the dashed line and the green line stand at the same time, the dashed line lies on the green one.
- **An arrow to the left, "back":** the call's ROS time is earlier than `last_checked_ros_time`, so the clock went back.
- **A dimension line:** the silence $t - t_\text{valid}$ that a call of `seconds_since_last_valid_sample()` returns, with the test's name for the expected value.

### Before and after a valid sample

The first three tests show where the silence counts from. The first test asks a new validator for the silence. The second asks after one valid sample, and the third after a valid sample and a rejected one:

--8<-- "includes/localization-test-validator-silence-grows.html"

The excerpt shows the three tests, from [`test_imu_validator.py`, lines 226–242]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L226-L242):

```python title="test_imu_validator.py: the silence grows" linenums="226"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:226:242"
```

- **Before the first valid sample:** the method returns `None`.
- **After a valid sample:** the valid sample is stamped `SHORTLY_BEFORE`, 0.01 s, before it arrives at `NOW`. `SILENCE`, 3.5 s, after the arrival, the method returns 3.5 s and not 3.51 s, because the silence counts from the arrival and not from the stamp. `SILENCE` has a fraction of a second, so a silence counted in whole seconds would fail too.
- **While samples are rejected:** a sample that the validator rejects with [`DropReason.TOO_OLD`](#dropreason-the-reasons-to-drop-a-sample) doesn't end the silence. The rejected sample is stamped `ONE_SECOND` after `NOW`, later than the valid sample's stamp, so [check 5](#check-5-the-order-of-the-stamps) would let it through. The sample arrives `SILENCE` after `NOW`, when its stamp is 2.5 s old, more than `MAX_AGE`. So check 4 rejects the sample before check 5 runs. At the same ROS time, `SILENCE`, 3.5 s, after the valid sample, the method returns 3.5 s.

### After a sample at the ROS time 0

A sample at the ROS time 0 returns [`NO_CLOCK`](#dropreason-the-reasons-to-drop-a-sample) before the clock check and leaves the validator's attributes as they are. A test in [Before the checks: no clock](#before-the-checks-no-clock) checks this for `last_valid_stamp`. The next two tests check the other two attributes, `last_valid_arrival_time` and `last_checked_ros_time`. Both give the validator the same two samples, a valid one at `NOW` and one at the ROS time 0. They differ in the ROS time at which they ask for the silence:

--8<-- "includes/localization-test-validator-silence-zero.html"

The excerpt shows the two tests, from [`test_imu_validator.py`, lines 244–260]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L244-L260):

```python title="test_imu_validator.py: the silence after a sample at 0 s" linenums="244"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:244:260"
```

- **The arrival time:** a valid sample arrives at `NOW`, and a sample stamped 99.99 s, `SHORTLY_BEFORE` before `NOW`, returns `NO_CLOCK` at `ROS_TIME_ZERO`. At 100.1 s, `RECENT` after `NOW`, the method returns 0.1 s, counted from the arrival at `NOW`. A validator that cleared `last_valid_arrival_time` at 0 s would return `None`, and one that stored the stamp 99.99 s there would return 0.11 s.
- **The last clock check:** the same two calls, then the method is asked at 99.999 s, `ONE_MILLISECOND` before `NOW`. `last_checked_ros_time` is still 100 s, so the clock went back, and the method returns `None`. A validator that stored 0 s or the stamp 99.99 s as the last clock check would find no clock that went back and return −0.001 s.

### After the clock goes back

The next three tests are about a clock that goes back while no valid sample arrives. In each test, a valid sample arrives at `NOW`, and a later call has an earlier ROS time:

--8<-- "includes/localization-test-validator-silence-clock-back.html"

The excerpt shows the three tests, from [`test_imu_validator.py`, lines 262–288]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L262-L288):

```python title="test_imu_validator.py: the silence after the clock goes back" linenums="262"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:262:288"
```

- **Without samples:** a valid sample arrives at the ROS time [`NOW`](#the-constants-and-the-helpers), 100 s. Then the method is asked at the ROS time `RESTART`, 50 s, and `SILENCE` later, at 53.5 s, and returns `None` both times. The method itself notices that the clock went back, because the method checks the clock with [`clear_last_valid_sample_times_if_clock_went_back()`](#clear_last_valid_sample_times_if_clock_went_back-a-clock-that-goes-back) in each call.
- **Back by less than the silence:** a valid sample arrives at the ROS time `NOW`, 100 s. The method is asked at the ROS time 103.5 s, `SILENCE` after `NOW`, and returns a silence of 3.5 s. Then the method is asked at 100.1 s, `NOW` plus `RECENT`. The clock went back by 3.4 s, less than the silence, so the new ROS time is still later than the arrival of the valid sample, and $t - t_\text{valid}$ would be 0.1 s, a silence that [`health()`](#testhealth-the-status) reports as OK. The method returns `None` all the same, and at 101 s, `ONE_SECOND` after `NOW`, as well. A sample from before the clock went back says nothing about the IMU in the new time.
- **Shown by a rejected sample:** a valid sample arrives at the ROS time `NOW`, 100 s, and the method returns a silence of 3.5 s at 103.5 s. Then the clock goes back. The first call in the new time is a sample that arrives at 100.1 s, `NOW` plus `RECENT`, stamped 99 s, `ONE_SECOND` before `NOW`, which the validator rejects as too old. The validator checks the clock also for a sample that a check rejects, so in this call the validator sets its attributes `last_valid_stamp` and `last_valid_arrival_time` to `None`. The method returns `None` at 104.5 s, `SILENCE` plus `ONE_SECOND` after `NOW`. This last call is later than the call at 103.5 s, so its own clock check finds no clock that went back. Only the rejected sample's call can have cleared the two attributes. A sample at the ROS time 0 would not show that the clock went back: the validator rejects such a sample before the clock check.

### The first valid sample after the clock went back

The last test of the class follows the silence through a whole restart, from a valid sample before the clock goes back to the first valid sample after it:

--8<-- "includes/localization-test-validator-silence-restart.html"

The excerpt shows the test, from [`test_imu_validator.py`, lines 290–303]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L290-L303):

```python title="test_imu_validator.py: the silence starts again" linenums="290"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:290:303"
```

A valid sample arrives at the ROS time `NOW`, 100 s. Then the ROS time is `RESTART`, 50 s. At this ROS time, the validator rejects a sample from the future and a sample with the wrong frame. At 51 s, `ONE_SECOND` after `RESTART`, the method still returns `None`. At 53.5 s, `SILENCE` after `RESTART`, a valid sample arrives. At 53.6 s, `RECENT`, 0.1 s, after that sample, the method returns 0.1 s.

## `TestHealth`: the status

`health()` turns two numbers into the level and the message of the diagnostics status: the [silence](#seconds_since_last_valid_sample-the-silence) $t - t_\text{valid}$ in s, or `None`, and the number of samples dropped since the report before. The function reads no clock and keeps no state, so each call is independent of the others. The excerpt shows the class's five tests, from [`test_imu_validator.py`, lines 309–331]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L309-L331):

```python title="test_imu_validator.py: the level of the status" linenums="309"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:309:331"
```

| Silence | New drops | Level that the test expects |
|:--|:--|:--|
| `None`: no valid sample yet | 0 | `ERROR` |
| `SILENCE_LIMIT`, 1 s | 0 | `OK` |
| `SILENCE_LIMIT` plus [`ONE_MILLISECOND`](#the-constants-and-the-helpers), 1.001 s | 0 | `ERROR` |
| `RECENT`, 0.1 s | `NEW_DROP_COUNT`, 3 | `WARN`, and the message contains the number 3 |
| `RECENT`, 0.1 s | 0 | `OK` |
| `None`, and 1.001 s | 3 | `ERROR` for both: silence comes before the warning |

`SILENCE_LIMIT` is the validator's own constant, which the tests read from `imu_validator`. So a test doesn't repeat the limit's value. The test checks that the limit itself is still OK and that 1 ms more is an error.

## `TestSampleCounts`: the numbers of a report

`SampleCounts` counts the forwarded samples and the dropped samples by their reason. `take_snapshot()` returns the numbers of one diagnostics report as a `DiagnosticSnapshot`. The excerpt shows the first two tests, from [`test_imu_validator.py`, lines 340–358]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L340-L358):

```python title="test_imu_validator.py: the counts of a snapshot" linenums="340"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:340:358"
```

- **The counts:** after two forwarded samples, two samples dropped as too old and one dropped for its frame, the snapshot has `forwarded_count` 2 and `drops_by_reason` with 2 and 1. The snapshot also carries the [silence](#seconds_since_last_valid_sample-the-silence) that the test passed to `take_snapshot()`.
- **New drops:** `new_drop_count` is the number of samples dropped since the snapshot before. One dropped sample makes this number 1 in the next snapshot and 0 in the one after. Two more dropped samples, for two different reasons, make the number 2. [`health()`](#testhealth-the-status) uses this number for the report's level: a new drop can raise the next report to `WARN` when the IMU is otherwise healthy. The report after it is `OK` again if no further sample was dropped and the IMU is still healthy. A silence that is too long, or no valid sample yet, takes priority: the report is then `ERROR`, as the last row of the table in [`TestHealth`](#testhealth-the-status) shows.

The other two tests cover two cases in which this count could go wrong. The excerpt shows them, from [`test_imu_validator.py`, lines 360–376]({{ ekf_test_src_url }}/diffbot_control/test/test_imu_validator.py#L360-L376):

```python title="test_imu_validator.py: a snapshot in an error report, and a snapshot that stays" linenums="360"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_imu_validator.py:360:376"
```

- **In an error report:** a sample is dropped before any sample was valid. The first snapshot gets the silence `None`, so `health()` reports an error, not the warning. The dropped sample counts as new in this snapshot all the same, and not again in the next one.
- **A snapshot stays as it is:** after `take_snapshot()`, the test counts one more forwarded sample and one more dropped sample. The snapshot from before still has its numbers. A report is built from one snapshot, so its numbers all come from one moment.

## Run the tests

catkin runs the file with nose, a test runner for Python, as part of the package's tests. The excerpt shows how `diffbot_control` registers its three EKF tests, from [`CMakeLists.txt`, lines 226–234]({{ ekf_test_src_url }}/diffbot_control/CMakeLists.txt#L226-L234). [`catkin_add_nosetests()`](https://github.com/ros/catkin/blob/noetic-devel/doc/howto/format2/python_nose_configuration.rst) registers this file. [`add_rostest()`](https://github.com/ros/catkin/blob/noetic-devel/doc/howto/format2/rostest_configuration.rst) registers the two scenarios of the [stepped-clock test](stepped-clock.md), one `.test` file each:

```cmake title="diffbot_control/CMakeLists.txt: the package's tests" linenums="226"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/CMakeLists.txt:226:234"
```

CI runs all three with the workspace's other tests, as [Testing and CI](../../../../development/ci.md#ekf-tests) describes. To run only this file, open a shell in which the workspace is sourced and start the file with Python's `unittest` from its folder:

```console
roscd diffbot_control/test
python3 -m unittest -v test_imu_validator
```

The collapsed box shows the whole output of a run at the commit [`3882a19`](https://github.com/ros-mobile-robots/diffbot/commit/3882a19cbda8543f5a0f1c5d4f0965caa90d4f3d) of [diffbot#220](https://github.com/ros-mobile-robots/diffbot/pull/220), in the [dev container](../../../../development/dev-container.md)'s image with ROS Noetic and Python 3.8.10. The run started `python3 -m unittest -v test_imu_validator` in the folder `diffbot_control/test`, the folder that `roscd` changes to. With the option `-v`, unittest prints one line for each test: the test's name, its class in parentheses, and `ok` for a test that passed. unittest runs the classes, and the tests in each class, in the alphabetical order of their names, so the output's order differs from the file's. [`ValidatorTestCase`](#validatortestcase-a-new-validator-and-two-assertions) has no tests, so the output has no line for this class. The last lines give the number of tests and the time that they took, and `OK` when every test passed:

??? example "The output of `python3 -m unittest -v test_imu_validator`"

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

## What the tests don't show

- **The node:** the class [`ImuValidatorNode`](#where-the-ros-time-comes-from) reads the parameters, subscribes to the topic `imu/data_raw`, publishes on the topic `imu/data_validated` and on the topic `/diagnostics`, and reads the ROS time in a lock around each sample. The unit tests don't create an `ImuValidatorNode`: they call the methods of `ImuValidator` directly. The [stepped-clock test](stepped-clock.md) runs the node with the EKF and checks its counts on the topic `/diagnostics`.
- **The real clock and the real IMU:** every time in these tests is a number that the test chose. Whether `max_age` and `max_future` fit the delays of the robot's USB link is a measurement on the robot, as [Limits](../../robot_localization.md#limits) says. The issue [IMU validator: measure the samples' age on the robot, then set max_age and max_future (diffbot#219)](https://github.com/ros-mobile-robots/diffbot/issues/219) measures it.
- **The fields that the validator doesn't check:** the [overview](../../robot_localization.md#what-the-validator-doesnt-check) lists them. No test sends a sample with an implausible but finite value, because the validator accepts such a sample by design. The EKF doesn't drop such a value either. The first measurement initializes the filter. For each later measurement, robot_localization's EKF computes the difference between the measurement and the filter's prediction and corrects the estimate with this difference, unless a rejection threshold finds the difference too large ([`ekf.cpp`, lines 164–189](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ekf.cpp#L164-L189)), as the same section of the overview explains. The IMU's threshold for the angular velocity, `imu0_twist_rejection_threshold`, defaults to the largest `double` ([`ros_filter.cpp`, line 1418](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ros_filter.cpp#L1418)), and [`ekf.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/ekf.yaml) sets none. So a large but finite yaw rate in a valid sample moves the estimate.
