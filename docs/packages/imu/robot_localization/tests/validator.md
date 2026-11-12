# Tests: The IMU Sample Validator

What the unit tests of the IMU sample validator check, and how they do it. This page belongs to the [tests of the Localization](index.md).

[`test_imu_validator.py`]({{ diffbot_repo_url }}/diffbot_control/test/test_imu_validator.py) tests the [IMU sample validator](../../robot_localization.md#the-imu-sample-validator) one decision at a time. Its 27 unit tests call the validator's Python class and functions directly. No ROS master runs, no node starts and no message travels over a topic. The tests prove what the validator decides for a given sample at a given ROS time $t$, and what its diagnostics report. They don't prove that the node forwards the samples on its topic or that the EKF handles them. The [stepped-clock test](stepped-clock.md) checks the node together with the EKF.

The file has four test classes, one for each part of [`imu_validator.py`]({{ diffbot_repo_url }}/diffbot_control/scripts/imu_validator.py) that works without ROS communication:

| Test class | Tests | What it calls | What it checks |
|:--|:--|:--|:--|
| [`TestImuValidator`](#testimuvalidator-which-samples-are-valid) | 11 | `ImuValidator.validate_and_record()` | Which samples the validator forwards, and the reason for each sample that it drops |
| [`TestSecondsSinceLastValidSample`](#testsecondssincelastvalidsample-the-silence) | 7 | `ImuValidator.seconds_since_last_valid_sample()` | The time since the last valid sample, which the diagnostics report, also when the clock goes back |
| [`TestHealth`](#testhealth-the-status) | 5 | `health()` | The level of the diagnostics status: OK, warning or error |
| [`TestSampleCounts`](#testsamplecounts-the-numbers-of-a-report) | 4 | `SampleCounts` | The counts of forwarded and dropped samples in a diagnostics report |

[What the tests don't show](#what-the-tests-dont-show) lists what a pass doesn't prove.

## How the tests run

catkin runs the file with nose, a test runner for Python, as part of the package's tests. The excerpt shows how `diffbot_control` registers its two EKF tests, from [`CMakeLists.txt`, lines 226–232]({{ ekf_src_url }}/diffbot_control/CMakeLists.txt#L226-L232). `catkin_add_nosetests()` registers this file, and `add_rostest()` registers the [stepped-clock test](stepped-clock.md):

```cmake title="diffbot_control/CMakeLists.txt: the package's tests" linenums="226"
--8<-- "{{ ekf_src_raw }}/diffbot_control/CMakeLists.txt:226:232"
```

CI runs both with the workspace's other tests, as [Testing and CI](../../../../development/ci.md#ekf-tests) describes. To run only this file, open a shell in which the workspace is sourced and start the file with Python's `unittest` from its folder:

```console
roscd diffbot_control/test
python3 -m unittest -v test_imu_validator
```

In a run at the commit `3ef4a91` of [diffbot#154](https://github.com/ros-mobile-robots/diffbot/pull/154), the command printed one line for each test and ended with `Ran 27 tests in 0.003s` and `OK`.

## The clock and the sample of the tests

The validator never reads the ROS clock itself. `validate_and_record()` and `seconds_since_last_valid_sample()` get the ROS time $t$ as their argument `now`. So a test can choose any ROS time and any stamp $t_s$, and needs no clock and no waiting.

??? example "The constants of `test_imu_validator.py`, lines 22–47"

    The excerpt shows all constants of the file, from [`test_imu_validator.py`, lines 22–47]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L22-L47). `NOW` is the ROS time $t$ of most checks, 100 s. `MAX_AGE` and `MAX_FUTURE` are the two time limits that [`ekf.launch`]({{ diffbot_repo_url }}/diffbot_control/launch/ekf.launch) sets on the robot and in the simulation. `JUST_TOO_OLD` and `JUST_TOO_FAR_AHEAD` lie `ONE_MICROSECOND`, 1 µs, beyond these limits.

    ```python title="test_imu_validator.py: the constants" linenums="22"
    --8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:22:47"
    ```

Every test builds its samples with the helper `sample()`. The excerpt shows `sample()` and the helper `seconds()`, from [`test_imu_validator.py`, lines 50–66]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L50-L66):

```python title="test_imu_validator.py: sample() and seconds()" linenums="50"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:50:66"
```

- **`sample()`:** returns a `sensor_msgs/Imu` message that passes all five checks at the ROS time `NOW`. Its stamp $t_s$ is `NOW`, its frame is `imu_link`, its yaw rate $\omega_z$ is `YAW_RATE`, 0.5 rad/s, and its upward acceleration $f_z$ is `GRAVITY`. The variances $\sigma^2$ on the diagonals of the two covariances have the size of the firmware's. `orientation_covariance[0]` is −1, which marks the orientation as unavailable, as in the firmware's message.
- **The arguments:** a test passes `stamp`, `frame_id` or `yaw_rate` to change one property of the sample. For the other invalid samples, a test changes one field of the message that `sample()` returned. So each case aims at one check. A sample can fail a later check too, as the too-old sample of the time limits does.
- **`seconds()`:** turns a number of seconds into a `rospy.Duration`, which a test adds to `NOW` or subtracts from it.

## `TestImuValidator`: which samples are valid

The class tests `validate_and_record()`. The method runs the [five checks](../../robot_localization.md#the-five-checks) in their order. It returns `None` for a valid sample, which the node forwards, and otherwise the `DropReason` of the first check that fails. A valid sample is also recorded as the last valid one. The excerpt shows the class's `setUp()`, from [`test_imu_validator.py`, lines 69–73]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L69-L73). unittest calls `setUp()` before each test method, so each test starts with a new `ImuValidator` that has seen no sample:

```python title="test_imu_validator.py: a new validator for each test" linenums="69"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:69:73"
```

The figure shows the samples of the tests that are about time, on four time axes. On each axis, time runs to the right, and one tick is 0.02 s, the IMU's sample period at 50 Hz. A mark stands at the stamp $t_s$ of its sample. A stamp in a green band passes the check, and a stamp in a red band is dropped for the reason that the band names. Each axis has its own scale, and two slanted strokes show where an axis leaves out a stretch of time.

- **The first axis:** the two limits of check 4 around the ROS time $t$.
- **The second axis:** check 5 around $t_\text{last}$, the stamp of the last valid sample.
- **The third axis:** the test in which the clock goes back to 0.02 s. The bands are those of check 4 at the new ROS time.
- **The fourth axis:** the test in which the clock goes back to 50 s, with the bands of check 4 at the new ROS time.

The letters A to M are the samples of the tests in the sections below. The list below each axis says what the validator has to do with each sample, and why.

--8<-- "includes/localization-test-validator.html"

### A valid sample

`test_validator_forwards_valid_samples` is the baseline of the class. The excerpt shows the test, from [`test_imu_validator.py`, lines 75–78]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L75-L78):

```python title="test_imu_validator.py: valid samples" linenums="75"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:75:78"
```

- **The first call:** the sample of `sample()` at the ROS time `NOW`. `assertIsNone()` checks that the validator returns no reason to drop the sample.
- **The second call:** a sample stamped `MAX_FUTURE`, 0.02 s, later, at a ROS time that is 0.02 s later too. This sample is the next one of the IMU at 50 Hz. The validator forwards it, because its stamp is later than the first sample's.

### The clock and the frame

Two tests cover the two reasons that don't depend on the sample's values. The excerpt shows both, from [`test_imu_validator.py`, lines 80–89]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L80-L89):

```python title="test_imu_validator.py: no clock, and a wrong frame" linenums="80"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:80:89"
```

- **`DropReason.NO_CLOCK`:** at the ROS time zero, `rospy.Time(0)`, the validator drops even the valid sample. With simulation time, the ROS time is zero until the first message on the topic `/clock` arrives.
- **`DropReason.WRONG_FRAME`:** check 1 drops a sample whose `header.frame_id` is `base_link` or empty. The third argument of `assertEqual()` is the message of a failure. It names the frame of the loop's current pass.

### The values

Check 2 needs finite values. The excerpt shows the test, from [`test_imu_validator.py`, lines 91–102]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L91-L102):

```python title="test_imu_validator.py: NaN and infinite values" linenums="91"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:91:102"
```

The loop runs three times, for NaN, +∞ and −∞. Each pass builds two samples. The first has the value as its yaw rate $\omega_z$, `angular_velocity.z`. The second has the value in `linear_acceleration.x`. The validator has to drop all six samples with `DropReason.NOT_FINITE`.

### The variances

Check 3 looks at the covariance of the angular velocity. The excerpt shows the test, from [`test_imu_validator.py`, lines 104–115]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L104-L115):

```python title="test_imu_validator.py: three invalid covariances" linenums="104"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:104:115"
```

The test builds three samples, each with one defect in `angular_velocity_covariance`, and expects `DropReason.INVALID_ANGULAR_VELOCITY_COVARIANCE` for each:

- **`unavailable`:** the first element is −1 and the others are 0. This is how a `sensor_msgs/Imu` message says that it carries no angular velocity.
- **`zero_variance`:** the variance $\sigma^2$ of the yaw rate, the element with the index `YAW_RATE_VARIANCE_INDEX`, 8, is 0.
- **`nan_off_the_diagonal`:** the element with the index `OFF_DIAGONAL_INDEX`, 1, is NaN. This element is not a variance, so the test shows that the check looks at all nine values.

### The two time limits

Check 4 compares the stamp $t_s$ with the ROS time $t$: $t - \texttt{max\_age} \le t_s \le t + \texttt{max\_future}$. Two tests probe each limit from both sides. The excerpt shows both tests, from [`test_imu_validator.py`, lines 117–128]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L117-L128):

```python title="test_imu_validator.py: at the limits and 1 µs beyond" linenums="117"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:117:128"
```

- **Sample B:** its stamp is exactly `MAX_AGE`, 0.1 s, older than `NOW`. The validator forwards it, so the limit itself is still valid.
- **Sample A:** its stamp is `JUST_TOO_OLD`, 0.100001 s, older than `NOW`. The validator drops it with `DropReason.TOO_OLD`.
- **Sample D:** its stamp is `JUST_TOO_FAR_AHEAD`, 0.020001 s, ahead of `NOW`. The validator drops it with `DropReason.FROM_THE_FUTURE`.
- **Sample C:** its stamp is exactly `MAX_FUTURE`, 0.02 s, ahead of `NOW`. The validator forwards it.

In the first test, sample A is also older than the valid sample B before it. The validator reports `DropReason.TOO_OLD` and not `DropReason.NOT_NEWER`, because the checks run in their order and check 4 comes before check 5. The second test sends the dropped sample D first. For a dropped sample, the validator records only the ROS time of the call, so D doesn't change what the validator decides for C.

### The order of the stamps

Check 5 compares the stamp $t_s$ with $t_\text{last}$, the stamp of the last valid sample: $t_s > t_\text{last}$. The excerpt shows the two tests of this check, from [`test_imu_validator.py`, lines 130–145]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L130-L145):

```python title="test_imu_validator.py: the order check" linenums="130"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:130:145"
```

`test_validator_drops_a_sample_not_later_than_the_last_valid_one` sends four samples to one validator:

1. A valid sample stamped `NOW`. The validator records `NOW` as $t_\text{last}$.
2. Sample F, stamped `NOW` again. The validator drops it with `DropReason.NOT_NEWER`, because a sample is valid only once.
3. Sample E, stamped 0.01 s before `NOW`. The validator drops it with the same reason.
4. Sample G, stamped 1 ns after `NOW`, the smallest step of a ROS stamp, at a ROS time 1 ms later. The validator forwards it.

`test_validator_forwards_a_new_sample_after_dropping_a_future_one` shows what $t_\text{last}$ is not. After a valid sample stamped `NOW`, the test sends sample H, stamped 5 s ahead. Check 4 drops H with `DropReason.FROM_THE_FUTURE`. Then the test sends sample I, stamped 0.02 s after `NOW`. The validator forwards I. If the validator had kept the stamp of the dropped sample H as $t_\text{last}$, every sample of the next 5 s would count as not newer.

### A clock that goes back

The ROS time goes back after a call of the service `/gazebo/reset_simulation` in the simulation, or when the SBC's clock is corrected on the robot. The validator notices this in `observe_time()`, when a call's `now` is earlier than the `now` of the call before. It then forgets the last valid sample, so that the order check starts over. The excerpt shows the two tests, from [`test_imu_validator.py`, lines 147–164]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L147-L164):

```python title="test_imu_validator.py: the clock goes back" linenums="147"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:147:164"
```

- **`test_validator_forwards_new_samples_after_the_clock_goes_back`:** after a valid sample at 100 s, the ROS time is `restart`, 0.02 s, as after a reset of the simulation. This test is the third axis of the figure. Sample J is stamped `restart`, which is far older than $t_\text{last}$, 100 s. The validator forwards J all the same, because the order check started over. The order check works again from this sample on: the test sends the same stamp once more, 1 ms later, and expects `DropReason.NOT_NEWER`. This second sample is K.
- **`test_validator_still_drops_old_and_future_samples_after_the_clock_goes_back`:** starting over applies to the order check only. After a valid sample at 100 s, the ROS time is `restart`, 50 s. This test is the fourth axis of the figure. Sample M is stamped 100.02 s, which was valid before the clock went back and is 50.02 s ahead of the new ROS time. The validator drops it with `DropReason.FROM_THE_FUTURE`. Sample L is stamped 1 s before `restart`, and the validator drops it with `DropReason.TOO_OLD`.

## `TestSecondsSinceLastValidSample`: the silence

The [validator's diagnostics](../../robot_localization.md#the-validators-diagnostics) report an error when no sample was forwarded for too long. The time behind this error comes from `seconds_since_last_valid_sample()`. The method returns the ROS time in seconds since the last valid sample was recorded, or `None` when no sample was valid since the clock last went back. The class has a helper for the comparison. The excerpt shows the helper and the first three tests, from [`test_imu_validator.py`, lines 173–192]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L173-L192):

```python title="test_imu_validator.py: the silence grows" linenums="173"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:173:192"
```

- **`assert_silence()`:** calls the method at the ROS time `now` and compares the result with `expected`. `assertAlmostEqual()` rounds the difference of the two numbers to seven decimal places, its default, and expects zero.
- **Before the first valid sample:** the method returns `None`.
- **After a valid sample:** 1.5 s after the sample was recorded, the method returns 1.5 s.
- **While samples are dropped:** a sample that the validator drops with `DropReason.TOO_OLD` doesn't end the silence. 3 s after the valid sample, the method returns 3 s.

The next three tests are about a clock that goes back while no valid sample arrives. The excerpt shows them, from [`test_imu_validator.py`, lines 194–212]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L194-L212):

```python title="test_imu_validator.py: the silence after the clock goes back" linenums="194"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:194:212"
```

- **Without samples:** after a valid sample at 100 s, the method is asked at 50 s and at 55 s and returns `None` both times. The method itself notices that the clock went back, because it records the ROS time of each call.
- **A little back:** the method is asked 3.3 s after the valid sample and then 0.2 s after it. The clock went back by less than the silence, so the difference of the two times would still be positive. The method returns `None` here too, and 1 s after the valid sample as well. A sample from before the clock went back says nothing about the IMU in the new time.
- **Shown by a dropped sample:** the clock goes back between two calls, and the first call in the new time is a sample that the validator drops. The validator records the ROS time of dropped samples too, so the method returns `None` afterwards.

The last test of the class follows the silence through a whole restart. The excerpt shows the test, from [`test_imu_validator.py`, lines 214–226]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L214-L226):

```python title="test_imu_validator.py: the silence starts again" linenums="214"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:214:226"
```

After a valid sample at 100 s, the ROS time is `restart`, 50 s. The validator drops a sample from the future and a sample with the wrong frame. 1 s later, the method still returns `None`. 2 s after `restart`, a valid sample arrives. 0.5 s after that sample, the method returns 0.5 s.

## `TestHealth`: the status

`health()` turns two numbers into the level and the message of the diagnostics status: the silence in seconds, or `None`, and the number of samples dropped since the report before. The function reads no clock and keeps no state, so each call is independent of the others. The excerpt shows the class's five tests, from [`test_imu_validator.py`, lines 232–253]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L232-L253):

```python title="test_imu_validator.py: the level of the status" linenums="232"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:232:253"
```

| Silence | New drops | Level that the test expects |
|:--|:--|:--|
| `None`: no valid sample yet | 0 | `ERROR` |
| `SILENCE_LIMIT`, 1 s | 0 | `OK` |
| `SILENCE_LIMIT + JUST_BEYOND`, 1.001 s | 0 | `ERROR` |
| `RECENT`, 0.1 s | `NEW_DROP_COUNT`, 3 | `WARN`, and the message contains the number 3 |
| `RECENT`, 0.1 s | 0 | `OK` |
| `None`, and 1.001 s | 3 | `ERROR` for both: silence comes before the warning |

`SILENCE_LIMIT` is the validator's own constant, which the tests read from `imu_validator`. So a test doesn't repeat the limit's value. It checks that the limit itself is still OK and that 1 ms more is an error.

## `TestSampleCounts`: the numbers of a report

`SampleCounts` counts the forwarded samples and the dropped samples by their reason. `take_snapshot()` returns the numbers of one diagnostics report as a `DiagnosticSnapshot`. The excerpt shows the first two tests, from [`test_imu_validator.py`, lines 262–280]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L262-L280):

```python title="test_imu_validator.py: the counts of a snapshot" linenums="262"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:262:280"
```

- **The counts:** after two forwarded samples, two samples dropped as too old and one dropped for its frame, the snapshot has `forwarded_count` 2 and `drops_by_reason` with 2 and 1. The snapshot also carries the silence that the test passed to `take_snapshot()`.
- **New drops:** `new_drop_count` is the number of samples dropped since the snapshot before. One dropped sample makes it 1 in the next snapshot and 0 in the one after. Two more dropped samples, for two different reasons, make it 2. The warning of `health()` depends on this number, so a dropped sample causes one warning, not a warning in every report from then on.

The other two tests cover two cases in which this count could go wrong. The excerpt shows them, from [`test_imu_validator.py`, lines 282–298]({{ ekf_src_url }}/diffbot_control/test/test_imu_validator.py#L282-L298):

```python title="test_imu_validator.py: a snapshot in an error report, and a snapshot that stays" linenums="282"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_imu_validator.py:282:298"
```

- **In an error report:** a sample is dropped before any sample was valid. The first snapshot gets the silence `None`, so `health()` reports an error, not the warning. The dropped sample counts as new in this snapshot all the same, and not again in the next one.
- **A snapshot stays as it is:** after `take_snapshot()`, the test counts one more forwarded sample and one more dropped sample. The snapshot from before still has its numbers. A report is built from one snapshot, so its numbers all come from one moment.

## What the tests don't show

- **The node:** the class `ImuValidatorNode` reads the parameters, subscribes to the topic `imu/data_raw`, publishes on the topic `imu/data_validated` and on the topic `/diagnostics`, and takes a lock around each sample. These tests don't call it. The [stepped-clock test](stepped-clock.md) runs the node with the EKF and checks its counts on the topic `/diagnostics`.
- **The real clock and the real IMU:** every time in these tests is a number that the test chose. Whether `max_age` and `max_future` fit the delays of the robot's USB link is a measurement on the robot, as [Limits](../../robot_localization.md#limits) says.
- **The fields that the validator doesn't check:** the [overview](../../robot_localization.md#what-the-validator-doesnt-check) lists them. No test sends a sample with an implausible but finite value, because the validator forwards such a sample by design.
