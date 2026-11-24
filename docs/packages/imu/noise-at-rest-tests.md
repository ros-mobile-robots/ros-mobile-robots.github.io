# Tests: IMU Bias and Noise at Rest

What the tests of the tool [`imu_noise_at_rest.py`](driver-bno055.md#bias-and-noise-at-rest) check, and how they do it. [Bias and noise at rest](driver-bno055.md#bias-and-noise-at-rest) on the BNO055 page explains the tool itself: how to record the standing robot, how to run the tool and what its report means.

[`test_imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py) holds 82 tests for [`imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py). The first 59 call one function of the tool with a few values whose result can be calculated by hand. The last 23 run the tool as a command, on small bags that they write into a temporary folder. No test needs a robot or a ROS master. The tests prove the tool's arithmetic, its checks of the input and the text of its report. [What the tests don't show](#what-the-tests-dont-show) lists what a pass doesn't prove.

## Run the tests

The tests run in the [dev container](../../development/dev-container.md), in the [catkin](../../development/dev-container.md#in-the-container) workspace:

```console
cd ~/catkin_ws
catkin run_tests diffbot_bringup
catkin_test_results build/diffbot_bringup
```

The last command prints the result:

```
Summary: 82 tests, 0 errors, 0 failures, 0 skipped
```

catkin finds the tests through the package's [`CMakeLists.txt`]({{ imu_noise_src_url }}/diffbot_bringup/CMakeLists.txt#L217-L221), which registers the file for [nose](https://nose.readthedocs.io/), a runner for Python's [`unittest`](https://docs.python.org/3.8/library/unittest.html) tests. The excerpt shows the registration, [lines 217–221 of `CMakeLists.txt`]({{ imu_noise_src_url }}/diffbot_bringup/CMakeLists.txt#L217-L221):

```cmake title="diffbot_bringup/CMakeLists.txt: the test file is registered for nose" linenums="217"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/CMakeLists.txt:217:221"
```

[CI](../../development/ci.md) runs the same tests with every pull request.

## The test classes

The file holds the tests in fourteen test classes. The list has them in the order in which the tool uses the functions they test, from one message to the whole report:

- **[`TestMeasuredVector` and `TestToSample`](#which-readings-count), 2 + 2 tests:** they call [`measured_vector()`](#which-readings-count) and [`to_sample()`](#which-readings-count). They check which fields of a message count as measured, and that the header stamp keeps every nanosecond.
- **[`TestTopicChecks` and `TestSampleChecks`](#the-checks-of-the-input), 8 + 10 tests:** they call [`check_topic()`](#the-checks-of-the-input), the six checks of the samples and [`check_samples()`](#the-checks-of-the-input). They check which messages the tool refuses, and the reason that it gives.
- **[`TestAxisStatistics`](#the-statistics-of-one-axis), 8 tests:** they call [`axis_statistics()`](#the-statistics-of-one-axis). They check the mean, the sample variance, the standard deviation and the distinct values of one axis.
- **[`TestVectorStatistics` and `TestFieldStatistics`](#the-statistics-of-one-axis), 1 + 2 tests:** they call [`vector_statistics()`](#the-statistics-of-one-axis) and [`field_statistics()`](#the-statistics-of-one-axis). They check that each axis has its own statistics, and that a field needs two readings.
- **[`TestStampStatistics`](#the-header-stamps), 10 tests:** they call [`stamp_statistics()`](#the-header-stamps). They check the duration, the rate and the intervals of the header stamps, and the stamps that don't advance.
- **[`TestImuStatistics`](#the-statistics-of-a-recording), 5 tests:** they call [`imu_statistics()`](#the-statistics-of-a-recording). They check that each statistic comes from the right field, and gravity.
- **[`TestFormatDuration`, `TestFormatLongestInterval`, `TestFormatTable` and `TestFormatReport`](#the-reports-text), 3 + 2 + 1 + 5 tests:** they call [`format_duration()`](#the-reports-text), [`format_longest_interval()`](#the-reports-text), [`format_table()`](#the-reports-text) and [`format_report()`](#the-reports-text). They check the text of the report, line by line.
- **[`TestToolOnBags`](#the-tool-as-a-command), 23 tests:** they run the script as a command. They check the report and status 0 for a good bag, and one line and status 1 for an input that the tool refuses.

Every test method has a name that says what it checks and a first docstring line that says the same as a sentence, for example [`test_stamp_statistics_counts_no_long_interval_at_exactly_long_interval`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L278-L281) with "Check that an interval of exactly LONG_INTERVAL isn't long: only a longer one is."

## Which readings count

A [`sensor_msgs/Imu`](driver-bno055.md#what-the-firmware-publishes) message has a place for the angular velocity and one for the acceleration, also when the IMU measured only one of them. The message marks a field that it didn't measure with −1 as the first element of the field's covariance. The tool's function [`measured_vector()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L376-L384) reads this mark. It gets a field's vector and the field's covariance, and returns the vector as a reading, or `None` for a field that wasn't measured. The excerpt shows the function, [lines 376–384 of `imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L376-L384):

```python title="imu_noise_at_rest.py: a field's reading, or None" linenums="376"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/scripts/imu_noise_at_rest.py:376:384"
```

The figure shows three messages as the test [`test_imu_statistics_takes_a_field_only_from_the_samples_that_measured_it`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L531-L543) builds them. The second message has no angular velocity:

--8<-- "includes/imu-noise-test-measured.html"

The tests of this part:

- **[`test_measured_vector_gives_none_when_the_covariance_starts_with_minus_one`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L228-L231):** the reading (0.1, 0.2, 0.3) rad/s with a covariance that starts with −1 gives `None`.
- **[`test_measured_vector_gives_the_reading_for_a_covariance_of_zeros`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L233-L235):** the same reading with a covariance of nine zeros, which means "not known", gives the reading.
- **[`test_to_sample_gives_each_field_by_its_own_covariance`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L247-L253):** a message whose [`angular_velocity_covariance`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L251) starts with −1 and whose acceleration is 9.81 m/s² on z gives a sample without the angular velocity and with the acceleration.
- **[`test_to_sample_keeps_one_nanosecond_of_the_header_stamp`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L241-L245):** a message stamped 1 ns after [`EPOCH`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L90), 1,700,000,000 s, gives a sample whose stamp is `EPOCH + 1` ns, as an integer.
- **`test_imu_statistics_takes_a_field_only_from_the_samples_that_measured_it`:** the three messages of the figure give two readings of the angular velocity with the mean 0.3 rad/s, and three readings of the acceleration.

The stamp is an integer for a reason. A stamp of today is about 1.7 · 10⁹ s. A floating-point number of that size has steps of about 240 ns, so two stamps in seconds as floats can't tell an interval of 30 ms from one of 30 ms and 1 ns. As integers in nanoseconds, the stamps and the intervals between them are exact.

## The checks of the input

Before the tool calculates anything, it checks its input. Each check is one function that raises a [`MessageError`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L89-L90) with the reason.

The first check, [`check_topic()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L365-L373), runs before the tool reads a message. A bag stores three things about the message type of each of a topic's connections: the type's name, the text of the type's definition, and a checksum of the definition that the recorder got from the publisher. The check asks the bag for all three and refuses three kinds of topics:

- **Another type:** a topic whose type isn't named [`sensor_msgs/Imu`](robot_localization.md#the-messages-and-their-fields). A camera's topic chosen by mistake is refused without loading its images.
- **A checksum that doesn't fit:** [rosbag](https://github.com/ros/ros_comm/blob/25d371664e34ec9d26ee331434de9a38c412c890/tools/rosbag/src/rosbag/bag.py#L1910-L1916) builds a message's class from the definition's text in the bag. The check builds the class in the same way. If the class's checksum isn't the stored one, the stored checksum and the stored definition disagree, and the bag doesn't say which of the two belongs to the messages. rosbag would try to decode the messages' bytes as the fields of the stored definition, whether the definition is theirs or not. So the tool refuses the topic before it reads a message.
- **Other fields:** messages that are named `sensor_msgs/Imu`, but whose class has other fields than ROS's. The check compares the class's fields, and the fields of its fields, with those of ROS's `sensor_msgs/Imu`.

[`TestTopicChecks`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L349) gives the check stand-ins for a bag, which only know a topic's connections:

- **[`test_check_topic_passes_a_topic_of_ros_imu_messages`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L352-L354):** a topic with ROS's definition of `sensor_msgs/Imu` raises nothing.
- **[`test_check_topic_passes_a_definition_that_differs_in_its_comments`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L356-L359):** the same definition with one more comment line raises nothing either.
- **[`test_check_topic_fails_for_a_topic_with_several_types_and_names_them_all`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L361-L374):** a topic with `sensor_msgs/Imu`, [`std_msgs/String`](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html) and [`std_msgs/UInt32`](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/UInt32.html) is refused with the reason `the topic holds sensor_msgs/Imu, std_msgs/String, std_msgs/UInt32. Every message of the topic must be a sensor_msgs/Imu`. The reason names all three, because `rosbag info` shows only one type for such a topic.
- **[`test_check_topic_fails_for_a_checksum_that_is_not_the_definitions`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L399-L411):** ROS's definition, stored with the checksum of `std_msgs/String`, is refused with a reason that has both checksums.
- **[`test_check_topic_fails_for_an_imu_definition_with_an_array_in_place_of_a_vector`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L376-L387):** a definition whose [`angular_velocity`](robot_localization.md#the-messages-and-their-fields) is an array of one vector is refused with the reason `angular_velocity is a geometry_msgs/Vector3[1] instead of a geometry_msgs/Vector3`. ROS gives this definition the same checksum as its own, so only the comparison of the fields finds the difference.
- **[`test_check_topic_fails_for_a_field_of_a_field_and_names_its_path`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L389-L397):** a definition whose quaternion has the field [`reading_w`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L389-L397) instead of [`w`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Quaternion.html) is refused with the reason `orientation has the fields x, y, z, reading_w instead of x, y, z, w`.
- **[`test_check_topic_fails_for_an_imu_definition_that_is_no_definition`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L413-L420):** a text that isn't a message definition is refused with the reason of [genpy](https://github.com/ros/genpy/blob/0.6.18/src/genpy/dynamic.py#L114-L124), the ROS library that reads definitions.
- **[`test_check_topic_fails_with_the_readers_reason_when_the_reader_fails`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L422-L428):** a bag whose reader fails, as rosbag's reader of an old bag format does, is refused with the reader's error as the reason.

The other six checks look at the samples, and [`check_samples()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L556-L559) runs them in the order of the list [`SAMPLE_CHECKS`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L546-L553). The excerpt shows the list and the function, [lines 544–559 of `imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L544-L559):

```python title="imu_noise_at_rest.py: the checks of the samples and their runner" linenums="544"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/scripts/imu_noise_at_rest.py:544:559"
```

[`TestSampleChecks`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L431) gives each check samples that it has to refuse, and compares the reason's text:

| Check | What goes in | The reason that is checked |
|:--|:--|:--|
| [`check_message_count()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L450-L454) | One sample | `1 message, the statistics need at least two` |
| [`check_named_frame()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L457-L467) | Two samples with an empty [`frame_id`](driver-bno055.md#what-the-firmware-publishes) | `2 of 2 messages have an empty frame_id. The readings need one named frame, which says what their axes are` |
| [`check_one_frame()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L470-L479) | A sample in [`other_imu`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L448-L454) and one in [`imu_link`](robot_localization.md#the-frames) | `the messages have more than one frame_id ("imu_link", "other_imu"), so their readings have different axes` |
| [`check_finite_readings()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L511-L519) | A sample whose yaw rate is NaN, stamped 20 ms after [`EPOCH`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L90) | `the message stamped 1700000000.020000000 has the angular_velocity (0.0, 0.0, nan), which isn't finite` |
| `check_finite_readings()` | A sample whose x acceleration is infinite | A `MessageError` |
| `check_finite_readings()` | A sample without the angular velocity | Nothing is raised: a field that wasn't measured isn't looked at |
| [`check_reading_size()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L522-L532) | A sample whose z acceleration is 2,000,000 m/s², twice [`LARGEST_READING`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L74) | `the message stamped 1700000000.000000000 has the linear_acceleration (0.0, 0.0, 2000000.0). The tool takes a value beyond 1e+06 m/s^2 as damaged data` |
| `check_reading_size()` | A sample whose x rate is −1,000,000 rad/s, exactly `LARGEST_READING` | Nothing is raised |
| [`check_reading_count()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L535-L541) | One sample without the angular velocity and one without the acceleration | `angular_velocity has 1 reading and linear_acceleration has 1 reading. At least one of them needs two` |
| `check_samples()` | Two samples at rest in `imu_link` | Nothing is raised |

The frame matters because it says which way the axes x, y and z point. Readings in two frames, or in a frame without a name, don't belong on one axis. The mean of +9.81 and −9.81 m/s² would be a gravity of zero.

`LARGEST_READING` is 1,000,000, in rad/s or m/s². It is the tool's own limit for a plausible reading of a robot at rest. The tool takes a value beyond it as damaged data, for example one changed byte in a number. One such value alone would decide the mean and the variance of its axis.

## The statistics of one axis

[`axis_statistics()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L592-L596) gets the readings of one axis and returns their mean, their sample variance and the number of distinct values. It takes the variance from [`sample_variance()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L577-L589), which calculates with fractions instead of floating-point numbers. [The mean and the variance of three readings](driver-bno055.md#the-mean-and-the-variance-of-three-readings) on the BNO055 page explains the formulas. The test of the sample variance uses the four values 1, 2, 3 and 4:

--8<-- "includes/imu-noise-test-variance.html"

The excerpt shows this test, [lines 147–154 of `test_imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L147-L154). [`assertAlmostEqual()`](https://docs.python.org/3.8/library/unittest.html#unittest.TestCase.assertAlmostEqual) compares the two numbers to seven decimal places:

```python title="test_imu_noise_at_rest.py: the sample variance divides by the number of values minus one" linenums="147"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/test/test_imu_noise_at_rest.py:147:154"
```

The other tests of the statistics:

- **[`test_axis_statistics_gives_the_mean`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L143-L145):** 1, 2, 3 and 6 have the mean 3.
- **[`test_axis_statistics_gives_the_standard_deviation_as_the_root_of_the_variance`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L156-L160):** the standard deviation of 1, 2, 3 and 4 is the square root of 5/3.
- **[`test_axis_statistics_counts_each_distinct_value_once`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L162-L165):** five readings on three output steps of 1/900 rad/s are three distinct values.
- **[`test_axis_statistics_gives_a_variance_of_zero_without_noise`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L167-L169):** five equal readings have a variance of exactly 0.
- **[`test_axis_statistics_gives_the_exact_variance_of_two_neighbouring_floats`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L171-L180):** 9.81 and the next floating-point number above it have the variance $(b - a)^2 / 2$, compared exactly. Their mean lies between two floating-point numbers. Rounded to one of them, the mean would give one deviation of zero and one of a whole step, and twice the variance. Python's `statistics.variance()` gets the exact result with floating-point numbers too, so the test checks the result. The fractions are needed for the next test.
- **[`test_axis_statistics_gives_a_variance_of_zero_for_deviations_too_small_to_square`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L182-L189):** the readings 7 · 10⁻³⁰⁴, 0 and 0 have a variance of 0, without an error. The squares of deviations that small are zero as floating-point numbers, and Python 3.8's [`statistics.variance()`](https://docs.python.org/3.8/library/statistics.html#statistics.variance) stops with an error on them. As fractions, the squares aren't zero, and only the result is rounded to a floating-point number, to 0.
- **[`test_axis_statistics_fails_with_fewer_than_two_values`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L191-L194):** one value raises a [`StatisticsError`](https://docs.python.org/3.8/library/statistics.html#statistics.StatisticsError), because a sample variance needs two values.
- **[`test_vector_statistics_keeps_the_axes_apart`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L200-L205):** the vectors (1, 10, 100) and (3, 30, 300) have the means 2, 20 and 200 on x, y and z.
- **[`test_field_statistics_gives_the_reading_count_and_the_axes_statistics`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L211-L216):** two readings give a count of 2 and the statistics of each axis.
- **[`test_field_statistics_gives_no_axes_statistics_for_one_reading`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L218-L220):** one reading gives a count of 1 and no statistics, instead of an error.

## The header stamps

[`stamp_statistics()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L624-L640) gets the header stamps in the order in which the bag stored the messages, as integers in nanoseconds. It returns the time from the first stamp to the last, the rate, the shortest and the longest interval between two consecutive stamps, and three counts: the intervals longer than [`LONG_INTERVAL`](#the-limit-of-a-long-interval), the stamps that repeat the stamp before, and the stamps that are earlier than the stamp before. The excerpt shows the function together with [`mean_rate()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L612-L621), [lines 612–640 of `imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L612-L640):

```python title="imu_noise_at_rest.py: the rate and the statistics of the header stamps" linenums="612"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/scripts/imu_noise_at_rest.py:612:640"
```

### Stamps that advance

The first two tests use five stamps that advance, with one long interval:

--8<-- "includes/imu-noise-test-intervals.html"

- **[`test_stamp_statistics_gives_the_duration_and_the_mean_rate`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L259-L264):** the duration is 95 ms and the rate 4 / 0.095 s. Five messages span four intervals, so the rate counts the intervals and not the messages.
- **[`test_stamp_statistics_gives_the_shortest_and_the_longest_interval`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L266-L270):** 20 ms and 35 ms, compared as integers, so to the nanosecond.
- **[`test_stamp_statistics_counts_no_repeated_or_earlier_stamp_in_increasing_stamps`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L289-L293):** both counts are 0.

### The limit of a long interval

[`LONG_INTERVAL`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L79-L82) is 30 ms: one and a half periods of the 20 ms at which the firmware takes its samples. Three tests put two stamps around this limit, at the size of a real stamp:

- **29,999,999 ns, 1 ns below the limit:** no long interval ([`test_stamp_statistics_counts_no_long_interval_one_nanosecond_below_long_interval`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L272-L276)).
- **30,000,000 ns, exactly the limit:** no long interval ([`test_stamp_statistics_counts_no_long_interval_at_exactly_long_interval`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L278-L281)).
- **30,000,001 ns, 1 ns above the limit:** one long interval ([`test_stamp_statistics_counts_a_long_interval_one_nanosecond_above_long_interval`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L283-L287)).

### Stamps that don't advance

A clock that is set back during a recording gives stamps that go back. The fixture has one repeated stamp and one earlier stamp:

--8<-- "includes/imu-noise-test-stamps-back.html"

- **[`test_stamp_statistics_counts_a_repeated_and_an_earlier_stamp_apart`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L295-L299):** one repeated stamp, one earlier stamp, and a shortest interval of −10 ms.
- **[`test_stamp_statistics_gives_no_rate_when_a_stamp_inside_the_recording_goes_back`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L301-L309):** the duration is 80 ms and the rate is `None`. Without this rule, the four intervals in 80 ms would read as a healthy 50 Hz.
- **[`test_stamp_statistics_gives_no_rate_when_one_stamp_repeats_among_advancing_ones`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L311-L317):** stamps at 0, 20, 20 and 40 ms give no rate and one repeated stamp. A repeat alone, without a stamp that goes back, is enough.
- **[`test_stamp_statistics_gives_no_rate_when_no_stamp_advances`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L319-L323):** three stamps of 0, a clock that was never set, give no rate and two repeated stamps.

## The statistics of a recording

[`imu_statistics()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L655-L667) puts the parts together: the stamps' statistics, the frame, the statistics of the two fields and gravity. Its tests check that each number comes from the right place:

- **[`test_imu_statistics_gives_the_stamps_statistics_and_the_frame`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L508-L512):** two samples 20 ms apart in [`imu_link`](robot_localization.md#the-frames) give a duration of 20 ms and the frame `imu_link`.
- **[`test_imu_statistics_keeps_the_angular_velocity_and_the_acceleration_apart`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L514-L522):** two samples with different yaw rates and z accelerations give the mean 0.4 rad/s for the yaw rates and 10 m/s² for the z accelerations.
- **[`test_imu_statistics_gives_gravity_as_the_length_of_the_mean_acceleration`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L524-L529):** the accelerations (1, 3, 5) and (3, 3, 7) m/s² give a gravity of 7 m/s².
- **[`test_imu_statistics_gives_no_gravity_without_the_acceleration`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L545-L549):** two samples without the acceleration give `None` for gravity, not 0.

The test of gravity uses a tilted IMU. The mean of its two accelerations is (2, 3, 6) m/s², and the length of that vector is a whole number:

--8<-- "includes/imu-noise-test-gravity.html"

The excerpt shows the test, [lines 524–529 of `test_imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L524-L529). [`sample()`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L118-L122) is the test file's helper for a sample of a level robot at rest, and [`known()`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L128-L132) fails the test if gravity is `None`:

```python title="test_imu_noise_at_rest.py: gravity is the length of the mean acceleration" linenums="524"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/test/test_imu_noise_at_rest.py:524:529"
```

## The report's text

The functions whose names start with `format_` turn the statistics into the report's lines. Their tests compare text:

- **[`TestFormatDuration`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L552):** 179.78 s reads `179.8 s`, and 40 ms reads `40.0 ms`. A duration below one second is given in milliseconds, so that a short recording doesn't read `0.0 s`. The unit follows the rounded value: 999.96 ms reads `1.0 s`, not `1000.0 ms`.
- **[`TestFormatLongestInterval`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L569):** [`format_longest_interval()`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L681-L690) gives the longest interval. An interval 40 µs longer than [`LONG_INTERVAL`](#the-limit-of-a-long-interval) reads `30.040000 ms`, with ns, because one decimal would read `30.0 ms` next to a count that calls it longer. An interval 1 ms longer reads like any duration, `31.0 ms`.
- **[`TestFormatTable`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L585):** each column of a table is as wide as its widest cell, with the first column aligned left and the others right.
- **[`TestFormatReport`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L594):** one test compares all 18 lines of a report with both fields' statistics. One compares the stamps' lines of a recording with a repeated and an earlier stamp, which read "no rate (some stamps don't advance)". One compares the two lines that a field without statistics gets instead of its table, and checks that the report then has no line about gravity. One checks that a [`frame_id`](driver-bno055.md#what-the-firmware-publishes) with a line break in it stays on the report's first line, with the line break shown as `\n`. One checks the intervals' line with a longest interval of 30.04 ms: `longest 30.040000 ms, 1 longer than 30 ms`.

## The tool as a command

The 23 tests of [`TestToolOnBags`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L776) run the script the way a user does. Each test prepares an input in a temporary folder: a bag, a file that isn't a bag, or no file at all. It starts the script as a new process and compares what the process prints.

[`write_bag()`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L712-L721) stores the messages in the order of its list, each with the time at which a recorder would have received it. The excerpt shows the function, [lines 712–721 of `test_imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L712-L721):

```python title="test_imu_noise_at_rest.py: write the messages to a bag" linenums="712"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/test/test_imu_noise_at_rest.py:712:721"
```

[`run_tool()`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L769-L773) starts the script and returns a [`ToolRun`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L761-L766): the process's exit status and what it wrote to its standard output and its standard error. The class's helpers run the tool on the test's bag and check what it did. [`run_on_bag()`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L785-L787) runs it and returns the `ToolRun`. [`assert_tool_printed_a_report()`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L789-L791) expects status 0 with nothing on the standard error, and the test then reads the report's lines from `run.stdout`. [`assert_tool_ends_with()`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L793-L795) expects status 1, nothing on the standard output and exactly one given line on the standard error. [`assert_tool_ends_with_a_reason_after()`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L797-L807) expects the same, for a line of which the test fixes only the start. The excerpt shows `ToolRun`, `run_tool()`, the class's [`setUp()`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L779-L783) and the helpers, [lines 761–807 of `test_imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L761-L807):

```python title="test_imu_noise_at_rest.py: run the tool and compare what it prints" linenums="761"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/test/test_imu_noise_at_rest.py:761:807"
```

### The whole report for three messages

[`test_tool_prints_the_report_of_three_readings_one_output_step_apart`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L809-L837) writes three messages 20 ms apart, in the frame [`imu_link`](robot_localization.md#the-frames), with gravity on z. Their yaw rates are 0, 1 and 2 output steps of 1/900 rad/s:

--8<-- "includes/imu-noise-three-readings.html"

The mean is one step, 0.00111 rad/s, and the sample variance one step squared, 1.235 · 10⁻⁶ (rad/s)². The test compares all 18 lines of the report, as the excerpt shows, [lines 809–837 of `test_imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L809-L837):

```python title="test_imu_noise_at_rest.py: every line of the report for three readings" linenums="809"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/test/test_imu_noise_at_rest.py:809:837"
```

The messages show both meanings of a covariance. Their [`angular_velocity_covariance`](#which-readings-count) and [`linear_acceleration_covariance`](#which-readings-count) are all zeros, which means "not known", and the report has these fields' statistics. Their [`orientation_covariance`](driver-bno055.md#what-the-firmware-publishes) starts with −1, as the firmware's does, which means that the messages have no orientation.

### Reports for other bags

Each of these tests changes the three messages in one way and compares the lines that change:

- **[`test_tool_reads_another_topic_with_the_topic_option`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L839-L845):** the three messages are on the topic `/imu/data` and are read with `--topic /imu/data`. The first line starts with `/imu/data: 3 messages`.
- **[`test_tool_shows_a_line_break_in_the_topic_on_the_first_line`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L847-L855):** the topic's name has a line break in it. The report's first line shows the line break as `\n`, and the report still has its 18 lines.
- **[`test_tool_gives_the_reason_instead_of_statistics_for_a_field_not_measured`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L857-L875):** every message marks its angular velocity as not measured. The report says `angular_velocity: 0 readings, the statistics need at least two` and has the acceleration's table as before.
- **[`test_tool_takes_the_stamps_from_the_headers_not_from_the_arrival_times`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L877-L890):** the third message reaches the recorder 500 ms late. The stamps' lines are as before, with 50.00 Hz and no long interval, because the header stamps can't show a late delivery.
- **[`test_tool_gives_no_rate_and_keeps_the_statistics_when_a_stamp_goes_back`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L909-L915):** the second message is stamped 10 ms before the first. The report says `no rate (some stamps don't advance)` and `1 earlier than the one before`, and has the yaw rate's row as before.
- **[`test_tool_gives_no_rate_when_the_recorder_clock_goes_back_with_the_stamps`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L917-L928):** the same stamps, and the recorder's clock goes back with them. The report is the same.

The last two tests differ in the recorder's clock. [rosbag](https://github.com/ros/ros_comm/blob/25d371664e34ec9d26ee331434de9a38c412c890/tools/rosbag/src/rosbag/bag.py#L1332-L1341)'s reader returns the messages sorted by the time at which the recorder received each one. If that clock is set back together with the IMU's stamps, for example because the robot's computer corrects its time, the sorted stamps advance again and the step back would be invisible. So the tool puts the messages back into the order in which the bag stored them, and the report is the same in both cases.

### Inputs that the tool refuses

For these inputs the tool prints one line and ends with status 1. The line starts with the file's name. For a problem with the messages, the topic follows. All tests but the last two of the list compare the whole line. The list gives the line's reason:

- **[`test_tool_ends_with_a_message_when_the_messages_have_two_frames`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L930-L938):** the second message is in the frame [`other_imu`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L930-L938), with gravity on −z. The reason is `the messages have more than one frame_id ("imu_link", "other_imu"), so their readings have different axes`.
- **[`test_tool_ends_with_a_message_when_the_messages_have_no_frame_id`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L940-L949):** all three messages have an empty [`frame_id`](driver-bno055.md#what-the-firmware-publishes). The reason is `3 of 3 messages have an empty frame_id. The readings need one named frame, which says what their axes are`.
- **[`test_tool_ends_with_a_message_when_a_reading_is_not_a_number`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L951-L958):** the second yaw rate is NaN. The reason is `the message stamped 1700000000.020000000 has the angular_velocity (0.0, 0.0, nan), which isn't finite`.
- **[`test_tool_ends_with_a_message_when_the_topic_has_another_type`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L960-L965):** the topic holds two [`std_msgs/String`](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html) messages. The reason is `the topic holds std_msgs/String. Every message of the topic must be a sensor_msgs/Imu`.
- **[`test_tool_ends_with_a_message_when_the_imu_messages_have_other_fields`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L967-L983):** the bag names the topic's type [`sensor_msgs/Imu`](robot_localization.md#the-messages-and-their-fields) and stores the definition of `std_msgs/String` for it. The reason starts with `the bag defines the topic's sensor_msgs/Imu messages with other fields than ROS's: the message has the fields data instead of header, orientation`.
- **[`test_tool_ends_with_a_message_when_the_checksum_is_not_the_definitions`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L985-L1000):** the bag stores ROS's definition for the topic under the checksum of `std_msgs/String`. The reason is `the bag's checksum of the topic's sensor_msgs/Imu messages is`, the stored checksum, `but the definition that the bag stores for them has the checksum` and ROS's checksum.
- **[`test_tool_shows_a_line_break_in_a_checksum_on_its_one_line`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L1002-L1017):** the stored checksum is ROS's, followed by a line break. The tool's line shows the line break as `\n`, so the line stays one line.
- **[`test_tool_ends_with_a_message_when_the_angular_velocity_is_an_array_of_one_vector`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L1019-L1034):** the messages' [`angular_velocity`](robot_localization.md#the-messages-and-their-fields) is an array of one vector. The reason is `the bag defines the topic's sensor_msgs/Imu messages with other fields than ROS's: angular_velocity is a geometry_msgs/Vector3[1] instead of a geometry_msgs/Vector3`.
- **[`test_tool_ends_with_a_message_when_the_topic_has_one_message`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L1036-L1040):** the reason is `1 message, the statistics need at least two`.
- **[`test_tool_ends_with_a_message_when_the_topic_is_not_in_the_bag`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L1042-L1047):** the three messages are read with `--topic /other`. The reason is `0 messages, the statistics need at least two`.
- **[`test_tool_ends_with_a_message_when_the_file_is_missing`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L1049-L1051):** a file name without a file. The reason is `can't be opened: No such file or directory`.
- **[`test_tool_ends_with_a_message_when_the_file_is_not_a_rosbag`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L1053-L1058):** a text file. The reason is `isn't a rosbag: a bag file starts with the text #ROSBAG`.
- **[`test_tool_ends_with_a_message_when_the_file_is_a_compressed_copy_of_a_bag`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L1060-L1067):** a bag packed with gzip. The reason is the same.
- **[`test_tool_ends_with_a_message_when_the_file_ends_after_the_first_line`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L1069-L1074):** a file that holds only a bag's first line, like a recording that was cut off before the recorder wrote the index. The reason is `has no index, like a recording that was cut off. "rosbag reindex" can repair such a file`.
- **[`test_tool_ends_with_a_message_when_the_messages_data_is_damaged`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L1076-L1085):** a bag with compressed messages, of which the test inverts one byte. The bag still opens, because its index is intact, and reading its messages fails. The line starts with the file, the topic and `a message can't be read from the bag:`.
- **[`test_tool_ends_with_a_message_when_a_message_lacks_its_last_byte`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L1087-L1094):** the third message's data is one byte too short for a `sensor_msgs/Imu`. The line starts as in the test before.

In the last two tests, the reason at the line's end comes from [rosbag](https://github.com/ros/ros_comm/tree/25d371664e34ec9d26ee331434de9a38c412c890/tools/rosbag) or from Python. These tests don't fix its words. They compare the line's start and check that a reason follows it, and that the line is the only one.

## What the tests don't show

- **No real IMU:** the tests' messages are made up. That the BNO055's readings on Remo have the bias and the noise of the [results](driver-bno055.md#results-on-remo) is a measurement, not a test.
- **No recorder:** the tests write their bags with [rosbag](https://github.com/ros/ros_comm/blob/25d371664e34ec9d26ee331434de9a38c412c890/tools/rosbag/src/rosbag/bag.py#L581-L591)'s Python interface. They don't run `rosbag record`, and they don't cover a recording that was cut off.
- **No long recording:** the largest test bag has three messages. A recording of 180 s has about 9,000.
- **Nothing about driving:** the tool describes a robot at rest. How much noise driving adds is a separate check.
