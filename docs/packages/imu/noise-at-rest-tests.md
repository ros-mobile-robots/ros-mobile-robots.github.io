# Tests: IMU Bias and Noise at Rest

What the tests of the tool `imu_noise_at_rest.py` check, and how they do it. [Bias and noise at rest](driver-bno055.md#bias-and-noise-at-rest) on the BNO055 page explains the tool itself: how to record the standing robot, how to run the tool and what its report means.

[`test_imu_noise_at_rest.py`]({{ diffbot_repo_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py) holds 77 tests for [`imu_noise_at_rest.py`]({{ diffbot_repo_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py). The first 54 call one function of the tool with a few values whose result can be calculated by hand. The last 23 run the tool as a command, on small bags that they write into a temporary folder. No test needs a robot or a ROS master. The tests prove the tool's arithmetic, its checks of the input and the text of its report. [What the tests don't show](#what-the-tests-dont-show) lists what a pass doesn't prove.

## Run the tests

The tests run in the [dev container](../../development/dev-container.md), in the catkin workspace:

```console
cd ~/catkin_ws
catkin run_tests diffbot_bringup
catkin_test_results build/diffbot_bringup
```

The last command prints the result:

```
Summary: 77 tests, 0 errors, 0 failures, 0 skipped
```

catkin finds the tests through the package's `CMakeLists.txt`, which registers the file for [nose](https://nose.readthedocs.io/), a runner for Python's `unittest` tests. The excerpt shows the registration, [lines 217–221 of `CMakeLists.txt`]({{ imu_noise_src_url }}/diffbot_bringup/CMakeLists.txt#L217-L221):

```cmake title="diffbot_bringup/CMakeLists.txt: the test file is registered for nose" linenums="217"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/CMakeLists.txt:217:221"
```

[CI](../../development/ci.md) runs the same tests with every pull request.

## The test classes

The file holds the tests in thirteen test classes. The list has them in the order in which the tool uses the functions they test, from one message to the whole report:

- **[`TestMeasuredVector` and `TestToSample`](#which-readings-count), 2 + 2 tests:** they call `measured_vector()` and `to_sample()`. They check which fields of a message count as measured, and that the header stamp keeps every nanosecond.
- **[`TestTopicChecks` and `TestSampleChecks`](#the-checks-of-the-input), 8 + 10 tests:** they call `check_topic()`, the six checks of the samples and `check_samples()`. They check which messages the tool refuses, and the reason that it gives.
- **[`TestAxisStatistics`](#the-statistics-of-one-axis), 8 tests:** they call `axis_statistics()`. They check the mean, the sample variance, the standard deviation and the distinct values of one axis.
- **[`TestVectorStatistics` and `TestFieldStatistics`](#the-statistics-of-one-axis), 1 + 2 tests:** they call `vector_statistics()` and `field_statistics()`. They check that each axis has its own statistics, and that a field needs two readings.
- **[`TestStampStatistics`](#the-header-stamps), 9 tests:** they call `stamp_statistics()`. They check the duration, the rate and the intervals of the header stamps, and the stamps that don't advance.
- **[`TestImuStatistics`](#the-statistics-of-a-recording), 5 tests:** they call `imu_statistics()`. They check that each statistic comes from the right field, and gravity.
- **[`TestFormatDuration`, `TestFormatTable` and `TestFormatReport`](#the-reports-text), 2 + 1 + 4 tests:** they call `format_duration()`, `format_table()` and `format_report()`. They check the text of the report, line by line.
- **[`TestToolOnBags`](#the-tool-as-a-command), 23 tests:** they run the script as a command. They check the report and status 0 for a good bag, and one line and status 1 for an input that the tool refuses.

Every test method has a name that says what it checks and a first docstring line that says the same as a sentence, for example `test_stamp_statistics_counts_no_long_interval_at_exactly_long_interval` with "Check that an interval of exactly LONG_INTERVAL isn't long: only a longer one is."

## Which readings count

A `sensor_msgs/Imu` message has a place for the angular velocity and one for the acceleration, also when the IMU measured only one of them. The message marks a field that it didn't measure with −1 as the first element of the field's covariance. The tool's function `measured_vector()` reads this mark. It gets a field's vector and the field's covariance, and returns the vector as a reading, or `None` for a field that wasn't measured. The excerpt shows the function, [lines 376–384 of `imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L376-L384):

```python title="imu_noise_at_rest.py: a field's reading, or None" linenums="376"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/scripts/imu_noise_at_rest.py:376:384"
```

The figure shows three messages as the test `test_imu_statistics_takes_a_field_only_from_the_samples_that_measured_it` builds them. The second message has no angular velocity:

--8<-- "includes/imu-noise-test-measured.html"

The tests of this part:

- **`test_measured_vector_gives_none_when_the_covariance_starts_with_minus_one`:** the reading (0.1, 0.2, 0.3) rad/s with a covariance that starts with −1 gives `None`.
- **`test_measured_vector_gives_the_reading_for_a_covariance_of_zeros`:** the same reading with a covariance of nine zeros, which means "not known", gives the reading.
- **`test_to_sample_gives_each_field_by_its_own_covariance`:** a message whose `angular_velocity_covariance` starts with −1 and whose acceleration is 9.81 m/s² on z gives a sample without the angular velocity and with the acceleration.
- **`test_to_sample_keeps_one_nanosecond_of_the_header_stamp`:** a message stamped 1 ns after `EPOCH`, 1,700,000,000 s, gives a sample whose stamp is `EPOCH + 1` ns, as an integer.
- **`test_imu_statistics_takes_a_field_only_from_the_samples_that_measured_it`:** the three messages of the figure give two readings of the angular velocity with the mean 0.3 rad/s, and three readings of the acceleration.

The stamp is an integer for a reason. A stamp of today is about 1.7 · 10⁹ s. A floating-point number of that size has steps of about 240 ns, so two stamps in seconds as floats can't tell an interval of 30 ms from one of 30 ms and 1 ns. As integers in nanoseconds, the stamps and the intervals between them are exact.

## The checks of the input

Before the tool calculates anything, it checks its input. Each check is one function that raises a `MessageError` with the reason.

The first check, `check_topic()`, runs before the tool reads a message. A bag stores three things about the message type of each of a topic's connections: the type's name, the text of the type's definition, and a checksum of the definition that the recorder got from the publisher. The check asks the bag for all three and refuses three kinds of topics:

- **Another type:** a topic whose type isn't named `sensor_msgs/Imu`. A camera's topic chosen by mistake is refused without loading its images.
- **A checksum that doesn't fit:** rosbag builds a message's class from the definition's text in the bag. The check builds the class in the same way. If the class's checksum isn't the stored one, the stored checksum and the stored definition disagree, and the bag doesn't say which of the two belongs to the messages. rosbag would try to decode the messages' bytes as the fields of the stored definition, whether the definition is theirs or not. So the tool refuses the topic before it reads a message.
- **Other fields:** messages that are named `sensor_msgs/Imu`, but whose class has other fields than ROS's. The check compares the class's fields, and the fields of its fields, with those of ROS's `sensor_msgs/Imu`.

`TestTopicChecks` gives the check stand-ins for a bag, which only know a topic's connections:

- **`test_check_topic_passes_a_topic_of_ros_imu_messages`:** a topic with ROS's definition of `sensor_msgs/Imu` raises nothing.
- **`test_check_topic_passes_a_definition_that_differs_in_its_comments`:** the same definition with one more comment line raises nothing either.
- **`test_check_topic_fails_for_a_topic_with_several_types_and_names_them_all`:** a topic with `sensor_msgs/Imu`, `std_msgs/String` and `std_msgs/UInt32` is refused with the reason `the topic holds sensor_msgs/Imu, std_msgs/String, std_msgs/UInt32. Every message of the topic must be a sensor_msgs/Imu`. The reason names all three, because `rosbag info` shows only one type for such a topic.
- **`test_check_topic_fails_for_a_checksum_that_is_not_the_definitions`:** ROS's definition, stored with the checksum of `std_msgs/String`, is refused with a reason that has both checksums.
- **`test_check_topic_fails_for_an_imu_definition_with_an_array_in_place_of_a_vector`:** a definition whose `angular_velocity` is an array of one vector is refused with the reason `angular_velocity is a geometry_msgs/Vector3[1] instead of a geometry_msgs/Vector3`. ROS gives this definition the same checksum as its own, so only the comparison of the fields finds the difference.
- **`test_check_topic_fails_for_a_field_of_a_field_and_names_its_path`:** a definition whose quaternion has the field `reading_w` instead of `w` is refused with the reason `orientation has the fields x, y, z, reading_w instead of x, y, z, w`.
- **`test_check_topic_fails_for_an_imu_definition_that_is_no_definition`:** a text that isn't a message definition is refused with the reason of genpy, the ROS library that reads definitions.
- **`test_check_topic_fails_with_the_readers_reason_when_the_reader_fails`:** a bag whose reader fails, as rosbag's reader of an old bag format does, is refused with the reader's error as the reason.

The other six checks look at the samples, and `check_samples()` runs them in the order of the list `SAMPLE_CHECKS`. The excerpt shows the list and the function, [lines 544–559 of `imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L544-L559):

```python title="imu_noise_at_rest.py: the checks of the samples and their runner" linenums="544"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/scripts/imu_noise_at_rest.py:544:559"
```

`TestSampleChecks` gives each check samples that it has to refuse, and compares the reason's text:

| Check | What goes in | The reason that is checked |
|:--|:--|:--|
| `check_message_count()` | One sample | `1 message, the statistics need at least two` |
| `check_named_frame()` | Two samples with an empty `frame_id` | `2 of 2 messages have an empty frame_id. The readings need one named frame, which says what their axes are` |
| `check_one_frame()` | A sample in `other_imu` and one in `imu_link` | `the messages have more than one frame_id ("imu_link", "other_imu"), so their readings have different axes` |
| `check_finite_readings()` | A sample whose yaw rate is NaN, stamped 20 ms after `EPOCH` | `the message stamped 1700000000.020000000 has the angular_velocity (0.0, 0.0, nan), which isn't finite` |
| `check_finite_readings()` | A sample whose x acceleration is infinite | A `MessageError` |
| `check_finite_readings()` | A sample without the angular velocity | Nothing is raised: a field that wasn't measured isn't looked at |
| `check_reading_size()` | A sample whose z acceleration is 2,000,000 m/s², twice `LARGEST_READING` | `the message stamped 1700000000.000000000 has the linear_acceleration (0.0, 0.0, 2000000.0). The tool takes a value beyond 1e+06 m/s^2 as damaged data` |
| `check_reading_size()` | A sample whose x rate is −1,000,000 rad/s, exactly `LARGEST_READING` | Nothing is raised |
| `check_reading_count()` | One sample without the angular velocity and one without the acceleration | `angular_velocity has 1 reading and linear_acceleration has 1 reading. At least one of them needs two` |
| `check_samples()` | Two samples at rest in `imu_link` | Nothing is raised |

The frame matters because it says which way the axes x, y and z point. Readings in two frames, or in a frame without a name, don't belong on one axis. The mean of +9.81 and −9.81 m/s² would be a gravity of zero.

`LARGEST_READING` is 1,000,000, in rad/s or m/s². It is the tool's own limit for a plausible reading of a robot at rest. The tool takes a value beyond it as damaged data, for example one changed byte in a number. One such value alone would decide the mean and the variance of its axis.

## The statistics of one axis

`axis_statistics()` gets the readings of one axis and returns their mean, their sample variance and the number of distinct values. It takes the variance from `sample_variance()`, which calculates with fractions instead of floating-point numbers. [The mean and the variance of three readings](driver-bno055.md#the-mean-and-the-variance-of-three-readings) on the BNO055 page explains the formulas. The test of the sample variance uses the four values 1, 2, 3 and 4:

--8<-- "includes/imu-noise-test-variance.html"

The excerpt shows this test, [lines 146–153 of `test_imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L146-L153). `assertAlmostEqual()` compares the two numbers to seven decimal places:

```python title="test_imu_noise_at_rest.py: the sample variance divides by the number of values minus one" linenums="146"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/test/test_imu_noise_at_rest.py:146:153"
```

The other tests of the statistics:

- **`test_axis_statistics_gives_the_mean`:** 1, 2, 3 and 6 have the mean 3.
- **`test_axis_statistics_gives_the_standard_deviation_as_the_root_of_the_variance`:** the standard deviation of 1, 2, 3 and 4 is the square root of 5/3.
- **`test_axis_statistics_counts_each_distinct_value_once`:** five readings on three output steps of 1/900 rad/s are three distinct values.
- **`test_axis_statistics_gives_a_variance_of_zero_without_noise`:** five equal readings have a variance of exactly 0.
- **`test_axis_statistics_gives_the_exact_variance_of_two_neighbouring_floats`:** 9.81 and the next floating-point number above it have the variance $(b - a)^2 / 2$, compared exactly. Their mean lies between two floating-point numbers. Rounded to one of them, the mean would give one deviation of zero and one of a whole step, and twice the variance. Fractions hold the mean exactly.
- **`test_axis_statistics_gives_a_variance_of_zero_for_deviations_too_small_to_square`:** the readings 7 · 10⁻³⁰⁴, 0 and 0 have a variance of 0, without an error. The squares of deviations that small are zero as floating-point numbers, and Python 3.8's `statistics.variance()` stops with an error on them. As fractions, the squares aren't zero, and only the result is rounded to a floating-point number, to 0.
- **`test_axis_statistics_fails_with_fewer_than_two_values`:** one value raises a `StatisticsError`, because a sample variance needs two values.
- **`test_vector_statistics_keeps_the_axes_apart`:** the vectors (1, 10, 100) and (3, 30, 300) have the means 2, 20 and 200 on x, y and z.
- **`test_field_statistics_gives_the_reading_count_and_the_axes_statistics`:** two readings give a count of 2 and the statistics of each axis.
- **`test_field_statistics_gives_no_axes_statistics_for_one_reading`:** one reading gives a count of 1 and no statistics, instead of an error.

## The header stamps

`stamp_statistics()` gets the header stamps in the order in which the bag stored the messages, as integers in nanoseconds. It returns the time from the first stamp to the last, the rate, the shortest and the longest interval between two consecutive stamps, and three counts: the intervals longer than `LONG_INTERVAL`, the stamps that repeat the stamp before, and the stamps that are earlier than the stamp before. The excerpt shows the function together with `mean_rate()`, [lines 612–640 of `imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/scripts/imu_noise_at_rest.py#L612-L640):

```python title="imu_noise_at_rest.py: the rate and the statistics of the header stamps" linenums="612"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/scripts/imu_noise_at_rest.py:612:640"
```

### Stamps that advance

The first two tests use five stamps that advance, with one long interval:

--8<-- "includes/imu-noise-test-intervals.html"

- **`test_stamp_statistics_gives_the_duration_and_the_mean_rate`:** the duration is 95 ms and the rate 4 / 0.095 s. Five messages span four intervals, so the rate counts the intervals and not the messages.
- **`test_stamp_statistics_gives_the_shortest_and_the_longest_interval`:** 20 ms and 35 ms, compared as integers, so to the nanosecond.
- **`test_stamp_statistics_counts_no_repeated_or_earlier_stamp_in_increasing_stamps`:** both counts are 0.

### The limit of a long interval

`LONG_INTERVAL` is 30 ms: one and a half periods of the 20 ms at which the firmware takes its samples. Three tests put two stamps around this limit, at the size of a real stamp:

- **29,999,999 ns, 1 ns below the limit:** no long interval (`test_stamp_statistics_counts_no_long_interval_one_nanosecond_below_long_interval`).
- **30,000,000 ns, exactly the limit:** no long interval (`test_stamp_statistics_counts_no_long_interval_at_exactly_long_interval`).
- **30,000,001 ns, 1 ns above the limit:** one long interval (`test_stamp_statistics_counts_a_long_interval_one_nanosecond_above_long_interval`).

### Stamps that don't advance

A clock that is set back during a recording gives stamps that go back. The fixture has one repeated stamp and one earlier stamp:

--8<-- "includes/imu-noise-test-stamps-back.html"

- **`test_stamp_statistics_counts_a_repeated_and_an_earlier_stamp_apart`:** one repeated stamp, one earlier stamp, and a shortest interval of −10 ms.
- **`test_stamp_statistics_gives_no_rate_when_a_stamp_inside_the_recording_goes_back`:** the duration is 80 ms and the rate is `None`. Without this rule, the four intervals in 80 ms would read as a healthy 50 Hz.
- **`test_stamp_statistics_gives_no_rate_when_no_stamp_advances`:** three stamps of 0, a clock that was never set, give no rate and two repeated stamps.

## The statistics of a recording

`imu_statistics()` puts the parts together: the stamps' statistics, the frame, the statistics of the two fields and gravity. Its tests check that each number comes from the right place:

- **`test_imu_statistics_gives_the_stamps_statistics_and_the_frame`:** two samples 20 ms apart in `imu_link` give a duration of 20 ms and the frame `imu_link`.
- **`test_imu_statistics_keeps_the_angular_velocity_and_the_acceleration_apart`:** two samples with different yaw rates and z accelerations give the mean 0.4 rad/s for the yaw rates and 10 m/s² for the z accelerations.
- **`test_imu_statistics_gives_gravity_as_the_length_of_the_mean_acceleration`:** the accelerations (1, 3, 5) and (3, 3, 7) m/s² give a gravity of 7 m/s².
- **`test_imu_statistics_gives_no_gravity_without_the_acceleration`:** two samples without the acceleration give `None` for gravity, not 0.

The test of gravity uses a tilted IMU. The mean of its two accelerations is (2, 3, 6) m/s², and the length of that vector is a whole number:

--8<-- "includes/imu-noise-test-gravity.html"

The excerpt shows the test, [lines 515–520 of `test_imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L515-L520). `sample()` is the test file's helper for a sample of a level robot at rest, and `known()` fails the test if gravity is `None`:

```python title="test_imu_noise_at_rest.py: gravity is the length of the mean acceleration" linenums="515"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/test/test_imu_noise_at_rest.py:515:520"
```

## The report's text

The functions whose names start with `format_` turn the statistics into the report's lines. Their tests compare text:

- **`TestFormatDuration`:** 179.78 s reads `179.8 s`, and 40 ms reads `40.0 ms`. A duration below one second is given in milliseconds, so that a short recording doesn't read `0.0 s`.
- **`TestFormatTable`:** each column of a table is as wide as its widest cell, with the first column aligned left and the others right.
- **`TestFormatReport`:** one test compares all 18 lines of a report with both fields' statistics. One compares the stamps' lines of a recording with a repeated and an earlier stamp, which read "no rate (some stamps don't advance)". One compares the two lines that a field without statistics gets instead of its table, and checks that the report then has no line about gravity. One checks that a `frame_id` with a line break in it stays on the report's first line, with the line break shown as `\n`.

## The tool as a command

The 23 tests of `TestToolOnBags` run the script the way a user does. Each test prepares an input in a temporary folder: a bag, a file that isn't a bag, or no file at all. It starts the script as a new process and compares what the process prints.

`write_bag()` stores the messages in the order of its list, each with the time at which a recorder would have received it. The excerpt shows the function, [lines 668–677 of `test_imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L668-L677):

```python title="test_imu_noise_at_rest.py: write the messages to a bag" linenums="668"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/test/test_imu_noise_at_rest.py:668:677"
```

`run_tool()` starts the script and returns the process's exit status and what it wrote to its standard output and its standard error. The three helpers of the class turn that into assertions. `report_lines()` expects status 0 with nothing on the standard error. `assert_tool_ends_with()` expects status 1, nothing on the standard output and exactly one given line on the standard error. `assert_tool_ends_with_a_reason_after()` expects the same, for a line of which the test fixes only the start. The excerpt shows `run_tool()`, the class's `setUp()` and the three helpers, [lines 717–754 of `test_imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L717-L754):

```python title="test_imu_noise_at_rest.py: run the tool and compare what it prints" linenums="717"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/test/test_imu_noise_at_rest.py:717:754"
```

### The whole report for three messages

`test_tool_prints_the_report_of_three_readings_one_output_step_apart` writes three messages 20 ms apart, in the frame `imu_link`, with gravity on z. Their yaw rates are 0, 1 and 2 output steps of 1/900 rad/s:

--8<-- "includes/imu-noise-three-readings.html"

The mean is one step, 0.00111 rad/s, and the sample variance one step squared, 1.235 · 10⁻⁶ (rad/s)². The test compares all 18 lines of the report, as the excerpt shows, [lines 756–782 of `test_imu_noise_at_rest.py`]({{ imu_noise_src_url }}/diffbot_bringup/test/test_imu_noise_at_rest.py#L756-L782):

```python title="test_imu_noise_at_rest.py: every line of the report for three readings" linenums="756"
--8<-- "{{ imu_noise_src_raw }}/diffbot_bringup/test/test_imu_noise_at_rest.py:756:782"
```

The messages show both meanings of a covariance. Their `angular_velocity_covariance` and `linear_acceleration_covariance` are all zeros, which means "not known", and the report has these fields' statistics. Their `orientation_covariance` starts with −1, as the firmware's does, which means that the messages have no orientation.

### Reports for other bags

Each of these tests changes the three messages in one way and compares the lines that change:

- **`test_tool_reads_another_topic_with_the_topic_option`:** the three messages are on the topic `/imu/data` and are read with `--topic /imu/data`. The first line starts with `/imu/data: 3 messages`.
- **`test_tool_shows_a_line_break_in_the_topic_on_the_first_line`:** the topic's name has a line break in it. The report's first line shows the line break as `\n`, and the report still has its 18 lines.
- **`test_tool_gives_the_reason_instead_of_statistics_for_a_field_not_measured`:** every message marks its angular velocity as not measured. The report says `angular_velocity: 0 readings, the statistics need at least two` and has the acceleration's table as before.
- **`test_tool_takes_the_stamps_from_the_headers_not_from_the_arrival_times`:** the third message reaches the recorder 500 ms late. The stamps' lines are as before, with 50.00 Hz and no long interval, because the header stamps can't show a late delivery.
- **`test_tool_gives_no_rate_and_keeps_the_statistics_when_a_stamp_goes_back`:** the second message is stamped 10 ms before the first. The report says `no rate (some stamps don't advance)` and `1 earlier than the one before`, and has the yaw rate's row as before.
- **`test_tool_gives_no_rate_when_the_recorder_clock_goes_back_with_the_stamps`:** the same stamps, and the recorder's clock goes back with them. The report is the same.

The last two tests differ in the recorder's clock. rosbag's reader returns the messages sorted by the time at which the recorder received each one. If that clock is set back together with the IMU's stamps, for example because the robot's computer corrects its time, the sorted stamps advance again and the step back would be invisible. So the tool puts the messages back into the order in which the bag stored them, and the report is the same in both cases.

### Inputs that the tool refuses

For these inputs the tool prints one line and ends with status 1. The line starts with the file's name. For a problem with the messages, the topic follows. All tests but the last two of the list compare the whole line. The list gives the line's reason:

- **`test_tool_ends_with_a_message_when_the_messages_have_two_frames`:** the second message is in the frame `other_imu`, with gravity on −z. The reason is `the messages have more than one frame_id ("imu_link", "other_imu"), so their readings have different axes`.
- **`test_tool_ends_with_a_message_when_the_messages_have_no_frame_id`:** all three messages have an empty `frame_id`. The reason is `3 of 3 messages have an empty frame_id. The readings need one named frame, which says what their axes are`.
- **`test_tool_ends_with_a_message_when_a_reading_is_not_a_number`:** the second yaw rate is NaN. The reason is `the message stamped 1700000000.020000000 has the angular_velocity (0.0, 0.0, nan), which isn't finite`.
- **`test_tool_ends_with_a_message_when_the_topic_has_another_type`:** the topic holds two `std_msgs/String` messages. The reason is `the topic holds std_msgs/String. Every message of the topic must be a sensor_msgs/Imu`.
- **`test_tool_ends_with_a_message_when_the_imu_messages_have_other_fields`:** the bag names the topic's type `sensor_msgs/Imu` and stores the definition of `std_msgs/String` for it. The reason starts with `the bag defines the topic's sensor_msgs/Imu messages with other fields than ROS's: the message has the fields data instead of header, orientation`.
- **`test_tool_ends_with_a_message_when_the_checksum_is_not_the_definitions`:** the bag stores ROS's definition for the topic under the checksum of `std_msgs/String`. The reason is `the bag's checksum of the topic's sensor_msgs/Imu messages is`, the stored checksum, `but the definition that the bag stores for them has the checksum` and ROS's checksum.
- **`test_tool_shows_a_line_break_in_a_checksum_on_its_one_line`:** the stored checksum is ROS's, followed by a line break. The tool's line shows the line break as `\n`, so the line stays one line.
- **`test_tool_ends_with_a_message_when_the_angular_velocity_is_an_array_of_one_vector`:** the messages' `angular_velocity` is an array of one vector. The reason is `the bag defines the topic's sensor_msgs/Imu messages with other fields than ROS's: angular_velocity is a geometry_msgs/Vector3[1] instead of a geometry_msgs/Vector3`.
- **`test_tool_ends_with_a_message_when_the_topic_has_one_message`:** the reason is `1 message, the statistics need at least two`.
- **`test_tool_ends_with_a_message_when_the_topic_is_not_in_the_bag`:** the three messages are read with `--topic /other`. The reason is `0 messages, the statistics need at least two`.
- **`test_tool_ends_with_a_message_when_the_file_is_missing`:** a file name without a file. The reason is `can't be opened: No such file or directory`.
- **`test_tool_ends_with_a_message_when_the_file_is_not_a_rosbag`:** a text file. The reason is `isn't a rosbag: a bag file starts with the text #ROSBAG`.
- **`test_tool_ends_with_a_message_when_the_file_is_a_compressed_copy_of_a_bag`:** a bag packed with gzip. The reason is the same.
- **`test_tool_ends_with_a_message_when_the_file_ends_after_the_first_line`:** a file that holds only a bag's first line, like a recording that was cut off before the recorder wrote the index. The reason is `has no index, like a recording that was cut off. "rosbag reindex" can repair such a file`.
- **`test_tool_ends_with_a_message_when_the_messages_data_is_damaged`:** a bag with compressed messages, of which the test inverts one byte. The bag still opens, because its index is intact, and reading its messages fails. The line starts with the file, the topic and `a message can't be read from the bag:`.
- **`test_tool_ends_with_a_message_when_a_message_lacks_its_last_byte`:** the third message's data is one byte too short for a `sensor_msgs/Imu`. The line starts as in the test before.

In the last two tests, the reason at the line's end comes from rosbag or from Python. These tests don't fix its words. They compare the line's start and check that a reason follows it, and that the line is the only one.

## What the tests don't show

- **No real IMU:** the tests' messages are made up. That the BNO055's readings on Remo have the bias and the noise of the [results](driver-bno055.md#results-on-remo) is a measurement, not a test.
- **No recorder:** the tests write their bags with rosbag's Python interface. They don't run `rosbag record`, and they don't cover a recording that was cut off.
- **No long recording:** the largest test bag has three messages. A recording of 180 s has about 9,000.
- **Nothing about driving:** the tool describes a robot at rest. How much noise driving adds is a separate check.
