# Test: The EKF on a Stepped Clock

What the stepped-clock test checks on the EKF and the IMU sample validator, and how the test does it. This page belongs to the [tests of the Localization](index.md), which list the five tests by the question that each answers.

[`test_ekf_synthetic.py`]({{ diffbot_repo_url }}/diffbot_control/test/test_ekf_synthetic.py) runs the two nodes of [`ekf.launch`]({{ diffbot_repo_url }}/diffbot_control/launch/ekf.launch), the [IMU sample validator](../../robot_localization.md#the-imu-sample-validator) and robot_localization's EKF, with the parameters that the robot and the simulation use. The test computes their inputs itself: wheel odometry and IMU samples that report a constant yaw rate $\omega_z$. The test also owns the clock. It publishes the topic `/clock` and moves the ROS time $t$ on in steps of 20 ms. So the test decides which messages are published, with which stamps and at which ROS time, without Gazebo and without a robot.

## One step of the clock

The ROS time of the nodes is the time of the last message on the topic `/clock`. So the ROS time stands still between two of the test's clock messages, while the computer's own clock, the wall clock, runs on. The test can wait on the wall clock as long as a step needs, and no ROS time passes for the nodes. A step at the start of a scenario, at the ROS time $t$ = 100.00 s, the test's constant `START`, runs in five parts:

1. **The inputs:** the test publishes a wheel message and an IMU sample, both stamped 100.00 s.
2. **The validator:** the test waits on the wall clock until the validator's output, the forwarded IMU sample, has reached the test. The ROS time stays 100.00 s.
3. **The delivery:** the test sleeps `DELIVERY`, 20 ms of wall time, so that the inputs reach the EKF too. The ROS time still stays 100.00 s.
4. **The clock:** the test publishes the ROS time 100.02 s, which is `STEP`, 20 ms, later. The EKF updates 50 times per second of ROS time, so the new clock lets the EKF run its update at 100.02 s, which fuses the two inputs.
5. **The output:** the EKF publishes its output at the ROS time 100.02 s, stamped 100.00 s, the time of the inputs that it fused. The test waits on the wall clock until an output arrives that is stamped from the step's 100.00 s up to the clock's 100.02 s.

The figure shows the messages of this step. Read the figure from top to bottom. The three boxes are the test, the validator and the EKF. Each arrow is a message, from the node that publishes it to the node that receives it. The label above an arrow names the message, and the note below the arrow says how the message is stamped or which clock runs. $t$ is the ROS time of the step. The numbers are those of the list in the docstring of [`step()`](#the-code-of-one-step), and the key below the drawing names the code of each number.

--8<-- "includes/localization-test-stepped-clock-step.html"

The file holds two scenarios. A scenario is one test that runs its phases in order on one filter, from the filter's start:

- **[`invalid_imu_samples`](#the-seven-phases-of-invalid_imu_samples):** both inputs arrive in every step, 50 times per second of ROS time, as in the simulation. A pass proves four things about the filter's estimate of the yaw rate $\omega_z$. The estimate follows the IMU while valid samples arrive. The estimate goes to the wheels' yaw rate when no valid sample arrives. Samples that the validator has to drop don't reach the estimate. And the estimate follows the IMU again after the clock went back. A pass also proves that the validator's counts on the topic `/diagnostics` match the samples that the test sent.
- **[`robot_rates`](#the-robots-rates-robot_rates):** the inputs arrive at the robot's nominal rates, the IMU in every step and the wheel odometry in every fifth step, 10 times per second. A pass proves three things at these rates. The estimate follows the IMU's yaw rate. The estimate's forward speed $v_x$ follows the wheels', also when the wheels' speed changes, so the filter fuses the wheel messages. And after the IMU stops, each 100 ms wheel period contains the header stamp of one output, the time that the filter writes into the output, with an estimate that goes to the wheels' yaw rate.

[What the test doesn't show](#what-the-test-doesnt-show) lists what a pass doesn't prove.

## What the test files start

Each scenario has a `.test` file of its own, a launch file that rostest reads. rostest starts a ROS master and the nodes that the file declares, and runs the test node. So each scenario starts with a fresh EKF and a fresh validator, and no scenario sees what another one left in the filter.

Both `.test` files include the same nodes from [`ekf_synthetic.launch`]({{ diffbot_repo_url }}/diffbot_control/test/ekf_synthetic.launch). The collapsed box shows this launch file:

??? example "The nodes of both scenarios: `ekf_synthetic.launch`, lines 1–12"

    The excerpt shows the whole file, from [`ekf_synthetic.launch`, lines 1–12]({{ ekf_test_src_url }}/diffbot_control/test/ekf_synthetic.launch#L1-L12):

    ```xml title="ekf_synthetic.launch" linenums="1"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/ekf_synthetic.launch:1:12"
    ```

- **`use_sim_time`:** every node takes the ROS time from the topic `/clock`. No simulator runs here, so the only publisher of that topic is the test.
- **`diffbot/robot_description`:** DiffBot's URDF model, which xacro builds from `diffbot.urdf.xacro`.
- **`robot_state_publisher`:** publishes the fixed joints of this URDF model, the joint `imu_joint` among them. From them, TF knows where the IMU's frame `imu_link` sits on `base_footprint`, as on the robot. The EKF needs this transform to use the IMU's samples.
- **`ekf.launch`:** starts the nodes `/diffbot/imu_validator` and `/diffbot/ekf_localization` with the parameters of the robot: the validator's `max_age` and `max_future`, and [`ekf.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/ekf.yaml) for the filter.

The first `.test` file runs the scenario [`invalid_imu_samples`](#the-seven-phases-of-invalid_imu_samples), the second the scenario [`robot_rates`](#the-robots-rates-robot_rates). The collapsed box shows both files:

??? example "The two `.test` files, lines 1–9 each"

    The first excerpt shows the whole file of `invalid_imu_samples`, from [`ekf_synthetic.test`, lines 1–9]({{ ekf_test_src_url }}/diffbot_control/test/ekf_synthetic.test#L1-L9):

    ```xml title="ekf_synthetic.test" linenums="1"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/ekf_synthetic.test:1:9"
    ```

    The second excerpt shows the whole file of `robot_rates`, from [`ekf_synthetic_robot_rates.test`, lines 1–9]({{ ekf_test_src_url }}/diffbot_control/test/ekf_synthetic_robot_rates.test#L1-L9):

    ```xml title="ekf_synthetic_robot_rates.test" linenums="1"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/ekf_synthetic_robot_rates.test:1:9"
    ```

- **The `<include>` tag:** starts the nodes of `ekf_synthetic.launch`.
- **The `<test>` tag:** runs `test_ekf_synthetic.py` as the test node. `args` gives the node the argument `--scenario` with the scenario's name. `time-limit` gives the test 120 s of wall time.

The test node picks the scenario's class by this argument, as the collapsed box shows:

??? example "How the test node picks its scenario: lines 801–822"

    The excerpt shows the table of the scenarios and the start of the node, from [`test_ekf_synthetic.py`, lines 801–822]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L801-L822):

    ```python title="test_ekf_synthetic.py: SCENARIOS, scenario_from_arguments() and the start" linenums="801"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:801:822"
    ```

    1. `rospy.myargv()` returns the node's arguments without ROS's remapping arguments, such as the node's name.
    2. `scenario_from_arguments()` reads `--scenario` with Python's [`argparse`](https://docs.python.org/3/library/argparse.html). `parse_known_args()` leaves the other arguments alone, such as the path of the result file that rostest adds. The dictionary `SCENARIOS` maps the name to the scenario's class.
    3. `rospy.init_node()` starts the test node, and `rostest.rosrun()` runs the test method of that class and writes its result.

`diffbot_control` registers both files with [`add_rostest()`](https://github.com/ros/catkin/blob/noetic-devel/doc/howto/format2/rostest_configuration.rst) in its [`CMakeLists.txt`]({{ diffbot_repo_url }}/diffbot_control/CMakeLists.txt), so CI runs them with the workspace's other tests, as [Testing and CI](../../../../development/ci.md#ekf-tests) describes. To run only one scenario, open a shell in which the workspace is sourced:

```console
rostest diffbot_control ekf_synthetic.test
rostest diffbot_control ekf_synthetic_robot_rates.test
```

## The inputs

The test publishes on the two topics that the nodes read on the robot. The wheel odometry goes to the EKF on the topic `/diffbot/mobile_base_controller/odom`. The IMU samples go to the validator on the topic `/diffbot/imu/data_raw`. The test subscribes to the validator's output, the topic `/diffbot/imu/data_validated`, to the filter's output, the topic `/diffbot/odometry/filtered`, and to the topic `/diagnostics`.

??? example "The constants of `test_ekf_synthetic.py`, lines 49–132"

    The excerpt shows the timing, topic and test constants of the file, from [`test_ekf_synthetic.py`, lines 49–132]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L49-L132). `START` and `RESTART` are readings of the ROS clock, which the test sets. `STEP`, `ROBOT_WHEEL_ODOMETRY_PERIOD` and `DIAGNOSTICS_WAIT` are durations of ROS time. `WAIT`, `DELIVERY`, `POLL_INTERVAL`, `NO_OUTPUT_WAIT`, `TF_DELIVERY` and `RESET_DELIVERY` are seconds of wall time. `STEPS_PER_WHEEL_MESSAGE` is the number of steps from one wheel message to the next in the scenario [`robot_rates`](#the-robots-rates-robot_rates), 100 ms divided by 20 ms. `SPEED_CHANGE_STEP`, `CHANGED_SPEED` and `SPEED_CHANGE_WINDOW` belong to [phase 1 of `robot_rates`](#phase-1-of-robot_rates-the-estimate-follows-the-imu-and-the-wheels-speed).

    ```python title="test_ekf_synthetic.py: the constants" linenums="49"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:49:132"
    ```

What the wheels report in a wheel message is one value of the type `WheelInput`. The excerpt shows the type and its two values, from [`test_ekf_synthetic.py`, lines 149–158]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L149-L158):

```python title="test_ekf_synthetic.py: WheelInput, WHEEL_INPUT and CHANGED_SPEED_WHEEL_INPUT" linenums="149"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:149:158"
```

- **`speed`:** the forward speed $v_x$ that the wheels report.
- **`yaw_rate`:** the yaw rate $\omega_z$ that the wheels report.
- **`WHEEL_INPUT`:** `SPEED`, 0.1 m/s, and `WHEEL_RATE`, 0.4 rad/s. Every wheel message of the test reports these two values, apart from the second part of phase 1 of `robot_rates`.
- **`CHANGED_SPEED_WHEEL_INPUT`:** `CHANGED_SPEED`, 0.12 m/s, and `WHEEL_RATE`. Phase 1 of `robot_rates` sends this value from its 21st wheel message on.

Two functions build the messages. The excerpt shows both, from [`test_ekf_synthetic.py`, lines 209–238]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L209-L238):

```python title="test_ekf_synthetic.py: wheel_odometry() and imu_sample()" linenums="209"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:209:238"
```

- **`wheel_odometry()`:** a `nav_msgs/Odometry` message as the drive controller publishes it, in the frame `odom` with the child frame `base_footprint`. The forward speed $v_x$ and the yaw rate $\omega_z$ are those of the argument `wheel_input`. The variances $\sigma^2$ on the diagonals of both covariances are `WHEEL_VARIANCES`, with 0.03 (rad/s)² for the yaw rate. These are the numbers of [`diffbot_control.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/diffbot_control.yaml), which the test writes out and doesn't read from the file.
- **`imu_sample()`:** a `sensor_msgs/Imu` message as the firmware publishes it, in the frame `imu_link`. The orientation is marked unavailable, the yaw rate $\omega_z$ is the function's argument, and the upward acceleration $f_z$ is `GRAVITY`. The variance of the angular velocity is `IMU_ANGULAR_VELOCITY_VARIANCE`, 4 · 10⁻⁴ (rad/s)², the firmware's number, written out in the test too.
- **No noise:** the test sets every value of both messages itself, the same in every step of a phase apart from the speed change of `robot_rates`, and without noise. So what the test sees in the estimate comes from the filter and the validator, not from chance.

A step can publish several IMU samples. The class `ImuInput` describes one of them. The excerpt shows the class and the sample that most steps use, from [`test_ekf_synthetic.py`, lines 137–146]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L137-L146):

```python title="test_ekf_synthetic.py: ImuInput and VALID_SAMPLE" linenums="137"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:137:146"
```

- **`offset`:** the sample's stamp $t_s$ relative to the step's ROS time $t$, in seconds.
- **`yaw_rate`:** the yaw rate $\omega_z$ that the sample reports.
- **`drop_reason`:** `None` for a sample that the validator has to forward. For a sample that the validator has to drop, the reason as the validator's diagnostics name it, one of the texts of [`DropReason`](validator.md#dropreason-the-reasons-to-drop-a-sample).
- **`VALID_SAMPLE`:** stamped with the step's time, reporting `IMU_RATE`, 0.5 rad/s.

## Before the first step

The class `SteppedClockTestCase` holds what both scenarios share: the topics, the steps of the clock and the comparisons. Each scenario is a subclass with one test method. unittest calls `setUp()` before the test method. In short, `setUp()` sets the test's ROS time to 100 s, creates the topics' publishers and subscribers, publishes the first clock message, and waits until the validator and the EKF are connected and TF has the IMU's transform. The collapsed box shows the code:

??? example "`setUp()`, `on_validated()` and `all_connected()`: lines 266–305"

    The excerpt shows `setUp()` and its two helpers `on_validated()` and `all_connected()`, from [`test_ekf_synthetic.py`, lines 266–305]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L266-L305):

    ```python title="test_ekf_synthetic.py: setUp(), on_validated() and all_connected()" linenums="266"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:266:305"
    ```

    1. `setUp()` sets the test's own ROS time `self.now` to `START`, 100 s, and creates the three logs and the two counters that the phases fill. `self.forwarded_stamp_keys` holds the stamps of the samples that the validator forwarded, `self.ekf_outputs` the filter's outputs, and `self.diagnostics` the messages of the topic `/diagnostics`. `self.expected_forward_count` and `self.expected_drops` count what the validator has to do with each sample that the test sends.
    2. `setUp()` creates the three publishers and the three subscribers. `self.clock_publisher` is latched, so a node that connects later still gets the last clock message. Each subscriber's callback appends what arrives to one of the three logs. For the validator's output, the callback is `on_validated()`, which appends the sample's stamp.
    3. `setUp()` publishes the clock for the first time, at `START`.
    4. `setUp()` waits until the validator and the EKF are connected to the test's topics, with `all_connected()`, and until TF has the transform from `imu_link` to `base_footprint`. Then it sleeps `TF_DELIVERY`, 1 s of wall time, so that the EKF's own TF listener has the transform too.

The three logs are objects of the class `ArrivalLog`. rospy calls a subscriber's callback in a thread of its own, while the test's thread reads what arrived. A lock guards every access, so the test's thread never reads a log while a callback changes it. The collapsed box shows the class:

??? example "`ArrivalLog`: lines 164–194"

    The excerpt shows the class, from [`test_ekf_synthetic.py`, lines 164–194]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L164-L194):

    ```python title="test_ekf_synthetic.py: ArrivalLog" linenums="164"
    --8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:164:194"
    ```

    - **`append()`:** adds an item at the end. The subscribers of the filter's output and of the topic `/diagnostics` have it as their callback, and `on_validated()` calls it.
    - **`count()`:** returns the number of items so far. The test takes this number before a step.
    - **`since()`:** returns a copy of the items after the first `count` items. With the number from before a step, the test gets what arrived since the step started.
    - **`contains_all()`:** returns whether each of the given items has arrived.

The clock stands still during `setUp()`. A wait on the ROS time would never end, so the test waits on the wall clock with `wait_for()`. The excerpt shows `wait_for()` and the function `wait_until()` that it calls, from [`test_ekf_synthetic.py`, lines 241–254]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L241-L254):

```python title="test_ekf_synthetic.py: wait_until() and wait_for()" linenums="241"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:241:254"
```

- **`wait_until()`:** checks `condition()` every `POLL_INTERVAL`, 1 ms, and returns `True` as soon as the condition holds, or `False` after `WAIT`, 5 s.
- **`wait_for()`:** fails the test with the text `failure` when `wait_until()` returns `False`.

## The code of one step

Every phase is a number of steps, as [One step of the clock](#one-step-of-the-clock) describes them. 20 ms is the period of the IMU at 50 Hz. The excerpt shows `advance()`, `outputs_since()` and `step()`, from [`test_ekf_synthetic.py`, lines 343–384]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L343-L384). The docstring of `step()` lists the five things that one step does, and `advance()` does the first four:

```python title="test_ekf_synthetic.py: advance(), outputs_since() and step()" linenums="343"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:343:384"
```

The five things of one step, as the docstring of `step()` numbers them:

1. **Publish the inputs:** `publish_inputs()` publishes the step's wheel odometry, stamped with the step's ROS time, and its IMU samples, each stamped with that time plus the sample's `offset`. A step whose `wheel_input` is `None` publishes no wheel odometry. The scenario [`robot_rates`](#the-robots-rates-robot_rates) uses such steps.
2. **Wait for the validator:** `wait_for_forwarded()` waits on the wall clock until the validator's output for the step's valid samples reached the test. The EKF subscribes to the same topic.
3. **Wait `DELIVERY`:** `advance()` sleeps `DELIVERY`, 20 ms of wall time, so that the inputs reach the EKF before the clock moves.
4. **Advance the clock:** `advance_clock()` adds `STEP` to `self.now` and publishes the new clock. The EKF's update runs 50 times per second of ROS time, so the new clock lets the filter process the step's inputs.
5. **Wait for the EKF:** `step()` waits until the filter published an output that belongs to the step, at most `WAIT`, 5 s of wall time. `step()` returns the last such output.

`advance()` returns a `StepStart`, which says where the step started. The excerpt shows the class, from [`test_ekf_synthetic.py`, lines 197–201]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L197-L201):

```python title="test_ekf_synthetic.py: StepStart" linenums="197"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:197:201"
```

- **`stamp`:** the step's ROS time. `advance()` reads it from `self.now` before the clock moves.
- **`output_count_before`:** the number of EKF outputs that the test had before the step, from `self.ekf_outputs.count()`.
- **`outputs_since()`:** takes the `StepStart` and returns the EKF's outputs that arrived after the first `output_count_before` and are stamped from the step's time up to the ROS time that the test has published, `self.now`. So an output of an earlier step doesn't count for this step, with its earlier stamp. Neither does an output from before the clock went back that arrives late: its stamp lies after the new clock.

`advance()` calls three helpers, and `publish_inputs()` calls a fourth, `count_expected_outcome()`. The excerpt shows the four helpers, from [`test_ekf_synthetic.py`, lines 307–341]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L307-L341):

```python title="test_ekf_synthetic.py: publish_inputs(), count_expected_outcome(), wait_for_forwarded() and advance_clock()" linenums="307"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:307:341"
```

- **`publish_inputs()`:** publishes the wheel odometry for `wheel_input`, stamped with the step's time `stamp`, unless `wheel_input` is `None`. For each [`ImuInput`](#the-inputs) of the step, it publishes an IMU sample stamped `stamp` plus the sample's `offset`, and passes the `ImuInput` to `count_expected_outcome()`. It returns `valid_stamp_keys`, the stamps of the samples that the validator has to forward.
- **`count_expected_outcome()`:** counts what the validator has to do with the sample, the outcome that the test expects. A sample without a `drop_reason` adds one to `self.expected_forward_count`. A sample with a `drop_reason` adds one to that reason in `self.expected_drops`.
- **`wait_for_forwarded()`:** waits with [`wait_for()`](#before-the-first-step) until `self.forwarded_stamp_keys` contains every stamp of `stamp_keys`. `on_validated()` appends to this log the stamp of each sample that arrives on the topic `/diffbot/imu/data_validated`.
- **`advance_clock()`:** adds `STEP` to `self.now` and publishes the new time on the topic `/clock`.

Two more methods run steps. The excerpt shows `step_allowing_no_output()` and `run_steps()`, from [`test_ekf_synthetic.py`, lines 386–397]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L386-L397):

```python title="test_ekf_synthetic.py: step_allowing_no_output() and run_steps()" linenums="386"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:386:397"
```

- **`step_allowing_no_output()`:** is for steps after which the filter may publish nothing: [phase 6 of `invalid_imu_samples`](#phase-6-the-clock-goes-back) and [phase 2 of `robot_rates`](#phase-2-of-robot_rates-the-imu-stops). It does 1 to 4 with `advance()`, sleeps `NO_OUTPUT_WAIT`, 0.2 s of wall time, and returns the step's last output, or `None` if no output came.
- **`run_steps()`:** runs `count` steps with the same inputs and returns one output for each step.

The steps use `wait_for()` for the messages that they need, and `wait_for()` fails the test if a message is still missing after `WAIT`, 5 s of wall time. The wait after the measured periods of phase 2 returns what is missing instead, as [phase 2](#phase-2-of-robot_rates-the-imu-stops) explains. The waits reduce how much the result depends on the computer's speed. They don't prove that the EKF received each input before the clock moved. The checks allow for a late input. In a phase that waits for an output in every step, the filter fuses an input that arrives late one step later, and the checks look at the last output of a phase and at the highest of the outputs that the test keeps in a phase. [Phase 2 of `robot_rates`](#phase-2-of-robot_rates-the-imu-stops) moves the clock on in every step of its measured wheel periods without waiting for an output, and its check of the outputs allows a late wheel message within its wheel period.

Three helpers hold the checks of the fusion, which both scenarios use in their phase 1. The excerpt shows them, from [`test_ekf_synthetic.py`, lines 412–432]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L412-L432):

```python title="test_ekf_synthetic.py: assert_forward_speed(), assert_fused_estimate() and print_fusion()" linenums="412"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:412:432"
```

- **`assert_forward_speed()`:** the output's forward speed $v_x$ has to be `speed`, the wheels' forward speed, within `SPEED_TOLERANCE`, 0.002 m/s.
- **`assert_fused_estimate()`:** the output has the frame `odom` and the child frame `base_footprint`, the forward speed `speed` of `assert_forward_speed()`, and the yaw rate $\omega_z$ `IMU_RATE`, 0.5 rad/s, within `YAW_RATE_TOLERANCE`, 0.01 rad/s.
- **`print_fusion()`:** prints the inputs' yaw rates and the output's estimate, a line that the test's result file keeps.

## The seven phases of `invalid_imu_samples`

The scenario `invalid_imu_samples` is the class `TestEkfWithInvalidImuSamples`, one test on one filter. The excerpt shows the test method, from [`test_ekf_synthetic.py`, lines 453–460]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L453-L460). Each `check_...()` method is one phase, and each phase needs the state that the phases before it left. The method stops at the first phase that fails:

```python title="test_ekf_synthetic.py: the test method of invalid_imu_samples" linenums="453"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:453:460"
```

The figure shows the seven phases from left to right, one step every 20 ms of ROS time. The rows "IMU valid" and "IMU to drop" show the samples of one step: ✓ is a valid sample, ✕ is a sample that the validator has to drop. The chart below the rows shows the yaw rate $\omega_z$ that the wheel odometry and the valid IMU samples report, the range that the test accepts for the last output of each phase, the limit for the outputs that the test keeps in phases 2 to 5, and the estimates that the test printed in a run. The run printed the first and the last estimate of phase 2, so phase 2 has two markers. The three lists below the chart explain each mark and give its numbers.

--8<-- "includes/localization-test-stepped-clock.html"

In every step of every phase, the wheel odometry reports [`WHEEL_INPUT`](#the-inputs): `SPEED`, 0.1 m/s, and `WHEEL_RATE`, 0.4 rad/s, as with a wheel separation that is too large in the drive controller's configuration. The table lists what changes from phase to phase:

| Phase and its steps | IMU samples of each step | What the test asserts |
|:--|:--|:--|
| 1: `FUSION_STEPS`, 150 | One valid sample, 0.5 rad/s | The last output has the frames `odom` and `base_footprint`, $v_x$ = 0.1 ± 0.002 m/s and $\omega_z$ = 0.5 ± 0.01 rad/s |
| 2: `PHASE_STEPS`, 50 | One sample stamped 0.5 s before the step, 2.0 rad/s | No kept output above 0.51 rad/s. The last output has $\omega_z$ = 0.4 ± 0.02 rad/s |
| 3: 50 | The valid sample, then one with the same stamp, 2.0 rad/s | No kept output above 0.51 rad/s. The last output has $\omega_z$ = 0.5 ± 0.01 rad/s |
| 4: 50 | The valid sample, then one stamped 0.5 s after the step, 2.0 rad/s | As phase 3 |
| 5: 50 | The valid sample, then one stamped 0.01 s before the step, 2.0 rad/s | As phase 3 |
| 6: at most `RESET_STEPS`, 5, then `AFTER_RESET_STEPS`, 100 | The clock goes back to 50 s. Then one valid sample | An output comes within 5 steps. The first of the next 100 outputs is stamped before 100 s. The last output has $\omega_z$ = 0.5 ± 0.01 rad/s |
| 7: 75 | None | The validator's counts on the topic `/diagnostics` are the test's own counts |

Two helpers hold the comparisons. The excerpt shows them, from [`test_ekf_synthetic.py`, lines 399–410]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L399-L410). `assert_yaw_rate()` checks one output against `expected` ± `delta`. `assert_stays_at_most()` checks that none of a phase's kept outputs is above `limit`. The test keeps one output per step, the last one that [`step()`](#the-code-of-one-step) finds when its wait ends:

```python title="test_ekf_synthetic.py: the two comparisons" linenums="399"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:399:410"
```

### Phase 1: the estimate follows the IMU

The wheels report 0.4 rad/s and the IMU 0.5 rad/s, for 150 steps, 3 s of ROS time. The excerpt shows the phase, from [`test_ekf_synthetic.py`, lines 462–466]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L462-L466):

```python title="test_ekf_synthetic.py: phase 1" linenums="462"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:462:466"
```

[`assert_fused_estimate()`](#the-code-of-one-step) checks the last output:

- **The frames:** the last output is in the frame `odom` with the child frame `base_footprint`, as [`ekf.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/ekf.yaml) sets them.
- **The forward speed $v_x$:** `SPEED`, 0.1 m/s, within `SPEED_TOLERANCE`, 0.002 m/s. Only the wheel odometry carries a forward speed, so this check shows that the filter fuses the wheel odometry.
- **The yaw rate $\omega_z$:** `IMU_RATE`, 0.5 rad/s, within `YAW_RATE_TOLERANCE`, 0.01 rad/s. The IMU's variance is 75 times smaller than the wheels', so the estimate lies close to the IMU's yaw rate. The wheels' 0.4 rad/s is ten tolerances away, so the check shows that the filter fuses the IMU's samples.

### Phase 2: only samples that are too old

Each step sends one IMU sample that is stamped `STALE_OFFSET`, 0.5 s, before the step's time and reports `BAD_RATE`, 2.0 rad/s. 0.5 s is five times the validator's `max_age`, so the validator has to drop every one of these samples. The excerpt shows the phase, from [`test_ekf_synthetic.py`, lines 468–477]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L468-L477):

```python title="test_ekf_synthetic.py: phase 2" linenums="468"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:468:477"
```

- **No kept output above 0.51 rad/s:** the limit is `IMU_RATE + YAW_RATE_TOLERANCE`, and the test compares the one output that it keeps per step. A filter that fused one of the 2.0 rad/s samples would go far above this limit.
- **The last output:** `WHEEL_RATE`, 0.4 rad/s, within `WHEEL_RATE_TOLERANCE`, 0.02 rad/s. Without valid IMU samples, the wheel odometry is the filter's only source for the yaw rate, as when the IMU stops. The 50 steps, 1 s of ROS time, give the estimate the time to get there.

### Phases 3 to 5: a valid sample and one to drop

In these phases, each step sends [`VALID_SAMPLE`](#the-inputs) first and then a second sample that reports 2.0 rad/s. The excerpt shows the three phases, from [`test_ekf_synthetic.py`, lines 479–504]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L479-L504):

```python title="test_ekf_synthetic.py: phases 3 to 5" linenums="479"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:479:504"
```

- **Phase 3, a repeated stamp:** the second sample has the stamp of the valid one: its offset is the valid sample's, `VALID_SAMPLE.offset`, 0 s. The validator has to drop it as not newer than the last valid sample, check 5 of the [five checks](../../robot_localization.md#the-five-checks).
- **Phase 4, from the future:** the second sample is stamped `FUTURE_OFFSET`, 0.5 s, after the step's time, 25 times the validator's `max_future`. Check 4 has to drop it.
- **Phase 5, out of order:** the second sample is stamped `OLDER_OFFSET`, 0.01 s, before the step's time. This stamp is well within `max_age`, so check 4 passes. Check 5 has to drop the sample, because its stamp is older than the valid sample's of the same step.

The assertions are the same in the three phases. No kept output is above 0.51 rad/s, and the last output is 0.5 rad/s within 0.01 rad/s. So the valid samples keep reaching the filter. The validator's counts in [phase 7](#phase-7-the-validators-counts) show that the validator dropped the 150 samples that report 2.0 rad/s.

### Phase 6: the clock goes back

The test sets the clock back from 107 s to `RESTART`, 50 s, as a call of the service `/gazebo/reset_simulation` does in the simulation. The validator starts its order check over, as [a clock that goes back](validator.md#clear_last_valid_sample_times_if_clock_went_back-a-clock-that-goes-back) on the validator's tests page explains, and the EKF starts over from the new time, because [`ekf.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/ekf.yaml) sets `reset_on_time_jump`. The excerpt shows the phase, from [`test_ekf_synthetic.py`, lines 506–529]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L506-L529):

```python title="test_ekf_synthetic.py: phase 6" linenums="506"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:506:529"
```

1. The phase sets `self.now` to `RESTART`, publishes the clock and sleeps `RESET_DELIVERY`, 0.5 s of wall time, so that the jump reaches the nodes.
2. The phase runs up to `RESET_STEPS`, 5, steps with [`step_allowing_no_output()`](#the-code-of-one-step) and stops at the first step with an output of its own. The filter can stay without an output in the first steps after the jump. The comment in the excerpt says why. An output from before the jump that arrives late doesn't end this wait, because its stamp lies after the new clock and `outputs_since()` leaves it out. No output within 5 steps fails the test.
3. The phase runs `AFTER_RESET_STEPS`, 100, steps with valid samples, 2 s of ROS time.
4. The first of these outputs has to be stamped before `START`, 100 s. So the filter works in the new time.
5. The last output has to be 0.5 rad/s within 0.01 rad/s, as in [phase 1](#phase-1-the-estimate-follows-the-imu).

### Phase 7: the validator's counts

The validator counts every sample that it forwards and every sample that it drops, by reason, and reports the counts on the topic `/diagnostics` once per second of ROS time, as [the validator's diagnostics](../../robot_localization.md#the-validators-diagnostics) on the Localization page show. The test kept its own counts with [`count_expected_outcome()`](#the-code-of-one-step). Phase 7 compares the two. The excerpt shows the phase, from [`test_ekf_synthetic.py`, lines 531–571]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L531-L571):

```python title="test_ekf_synthetic.py: phase 7" linenums="531"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:531:571"
```

- **`wait_for_validator_diagnostics()`:** notes in `array_count_before` how many messages of the topic `/diagnostics` the test has so far. Then it runs steps without IMU samples until the clock is `DIAGNOSTICS_WAIT`, 1.5 s, further. That is longer than the 1 s between two reports, so a report from after the last IMU sample arrives. The method returns the keys and values of the validator's latest status.
- **`validator_values()`:** looks through the messages that arrived after the first `array_count_before`, the newest first, and finds the validator's status by its name. The name contains `VALIDATOR_STATUS_NAME`, "IMU sample validator". The test writes this text and the keys out and doesn't import them from the script, so the test pins what a user reads on the topic.
- **`assert_diagnostic_counts()`:** the value of `FORWARDED_KEY`, "Samples forwarded", has to be `self.expected_forward_count`. For each reason, the value of "Dropped, " plus the reason has to be the test's count. The last assertion compares the number of values with the number of expected reasons plus one, so the status has no other reason to drop a sample.

## The robot's rates: `robot_rates`

On the robot, the inputs arrive at different rates. The IMU is configured to publish at 50 Hz. The wheel odometry arrives at most 10 times per second, because the control loop of the node `diffbot_base` runs the drive controller at a nominal 10 Hz, as the box under [`frequency`](../../robot_localization.md#the-other-settings) on the Localization page explains. The scenario `robot_rates` feeds the filter at these rates: an IMU sample in every step, every 20 ms of ROS time, and a wheel message in every fifth step, every 100 ms. Then the IMU stops, and the wheel messages go on.

The scenario is the class `TestEkfAtTheRobotRates`. The excerpt shows the class with its docstring, its `setUp()` and the test method, from [`test_ekf_synthetic.py`, lines 574–604]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L574-L604). [Phase 2](#phase-2-of-robot_rates-the-imu-stops) needs a filter that fused the IMU's samples before, so the two phases are one test:

```python title="test_ekf_synthetic.py: the class, setUp() and the test method of robot_rates" linenums="574"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:574:604"
```

`setUp()` first runs the `setUp()` of [`SteppedClockTestCase`](#before-the-first-step), through `super()`. Then `setUp()` subscribes to the wheel odometry's topic itself, with the callback `on_wheel_odometry()`, which notes the stamp of each wheel message in `self.wheel_message_stamps`. So phase 2 counts the wheel messages that the topic carried, not only those that the test meant to send.

The figure shows the two phases from left to right, over the ROS time $t$. The rows "wheels" and "IMU" mark each wheel message and each IMU sample. On a narrow screen, the ticks of the IMU's samples, 20 ms apart, merge into a band. So the bracket at the top marks one wheel period of 100 ms: 5 IMU samples and 1 wheel message. The chart below the rows shows the yaw rate $\omega_z$ that the inputs report, the range that the test accepts for the last output of each phase, and the filter's estimate in each of its outputs, recorded in a run of the scenario. The chart ends with the last of the 20 wheel periods that phase 2 measures, at 105.00 s, so the messages after these periods lie beyond the chart. The lists below the chart explain each mark and give its numbers. [The figure in phase 2](#phase-2-of-robot_rates-the-imu-stops) shows three wheel messages of each phase in close-up.

--8<-- "includes/localization-test-stepped-clock-robot-rates.html"

The IMU samples report `IMU_RATE`, 0.5 rad/s, as in [`invalid_imu_samples`](#the-seven-phases-of-invalid_imu_samples). The wheel odometry reports `WHEEL_RATE`, 0.4 rad/s, in each of its messages, and a forward speed that changes once in phase 1. The table lists the two phases:

| Phase and its steps | Inputs | What the test asserts |
|:--|:--|:--|
| 1: `FUSION_STEPS`, 150 | One valid IMU sample in every step, a wheel message in every fifth step: 0.1 m/s, from step 100 on 0.12 m/s | An output in every step. The output of step 99 has the frames, $v_x$ = 0.1 ± 0.002 m/s and $\omega_z$ = 0.5 ± 0.01 rad/s. From step 106 on, the output that the test keeps for each step has $v_x$ = 0.12 ± 0.002 m/s. The last output has the frames, $v_x$ = 0.12 ± 0.002 m/s and $\omega_z$ = 0.5 ± 0.01 rad/s |
| 2: `IMU_STOPPED_WHEEL_PERIODS`, 20 measured wheel periods of 5 steps, then 1 extra period | No IMU sample, a wheel message of 0.1 m/s in the first step of each period | One wheel message on the topic in each wheel period. The header stamp of one output in each wheel period. The last output has $v_x$ = 0.1 ± 0.002 m/s and $\omega_z$ = 0.4 ± 0.02 rad/s |

### Phase 1 of `robot_rates`: the estimate follows the IMU and the wheels' speed

The phase runs 150 steps, 3 s of ROS time, with a valid IMU sample in each step and a wheel message in every fifth step. With the wheel message of step 100, 2 s into the phase, the wheels' forward speed changes from `SPEED`, 0.1 m/s, to `CHANGED_SPEED`, 0.12 m/s. Only the wheel odometry carries a forward speed. So the estimate's forward speed $v_x$ reaches 0.12 m/s only if the filter fuses the wheel messages after the change. The excerpt shows the phase, from [`test_ekf_synthetic.py`, lines 606–624]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L606-L624):

```python title="test_ekf_synthetic.py: phase 1 of robot_rates" linenums="606"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:606:624"
```

The phase runs its steps with `run_steps_at_the_robot_rates()`, which asks `wheel_input_at_the_robot_rates()` for each step's wheel odometry. The excerpt shows the method, from [`test_ekf_synthetic.py`, lines 668–676]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L668-L676):

```python title="test_ekf_synthetic.py: run_steps_at_the_robot_rates()" linenums="668"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:668:676"
```

The next excerpt shows the function that decides each step's wheel odometry, from [`test_ekf_synthetic.py`, lines 777–785]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L777-L785). The step with the `step_index` 0 publishes a wheel message, and so does every step whose `step_index` is a multiple of `STEPS_PER_WHEEL_MESSAGE`, 5. The other steps get `None` and publish no wheel odometry:

```python title="test_ekf_synthetic.py: wheel_input_at_the_robot_rates()" linenums="777"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:777:785"
```

The phase checks four things:

- **An output in every step:** each step goes through [`step()`](#the-code-of-one-step), which fails when the filter publishes no output for the step. Between two wheel messages, each IMU sample gives the filter a measurement to fuse, so the filter publishes 50 times per second of ROS time, as in [`invalid_imu_samples`](#the-seven-phases-of-invalid_imu_samples).
- **Before the change:** `assert_fused_estimate()` checks the output of the step with the `index` 99, `SPEED_CHANGE_STEP - 1`, against `SPEED` and `IMU_RATE`, with the tolerances of [phase 1 of `invalid_imu_samples`](#phase-1-the-estimate-follows-the-imu). Step 99 is the fourth step after the wheel message of step 95, so this check looks at an output 80 ms after the last wheel message. Between two wheel messages, the filter's prediction keeps $v_x$, and the IMU's samples carry only the yaw rate $\omega_z$.
- **After the change:** `assert_forward_speeds()` checks the output that the test keeps for each step, the last one that `step()` returns, from the step with the `index` 106 on, `SPEED_CHANGE_STEP + SPEED_CHANGE_WINDOW`, against `CHANGED_SPEED`, 0.12 m/s, within 0.002 m/s. `SPEED_CHANGE_WINDOW`, 6 steps, gives the estimate time for two wheel messages, in the steps 100 and 105, and one more step for a message that arrives late. The constants' comments say why the change is 0.02 m/s: the filter estimates an acceleration from the change, which moves the estimate between two wheel messages, by less than the tolerance for this small change.
- **The last output:** `assert_fused_estimate()` checks the output of the step with the `index` 149, again the fourth after a wheel message, against `CHANGED_SPEED` and `IMU_RATE`.

The excerpt shows the helper of the check after the change, from [`test_ekf_synthetic.py`, lines 724–729]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L724-L729):

```python title="test_ekf_synthetic.py: assert_forward_speeds()" linenums="724"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:724:729"
```

### Phase 2 of `robot_rates`: the IMU stops

The phase sends no IMU sample. The wheel messages go on, one in every fifth step, for `IMU_STOPPED_WHEEL_PERIODS`, 20 wheel periods, and report 0.1 m/s again. A wheel period runs from one wheel message's stamp to the next one's, 100 ms. The phase checks that each wheel period contains the header stamp of one output, and that the estimate goes back to the wheels' forward speed and to their yaw rate.

An output's message gives the time of its estimate in its header stamp, which the filter writes into the message. robot_localization's debug trace tells when the filter published the output, and that ROS time can be later. The figure shows both times, for three wheel messages at the end of [phase 1](#phase-1-of-robot_rates-the-estimate-follows-the-imu-and-the-wheels-speed) and three in phase 2 of a recorded run. In each drawing, the row "wheels" marks the wheel messages at their stamps, the row "IMU" the IMU samples, and the row "EKF" each output at the ROS time of the update that published it. In the lower drawing, a thin line leads down from each wheel message's stamp, and an arrow leads from there to an output that was published later.

--8<-- "includes/localization-test-stepped-clock-between-wheels.html"

The table follows the first two wheel messages of the lower drawing, one column for each. All times are ROS times:

| Wheel message's stamp | 103.10 s | 103.20 s |
|:--|--:|--:|
| What the EKF did | fused the wheel message | predicted at the timeout |
| Output's header stamp | 103.10 s | 103.20 s |
| Published at | 103.12 s | 103.20 s |

What the test asserts and what the recorded run shows, side by side:

- **The test asserts:** the header stamp of one output in each wheel period, at any time within the period. So a wheel message that reaches the filter late can still pass. The test doesn't look at the times at which the outputs are published.
- **The recorded run shows:** each of the 20 outputs is stamped exactly with its wheel message's time. 12 of them were published 20 ms after their stamp, by the update that fused the wheel message. 8 were published at their stamp, by a prediction at the sensor timeout. So a fused output is published between two wheel messages, while its header stamp lies at the start of its period.

The excerpt shows the phase, from [`test_ekf_synthetic.py`, lines 626–666]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L626-L666). Its docstring says what the filter does between two wheel messages:

```python title="test_ekf_synthetic.py: phase 2 of robot_rates" linenums="626"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:626:666"
```

The phase runs its steps with `run_wheel_periods_without_imu()`. The excerpt shows the method, from [`test_ekf_synthetic.py`, lines 678–692]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L678-L692):

```python title="test_ekf_synthetic.py: run_wheel_periods_without_imu()" linenums="678"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:678:692"
```

1. A wheel period is `STEPS_PER_WHEEL_MESSAGE`, 5, steps. The first step of a period notes the clock `self.now` in `period_starts` and publishes a wheel message stamped with this time, which reports [`WHEEL_INPUT`](#the-inputs).
2. The other 4 steps of the period publish nothing.
3. Every step runs with [`step_allowing_no_output()`](#the-code-of-one-step): the clock moves on whether the filter published or not, and each step waits `NO_OUTPUT_WAIT`, 0.2 s of wall time, so that the filter's update for the new clock very likely runs before the next step.
4. After `IMU_STOPPED_WHEEL_PERIODS`, 20, periods, the clock stands at the end of the last period, 2 s of ROS time after the phase's start.

robot_localization decides in each update what to publish, in four places of its code:

- **When the filter predicts without a measurement:** in each update, the filter fuses the measurements that have arrived up to the ROS time. With no measurement in its queue, the filter predicts to the ROS time only when the ROS time is at least [`sensor_timeout`](../../robot_localization.md#the-other-settings), 0.1 s, past its last measurement. 0.1 s is also the wheel odometry's period.
- **The stamp of an output:** the filter's last measurement time, which a prediction moves to the ROS time.
- **No second output with the same stamp:** the filter publishes an output only when its stamp is later than the stamp of the last output it published. [`ekf.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/ekf.yaml) doesn't set `permit_corrected_publication`, which would allow it.
- **A measurement older than the filter's time:** the filter corrects its state with it but doesn't predict and doesn't move its time back.

??? info "The code in robot_localization 2.7.7"

    The function [`integrateMeasurements()`](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ros_filter.cpp#L597) predicts without a measurement only when `lastUpdateDelta`, the ROS time minus the filter's last measurement time, is at least the sensor timeout. The prediction moves the last measurement time to the ROS time. The excerpt shows this part, from [lines 680–711 of `ros_filter.cpp`](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ros_filter.cpp#L680-L711):

    ```cpp title="ros_filter.cpp: integrateMeasurements() with an empty queue" linenums="680"
    --8<-- "https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ros_filter.cpp:680:711"
    ```

    The function `getFilteredOdometryMessage()` stamps the output with the filter's last measurement time, as [lines 427–429 of `ros_filter.cpp`](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ros_filter.cpp#L427-L429) show:

    ```cpp title="ros_filter.cpp: the output's stamp and frames" linenums="427"
    --8<-- "https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ros_filter.cpp:427:429"
    ```

    The function `periodicUpdate()` marks an output as `corrected_data` when its stamp isn't later than `lastPublishedStamp_`, as [lines 1975–1980 of `ros_filter.cpp`](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ros_filter.cpp#L1975-L1980) show. `permitCorrectedPublication_` is the parameter `permit_corrected_publication`, whose default is `false`:

    ```cpp title="ros_filter.cpp: an output with the stamp of the last one" linenums="1975"
    --8<-- "https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ros_filter.cpp:1975:1980"
    ```

    The function publishes only an output that isn't `corrected_data`, as [lines 2048–2055 of `ros_filter.cpp`](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ros_filter.cpp#L2048-L2055) show:

    ```cpp title="ros_filter.cpp: publishing the output" linenums="2048"
    --8<-- "https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ros_filter.cpp:2048:2055"
    ```

    The function [`processMeasurement()`](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/filter_base.cpp#L205) of the filter predicts to a measurement's time only when `delta`, the measurement's time minus the last measurement time, is positive, and corrects in every case, as [lines 214–235 of `filter_base.cpp`](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/filter_base.cpp#L214-L235) show:

    ```cpp title="filter_base.cpp: predict, then correct" linenums="214"
    --8<-- "https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/filter_base.cpp:214:235"
    ```

    The same function moves the last measurement time only for a `delta` that isn't negative, as [lines 261–264 of `filter_base.cpp`](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/filter_base.cpp#L261-L264) show:

    ```cpp title="filter_base.cpp: the last measurement time" linenums="261"
    --8<-- "https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/filter_base.cpp:261:264"
    ```

`sensor_timeout` in [`ekf.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/ekf.yaml) is 0.1 s, the same as the wheel period. So the setting sits exactly at the limit: the ROS time at which the filter may predict without a measurement is the stamp of the next wheel message.

The three recordings have their wheel messages on time. In them, with the IMU stopped, the updates 40, 60 and 80 ms after a wheel message's stamp publish nothing. In these updates, the filter has nothing to fuse, and its last measurement is less than 0.1 s old. In the test, the filter's update at the stamp $t_\text{w}$ of the next wheel message runs when the clock reaches $t_\text{w}$, before the test publishes the message. In the first period, that update at 103.00 s still has the last IMU sample, stamped 102.98 s, in its queue and fuses it, and the update 20 ms later fuses the wheel message. In the other 19 periods, the queue of the update at $t_\text{w}$ is empty, and the update decides between two cases:

- **Fused:** the ROS time is less than 0.1 s past the filter's last measurement, so the update does nothing. The update 20 ms later fuses the message and publishes the result, stamped $t_\text{w}$, at the ROS time $t_\text{w}$ + 20 ms.
- **Predicted:** the ROS time is 0.1 s or more past the filter's last measurement, so the update predicts to $t_\text{w}$ and publishes the prediction, stamped $t_\text{w}$, at the ROS time $t_\text{w}$, with the yaw rate $\omega_z$ of the output before. The update 20 ms later fuses the message, but its output has the same stamp and isn't published. The next output, stamped with the next wheel message's time, contains the message.

Which of the two happens depends on the last digits of the difference between two stamps. The filter computes with times in seconds as double-precision floating-point numbers, and the difference of two stamps 0.1 s apart comes out a little above or a little below 0.1. For example, 103.2 s minus 103.1 s gives 0.1000000000000085, and the filter predicts. 103.3 s minus 103.2 s gives 0.0999999999999943, and the filter waits for the message. In each of three recorded runs, robot_localization's debug trace shows 8 of these 19 outputs as predicted and 11 as fused, at the same wheel messages: the predicted ones where the difference of two stamps comes out at 0.1 or above. With the first period, 12 of the 20 outputs of phase 2 are fused. In both cases, the output is stamped with the wheel message's time.

A wheel message can also reach the filter late, for example on a busy computer. The test doesn't wait for an output in the measured periods, so the clock moves on, and the filter fuses the message in a later update of its period. That late update either publishes the message, stamped with its time, or an earlier update of the period has already predicted and published, stamped with that update's time. Then the late message is older than the filter's time: the filter corrects with it but publishes no output for it. So each period still gets one output stamp, as long as its wheel message reaches the filter, and the filter's updates run, before the clock reaches the next wheel message's stamp. The steps' sleeps give a message about 0.9 s of wall time for that, five times `DELIVERY` and four times `NO_OUTPUT_WAIT`, but they don't prove that the filter's updates ran. The check of the outputs allows any stamp within a wheel period. Two functions put stamps into wheel periods. The excerpt shows them, from [`test_ekf_synthetic.py`, lines 788–798]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L788-L798):

```python title="test_ekf_synthetic.py: within_wheel_periods() and wheel_period_index()" linenums="788"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:788:798"
```

- **`within_wheel_periods()`:** returns whether a stamp lies from the first period's start to before the end of the last period. The phase takes the outputs and the wheel messages by their stamps with this function, from all that the test has received after one more wheel period, which the next excerpt shows. So the stamps, not the order of arrival, decide the period of each message.
- **`wheel_period_index()`:** returns for a stamp the index of its wheel period, the number of periods that started at or before the stamp, minus one. A period starts at a wheel message's stamp and ends at the next one's.

The phase reads the outputs and the wheel messages only after one more wheel period, which it doesn't measure. `run_wheel_periods_without_imu(1)` runs this period like the others, from the end of the last measured period at 105.00 s, and returns its start. Then `wait_for_messages_from()` waits for the messages of this extra period. The excerpt shows this method and the two methods that take the outputs and the wheel messages of the measured periods, from [`test_ekf_synthetic.py`, lines 694–722]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L694-L722):

```python title="test_ekf_synthetic.py: the wait after the measured periods, and what the phase reads" linenums="694"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:694:722"
```

- **`wait_for_messages_from()`:** waits with [`wait_until()`](#before-the-first-step), at most `WAIT`, until the test's subscriber has the extra period's wheel message, stamped at its start, and an output stamped at that start or later. Each subscription receives the messages of its one publisher in their order and runs their callbacks in that order. So when these two messages have arrived, the callbacks of all earlier wheel messages and outputs have run. The extra period moves the clock on by a whole wheel period first, so that the filter publishes an output even when the extra wheel message reaches the filter late. The method doesn't fail. It returns the names of the messages that didn't arrive.
- **`outputs_within_the_periods()` and `wheel_message_stamps_within_the_periods()`:** take the outputs and the wheel messages' stamps that lie within the measured periods, by `within_wheel_periods()`, from all that the test received, in the order of arrival. The extra period's messages lie after the last measured period, so they don't count.

The excerpt shows the two assertions of the periods, from [`test_ekf_synthetic.py`, lines 731–747]({{ ekf_test_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L731-L747):

```python title="test_ekf_synthetic.py: the two assertions of the wheel periods" linenums="731"
--8<-- "{{ ekf_test_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:731:747"
```

- **`assert_one_wheel_message_per_period()`:** the stamps of the received wheel messages have to be exactly the periods' starts, in their order, one message at the start of each period. A failure names the starts without a wheel message.
- **`assert_one_output_per_period()`:** turns each output's header stamp into its period's index. The list of indices has to be 0, 1, 2 and so on, one output stamp in each period, in the order of the periods.

The phase checks these things, in this order:

- **Outputs at all:** without any output within the measured periods, the phase fails with the message that it got no EKF output in the 20 wheel periods.
- **The extra period's messages:** its wheel message and an output from its start on arrived. Otherwise messages of the measured periods might still be missing, and the phase fails with a message that names what didn't arrive.
- **One wheel message per wheel period:** `assert_one_wheel_message_per_period()` checks the wheel messages that the test's own subscriber received on the topic. So the wheel topic carried the 20 wheel messages, one at the start of each period.
- **One output stamp per wheel period:** the phase takes the outputs stamped within the periods, before the end of the last period. The extra period's outputs are stamped at that end or later, so the phase leaves them out. `assert_one_output_per_period()` checks that each of the 20 periods contains the header stamp of one output, in their order. A filter that put two output stamps into one period would fail, for example one that predicted again before the next wheel message. In runs of the scenario with changed settings and the wheel messages on time, a `sensor_timeout` of 0.08 s of ROS time gave 24 outputs for the 20 periods, and `predict_to_current_time`, which makes the filter predict in every update, gave 100, five in each period. Both runs failed this check. The check doesn't prove the setting itself. In a run in which each of the 20 wheel messages of the measured periods was published 0.5 s of wall time late, a `sensor_timeout` of 0.08 s gave one output stamp per period, and the run passed. The steps moved the clock on by only 60 ms of ROS time during those 0.5 s, so the test published each message 60 ms of ROS time after its stamp, for example the message stamped 103.00 s at the ROS time 103.06 s. Both `sensor_timeout` and this delay are ROS times. The test prints how many outputs carry a wheel message's stamp.
- **The forward speed $v_x$:** the last output's, `SPEED`, 0.1 m/s within 0.002 m/s, with `assert_forward_speed()`. The estimate comes from phase 1's 0.12 m/s, so the estimate gets back to 0.1 m/s only if the filter fuses the wheel messages of phase 2.
- **The yaw rate $\omega_z$:** the last output's, `WHEEL_RATE`, 0.4 rad/s, within `WHEEL_RATE_TOLERANCE`, 0.02 rad/s. The wheel odometry is the filter's only source of the yaw rate in this phase. In the recorded run, the estimate was within the tolerance from the eleventh output on, and 0.4016 rad/s at the end.

## Runs

The runs on this page are runs of the commit [`3882a19`](https://github.com/ros-mobile-robots/diffbot/commit/3882a19cbda8543f5a0f1c5d4f0965caa90d4f3d) of [diffbot#220](https://github.com/ros-mobile-robots/diffbot/pull/220), in the dev container's image. Each scenario passed in 12 runs of 12, with the same printed values in each run. The rostest command took 17 s of wall time for [`invalid_imu_samples`](#the-seven-phases-of-invalid_imu_samples) and 31 s for [`robot_rates`](#the-robots-rates-robot_rates).

### `invalid_imu_samples`

In a run, the scenario printed these lines:

```text
fusion: wheels 0.4 rad/s, IMU 0.5 rad/s, estimate 0.4987 rad/s, forward 0.1000 m/s
stale IMU: estimate from 0.4966 to 0.4005 rad/s
after the clock went back to 50 s: estimate 0.4987 rad/s at 52.02 s
validator diagnostics: {'Samples forwarded': '402', 'Dropped, from the future': '50', 'Dropped, not newer than the last one': '100', 'Dropped, too old': '50'}
```

- **The estimate:** 0.4987 rad/s at the end of phase 1 and of phase 6, and 0.4005 rad/s at the end of phase 2. These are the three dots in [the figure of the seven phases](#the-seven-phases-of-invalid_imu_samples). 0.4966 rad/s is the first estimate of phase 2, the diamond in the figure.
- **The forwarded samples:** 402 are the 150 of phase 1, the 3 · 50 of phases 3 to 5, and the 102 of phase 6. In this run, the filter published again in the second step after the clock went back, so phase 6 had 2 + 100 steps.
- **The dropped samples:** 50 too old from phase 2, 100 not newer from phases 3 and 5, and 50 from the future from phase 4.

### `robot_rates`

In a run, the scenario printed these lines:

```text
fusion at the robot's rates: wheels 0.4 rad/s, IMU 0.5 rad/s, estimate 0.5000 rad/s, forward 0.1207 m/s
speed change at 102.00 s, wheels 0.1 to 0.12 m/s: estimate 0.1000 m/s before, 0.1155 m/s in the step of the change, 0.1194 to 0.1209 m/s from 6 steps after it
IMU stopped: 20 outputs for 20 wheel periods with 20 wheel messages, 20 of the outputs stamped at a wheel message, estimate from 0.4979 to 0.4016 rad/s, forward from 0.1046 to 0.0997 m/s
```

- **Phase 1, the yaw rate:** the last output has 0.5000 rad/s. In the recorded run, the filter's first output, at the first wheel message, has 0.4987 rad/s. From the second wheel message on, each wheel message pulled the estimate down to 0.4992 rad/s, and the IMU's samples brought it back to 0.5000 rad/s within three steps. In [`invalid_imu_samples`](#the-seven-phases-of-invalid_imu_samples), with a wheel message in every step, phase 1 ends at 0.4987 rad/s.
- **Phase 1, the forward speed:** 0.1000 m/s in the output of step 99, before the change. 0.1155 m/s in the output of step 100, the first that contains a wheel message of 0.12 m/s. From step 106 on, the outputs that the test keeps lie between 0.1194 and 0.1209 m/s, within 0.002 m/s of 0.12 m/s. 0.1207 m/s at the end of the phase.
- **Phase 2:** 20 outputs for 20 wheel periods, with 20 wheel messages on the topic, each output stamped with its wheel message's time. The estimate goes from 0.4979 to 0.4016 rad/s, and the forward speed from 0.1046 to 0.0997 m/s.

The figures of [`robot_rates`](#the-robots-rates-robot_rates) come from a recording run of the scenario at the same commit, in a copy of `ekf_synthetic_robot_rates.test` with two additions. [rosbag](https://wiki.ros.org/rosbag) records the filter's outputs and the inputs, and robot_localization writes its debug trace, a log of each update, which the filter's parameter [`debug`](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ros_filter.cpp#L774) turns on. The outputs' stamps, yaw rates and forward speeds come from the recording. The ROS time of the update that published each output, and whether that update predicted at the sensor timeout, come from the debug trace. The scenario passed in each of three such runs, with the same outputs. Neither the recording nor the trace is part of the test. Their values are in [`localization-stepped-clock-test-runs.jsonl`]({{ docs_repo_url }}/scripts/figures/data/localization-stepped-clock-test-runs.jsonl), which [`localization_control_test_figures.py`]({{ docs_repo_url }}/scripts/figures/localization_control_test_figures.py) reads to draw the figures.

## What the test doesn't show

- **The rates on the robot:** the scenario [`robot_rates`](#the-robots-rates-robot_rates) uses the robot's nominal rates, the IMU at 50 Hz and the wheel odometry at 10 Hz. These rates are not measured on the robot yet. The issue [Base: measure the control loop's rate on the robot, then decide (diffbot#218)](https://github.com/ros-mobile-robots/diffbot/issues/218) measures them.
- **Whether `sensor_timeout` fits the robot:** the scenario runs with the robot's `sensor_timeout` of 0.1 s, the nominal wheel period. The filter runs its update 50 times per second, `frequency` in [`ekf.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/ekf.yaml). An update whose queue holds no new measurement predicts without one, and publishes the prediction, when the time since the filter's last measurement computes to at least `sensor_timeout` ([`ros_filter.cpp`, lines 680–711](https://github.com/cra-ros-pkg/robot_localization/blob/ef5ad3c89457209530e262f3bac39b0f35d07b2b/src/ros_filter.cpp#L680-L711)). In [phase 2](#phase-2-of-robot_rates-the-imu-stops), only the wheels report, every 0.1 s. The update at a wheel message's stamp runs before that message is in the filter's queue. In the first period, that update still fuses the last IMU sample, and the next update fuses the wheel message. In 8 of the other 19 periods, the time since the last measurement computes to 0.1 s or a little more at that update, and the update publishes a prediction. In the other 11, it computes to a little less, and the next update, 20 ms later, fuses the wheel message. With a larger value, for example 0.12 s, the update at a wheel message's stamp wouldn't predict, and the next update would fuse the message, if the message is in the queue by then. The price: when all inputs stop, the filter predicts first at the first update at which the time since its last measurement computes to at least 0.12 s. With the last measurement at 103.20 s, 103.32 s minus 103.20 s computes to 0.11999999999999034 s, so the first prediction comes at 103.34 s, 0.14 s after the last measurement. On the robot, the value has to cover the wheel period and the delay of the wheel messages. The issue [Base: measure the control loop's rate on the robot, then decide (diffbot#218)](https://github.com/ros-mobile-robots/diffbot/issues/218) measures both and sets the value.
- **The two clocks of the robot's inputs:** in the test, a wheel message and an IMU sample of the same step have the same stamp. On the robot, the wheel odometry is stamped with the time of the control loop, and the IMU's samples with the time of the Teensy, which rosserial keeps in step with the robot's single-board computer (SBC). The test doesn't show how the filter handles a difference between the two.
- **Real sensors:** the inputs have no noise, no bias and no delay. How the real gyroscope behaves is a measurement on the robot, as [Limits](../../robot_localization.md#limits) says.
- **The pose and the transform:** the test checks the forward speed $v_x$ and the yaw rate $\omega_z$ of the filter's output, and the two frame names in the output's message: `header.frame_id` has to be `odom` and `child_frame_id` `base_footprint`. `assert_fused_estimate()` checks them in lines 422 and 423 of `test_ekf_synthetic.py`, which the excerpt in [the code of one step](#the-code-of-one-step) shows. It doesn't check the position, the heading $\theta$ or the transform `odom → base_footprint` on the topic `/tf`. The [test in Gazebo](gazebo.md) compares the heading with the simulated robot's true turn.
- **Where the validator's limits lie:** the samples that phases 2 and 4 send to be dropped are stamped 0.5 s before or after the step's time, far outside `max_age`, 0.1 s, and `max_future`, 0.02 s. So the test shows that the running node drops clearly invalid samples, not where the boundary between valid and invalid lies. The validator's unit tests check the boundary: the validator accepts a sample exactly at a limit and rejects one 1 µs beyond it, as [Check 4: the two time limits](validator.md#check-4-the-two-time-limits) shows.
