# Test: The EKF on a Stepped Clock

What the stepped-clock test checks on the EKF and the IMU sample validator, and how the test does it. This page belongs to the [tests of the Localization](index.md).

[`test_ekf_synthetic.py`]({{ diffbot_repo_url }}/diffbot_control/test/test_ekf_synthetic.py) runs the two nodes of [`ekf.launch`]({{ diffbot_repo_url }}/diffbot_control/launch/ekf.launch), the [IMU sample validator](../../robot_localization.md#the-imu-sample-validator) and robot_localization's EKF, with the parameters that the robot and the simulation use. The test computes their inputs itself: wheel odometry and IMU samples that report a constant yaw rate $\omega_z$. The test also owns the clock. It publishes the topic `/clock` and moves the ROS time $t$ on in steps of 20 ms. So the test decides which messages are published, with which stamps and at which ROS time, without Gazebo and without a robot.

A pass proves four things about the filter's estimate of the yaw rate $\omega_z$. The estimate follows the IMU while valid samples arrive. The estimate goes to the wheels' yaw rate when no valid sample arrives. Samples that the validator has to drop don't reach the estimate. And the estimate follows the IMU again after the clock went back. A pass also proves that the validator's counts on the topic `/diagnostics` match the samples that the test sent. [What the test doesn't show](#what-the-test-doesnt-show) lists what a pass doesn't prove.

## What the test file starts

rostest reads [`ekf_synthetic.test`]({{ diffbot_repo_url }}/diffbot_control/test/ekf_synthetic.test), starts a ROS master and the nodes that the file declares, and runs the test node. The excerpt shows the whole file, from [`ekf_synthetic.test`, lines 1–12]({{ ekf_src_url }}/diffbot_control/test/ekf_synthetic.test#L1-L12):

```xml title="ekf_synthetic.test" linenums="1"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/ekf_synthetic.test:1:12"
```

- **`use_sim_time`:** every node takes the ROS time from the topic `/clock`. No simulator runs here, so the only publisher of that topic is the test.
- **`diffbot/robot_description`:** DiffBot's URDF model, which xacro builds from `diffbot.urdf.xacro`.
- **`robot_state_publisher`:** publishes the fixed joints of this URDF model, the joint `imu_joint` among them. From them, TF knows where the IMU's frame `imu_link` sits on `base_footprint`, as on the robot. The EKF needs this transform to use the IMU's samples.
- **`ekf.launch`:** starts the nodes `/diffbot/imu_validator` and `/diffbot/ekf_localization` with the parameters of the robot: the validator's `max_age` and `max_future`, and [`ekf.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/ekf.yaml) for the filter.
- **The `<test>` tag:** runs `test_ekf_synthetic.py` as the test node. `time-limit` gives the test 120 s of wall time, the computer's clock.

`diffbot_control` registers the file with `add_rostest()` in its [`CMakeLists.txt`]({{ diffbot_repo_url }}/diffbot_control/CMakeLists.txt), so CI runs it with the workspace's other tests, as [Testing and CI](../../../../development/ci.md#ekf-tests) describes. To run only this test, open a shell in which the workspace is sourced:

```console
rostest diffbot_control ekf_synthetic.test
```

## The inputs

The test publishes on the two topics that the nodes read on the robot. The wheel odometry goes to the EKF on the topic `/diffbot/mobile_base_controller/odom`. The IMU samples go to the validator on the topic `/diffbot/imu/data_raw`. The test subscribes to the validator's output, the topic `/diffbot/imu/data_validated`, to the filter's output, the topic `/diffbot/odometry/filtered`, and to the topic `/diagnostics`.

??? example "The constants of `test_ekf_synthetic.py`, lines 53–117"

    The excerpt shows all constants of the file, from [`test_ekf_synthetic.py`, lines 53–117]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L53-L117). `STEP`, `START`, `RESTART` and `DIAGNOSTICS_WAIT` are ROS times, which the test sets. `WAIT`, `DELIVERY`, `POLL_INTERVAL`, `NO_OUTPUT_WAIT`, `TF_DELIVERY` and `RESET_DELIVERY` are seconds of wall time, the computer's clock.

    ```python title="test_ekf_synthetic.py: the constants" linenums="53"
    --8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:53:117"
    ```

Two functions build the messages. The excerpt shows both, from [`test_ekf_synthetic.py`, lines 182–211]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L182-L211):

```python title="test_ekf_synthetic.py: wheel_odometry() and imu_sample()" linenums="182"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:182:211"
```

- **`wheel_odometry()`:** a `nav_msgs/Odometry` message as the drive controller publishes it, in the frame `odom` with the child frame `base_footprint`. The forward speed $v_x$ is `SPEED`, 0.1 m/s. The yaw rate $\omega_z$ is the function's argument. The variances $\sigma^2$ on the diagonals of both covariances are `WHEEL_VARIANCES`, with 0.03 (rad/s)² for the yaw rate. These are the numbers of [`diffbot_control.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/diffbot_control.yaml), which the test writes out and doesn't read from the file.
- **`imu_sample()`:** a `sensor_msgs/Imu` message as the firmware publishes it, in the frame `imu_link`. The orientation is marked unavailable, the yaw rate $\omega_z$ is the function's argument, and the upward acceleration $f_z$ is `GRAVITY`. The variance of the angular velocity is `IMU_ANGULAR_VELOCITY_VARIANCE`, 4 · 10⁻⁴ (rad/s)², the firmware's number, written out in the test too.
- **No noise:** both messages report the same values in every step. So what the test sees in the estimate comes from the filter and the validator, not from chance.

A step can publish several IMU samples. The class `ImuInput` describes one of them. The excerpt shows the class and the sample that most steps use, from [`test_ekf_synthetic.py`, lines 122–131]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L122-L131):

```python title="test_ekf_synthetic.py: ImuInput and VALID_SAMPLE" linenums="122"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:122:131"
```

- **`offset`:** the sample's stamp $t_s$ relative to the step's ROS time $t$, in seconds.
- **`yaw_rate`:** the yaw rate $\omega_z$ that the sample reports.
- **`drop_reason`:** `None` for a sample that the validator has to forward. For a sample that the validator has to drop, the reason as the validator's diagnostics name it.
- **`VALID_SAMPLE`:** stamped with the step's time, reporting `IMU_RATE`, 0.5 rad/s.

## Before the first step

unittest calls `setUp()` before the test method. The excerpt shows `setUp()` and its two helpers `on_validated()` and `all_connected()`, from [`test_ekf_synthetic.py`, lines 231–270]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L231-L270):

```python title="test_ekf_synthetic.py: setUp(), on_validated() and all_connected()" linenums="231"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:231:270"
```

1. `setUp()` sets the test's own ROS time `self.now` to `START`, 100 s, and creates the three logs and the two counters that the phases fill. `self.forwarded_stamp_keys` holds the stamps of the samples that the validator forwarded, `self.ekf_outputs` the filter's outputs, and `self.diagnostics` the messages of the topic `/diagnostics`. `self.expected_forward_count` and `self.expected_drops` count what the validator has to do with each sample that the test sends.
2. `setUp()` creates the three publishers and the three subscribers. `self.clock_publisher` is latched, so a node that connects later still gets the last clock message. Each subscriber's callback appends what arrives to one of the three logs. For the validator's output, the callback is `on_validated()`, which appends the sample's stamp.
3. `setUp()` publishes the clock for the first time, at `START`.
4. `setUp()` waits until the validator and the EKF are connected to the test's topics, with `all_connected()`, and until TF has the transform from `imu_link` to `base_footprint`. Then it sleeps `TF_DELIVERY`, 1 s of wall time, so that the EKF's own TF listener has the transform too.

The three logs are objects of the class `ArrivalLog`. rospy calls a subscriber's callback in a thread of its own, while the test's thread reads what arrived. The excerpt shows the class, from [`test_ekf_synthetic.py`, lines 137–167]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L137-L167). A lock guards every access, so the test's thread never reads a log while a callback changes it:

```python title="test_ekf_synthetic.py: ArrivalLog" linenums="137"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:137:167"
```

- **`append()`:** adds an item at the end. The subscribers of the filter's output and of the topic `/diagnostics` have it as their callback, and `on_validated()` calls it.
- **`count()`:** returns the number of items so far. The test takes this number before a step.
- **`since()`:** returns a copy of the items after the first `count` items. With the number from before a step, the test gets what arrived since the step started.
- **`contains_all()`:** returns whether each of the given items has arrived.

The clock stands still during `setUp()`. A wait on the ROS time would never end, so the test waits on the wall clock with `wait_for()`. The excerpt shows `wait_for()`, from [`test_ekf_synthetic.py`, lines 214–220]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L214-L220). The function checks `condition()` every `POLL_INTERVAL`, 1 ms, and fails the test with the text `failure` after `WAIT`, 5 s:

```python title="test_ekf_synthetic.py: wait_for()" linenums="214"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:214:220"
```

## One step

Every phase is a number of steps. One step publishes the inputs for one ROS time, moves the clock on by `STEP`, 20 ms, and collects the filter's output. 20 ms is the period of the IMU at 50 Hz. The excerpt shows `advance()`, `outputs_since()` and `step()`, from [`test_ekf_synthetic.py`, lines 306–338]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L306-L338). The docstring of `step()` lists the five things that one step does, and `advance()` does the first four:

```python title="test_ekf_synthetic.py: advance(), outputs_since() and step()" linenums="306"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:306:338"
```

The figure shows the messages of one step with a valid IMU sample. Read the figure from top to bottom. The three boxes are the test, the validator and the EKF. Each arrow is a message, from the node that publishes it to the node that receives it. The label above an arrow names the message, and the note below the arrow says how the message is stamped. $t$ is the ROS time of the step. The numbers are those of the list in the docstring of `step()`, and the key below the drawing names the code of each number.

--8<-- "includes/localization-test-stepped-clock-step.html"

The five things of one step, as the docstring of `step()` numbers them:

1. **Publish the inputs:** `publish_inputs()` publishes the step's wheel odometry, stamped with the step's ROS time, and its IMU samples, each stamped with that time plus the sample's `offset`.
2. **Wait for the validator:** `wait_for_forwarded()` waits on the wall clock until the validator's output for the step's valid samples reached the test. The EKF subscribes to the same topic.
3. **Wait `DELIVERY`:** `advance()` sleeps `DELIVERY`, 20 ms of wall time, so that the inputs reach the EKF before the clock moves.
4. **Advance the clock:** `advance_clock()` adds `STEP` to `self.now` and publishes the new clock. The EKF's update runs 50 times per second of ROS time, so the new clock lets the filter process the step's inputs.
5. **Wait for the EKF:** `step()` waits until the filter published an output that is stamped at the step's time or later, at most `WAIT`, 5 s of wall time. `step()` returns the last such output.

`advance()` returns a `StepStart`, which says where the step started. The excerpt shows the class, from [`test_ekf_synthetic.py`, lines 170–174]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L170-L174):

```python title="test_ekf_synthetic.py: StepStart" linenums="170"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:170:174"
```

- **`stamp`:** the step's ROS time. `advance()` reads it from `self.now` before the clock moves.
- **`output_count_before`:** the number of EKF outputs that the test had before the step, from `self.ekf_outputs.count()`.
- **`outputs_since()`:** takes the `StepStart` and returns the EKF's outputs that arrived after the first `output_count_before` and are stamped at the step's time or later. So an output of an earlier step doesn't count for this step.

`advance()` calls three helpers, and `publish_inputs()` calls a fourth, `expect()`. The excerpt shows the four helpers, from [`test_ekf_synthetic.py`, lines 272–304]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L272-L304):

```python title="test_ekf_synthetic.py: publish_inputs(), expect(), wait_for_forwarded() and advance_clock()" linenums="272"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:272:304"
```

- **`publish_inputs()`:** publishes the wheel odometry, stamped with the step's time `stamp`. For each `ImuInput` of the step, it publishes an IMU sample stamped `stamp` plus the sample's `offset`, and passes the `ImuInput` to `expect()`. It returns `valid_stamp_keys`, the stamps of the samples that the validator has to forward.
- **`expect()`:** counts what the validator has to do with the sample. A sample without a `drop_reason` adds one to `self.expected_forward_count`. A sample with a `drop_reason` adds one to that reason in `self.expected_drops`.
- **`wait_for_forwarded()`:** waits with `wait_for()` until `self.forwarded_stamp_keys` contains every stamp of `stamp_keys`. `on_validated()` appends to this log the stamp of each sample that arrives on the topic `/diffbot/imu/data_validated`.
- **`advance_clock()`:** adds `STEP` to `self.now` and publishes the new time on the topic `/clock`.

Two more methods run steps. The excerpt shows `step_allowing_no_output()` and `run_steps()`, from [`test_ekf_synthetic.py`, lines 340–351]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L340-L351):

```python title="test_ekf_synthetic.py: step_allowing_no_output() and run_steps()" linenums="340"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:340:351"
```

- **`step_allowing_no_output()`:** is for phase 6 only. It does 1 to 4 with `advance()`, sleeps `NO_OUTPUT_WAIT`, 0.2 s of wall time, and returns the step's last output, or `None` if no output came.
- **`run_steps()`:** runs `count` steps with the same inputs and returns one output for each step.

Every wait is bounded: it ends when its message arrived, or it fails the test after `WAIT`. The waits reduce how much the result depends on the computer's speed. They don't prove that the EKF received each input before the clock moved. The checks of the estimate don't depend on that: the filter fuses an input that arrives late one step later, and these checks look at the last output of a phase and at the highest of the outputs that the test keeps in a phase.

## The seven phases

The test is one scenario on one filter. The excerpt shows the test method, from [`test_ekf_synthetic.py`, lines 366–373]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L366-L373). Each `check_...()` method is one phase, and each phase needs the state that the phases before it left. The method stops at the first phase that fails:

```python title="test_ekf_synthetic.py: the test method" linenums="366"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:366:373"
```

The figure shows the seven phases from left to right, one step every 20 ms of ROS time. The rows "IMU valid" and "IMU to drop" show the samples of one step: ✓ is a valid sample, ✕ is a sample that the validator has to drop. The chart below the rows shows the yaw rate $\omega_z$ that the wheel odometry and the valid IMU samples report, the range that the test accepts for the last output of each phase, the limit for the outputs that the test keeps in phases 2 to 5, and the estimates that the test printed in a run. The three lists below the chart explain each mark and give its numbers.

--8<-- "includes/localization-test-stepped-clock.html"

In every step of every phase, the wheel odometry reports `WHEEL_RATE`, 0.4 rad/s, as with a wheel separation that is too large in the drive controller's configuration. The table lists what changes from phase to phase:

| Phase and its steps | IMU samples of each step | What the test asserts |
|:--|:--|:--|
| 1: `FUSION_STEPS`, 150 | One valid sample, 0.5 rad/s | The last output has the frames `odom` and `base_footprint`, $v_x$ = 0.1 ± 0.002 m/s and $\omega_z$ = 0.5 ± 0.01 rad/s |
| 2: `PHASE_STEPS`, 50 | One sample stamped 0.5 s before the step, 2.0 rad/s | No kept output above 0.51 rad/s. The last output has $\omega_z$ = 0.4 ± 0.02 rad/s |
| 3: 50 | The valid sample, then one with the same stamp, 2.0 rad/s | No kept output above 0.51 rad/s. The last output has $\omega_z$ = 0.5 ± 0.01 rad/s |
| 4: 50 | The valid sample, then one stamped 0.5 s after the step, 2.0 rad/s | As phase 3 |
| 5: 50 | The valid sample, then one stamped 0.01 s before the step, 2.0 rad/s | As phase 3 |
| 6: at most `RESET_STEPS`, 5, then `AFTER_RESET_STEPS`, 100 | The clock goes back to 50 s. Then one valid sample | An output comes within 5 steps. The outputs are stamped before 100 s. The last output has $\omega_z$ = 0.5 ± 0.01 rad/s |
| 7: 75 | None | The validator's counts on the topic `/diagnostics` are the test's own counts |

Two helpers hold the comparisons. The excerpt shows them, from [`test_ekf_synthetic.py`, lines 353–364]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L353-L364). `assert_yaw_rate()` checks one output against `expected` ± `delta`. `assert_stays_at_most()` checks that none of a phase's kept outputs is above `limit`. The test keeps one output per step, the last one that `step()` finds when its wait ends:

```python title="test_ekf_synthetic.py: the two comparisons" linenums="353"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:353:364"
```

### Phase 1: the estimate follows the IMU

The wheels report 0.4 rad/s and the IMU 0.5 rad/s, for 150 steps, 3 s of ROS time. The excerpt shows the phase, from [`test_ekf_synthetic.py`, lines 375–386]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L375-L386):

```python title="test_ekf_synthetic.py: phase 1" linenums="375"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:375:386"
```

- **The frames:** the last output is in the frame `odom` with the child frame `base_footprint`, as [`ekf.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/ekf.yaml) sets them.
- **The forward speed $v_x$:** `SPEED`, 0.1 m/s, within `SPEED_TOLERANCE`, 0.002 m/s. Only the wheel odometry carries a forward speed, so this check shows that the filter fuses the wheel odometry.
- **The yaw rate $\omega_z$:** `IMU_RATE`, 0.5 rad/s, within `YAW_RATE_TOLERANCE`, 0.01 rad/s. The IMU's variance is 75 times smaller than the wheels', so the estimate lies close to the IMU's yaw rate. The wheels' 0.4 rad/s is ten tolerances away, so the check shows that the filter fuses the IMU's samples.

### Phase 2: only samples that are too old

Each step sends one IMU sample that is stamped `STALE_OFFSET`, 0.5 s, before the step's time and reports `BAD_RATE`, 2.0 rad/s. 0.5 s is five times the validator's `max_age`, so the validator has to drop every one of these samples. The excerpt shows the phase, from [`test_ekf_synthetic.py`, lines 388–397]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L388-L397):

```python title="test_ekf_synthetic.py: phase 2" linenums="388"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:388:397"
```

- **No kept output above 0.51 rad/s:** the limit is `IMU_RATE + YAW_RATE_TOLERANCE`, and the test compares the one output that it keeps per step. A filter that fused one of the 2.0 rad/s samples would go far above this limit.
- **The last output:** `WHEEL_RATE`, 0.4 rad/s, within `WHEEL_RATE_TOLERANCE`, 0.02 rad/s. Without valid IMU samples, the wheel odometry is the filter's only source for the yaw rate, as when the IMU stops. The 50 steps, 1 s of ROS time, give the estimate the time to get there.

### Phases 3 to 5: a valid sample and one to drop

In these phases, each step sends `VALID_SAMPLE` first and then a second sample that reports 2.0 rad/s. The excerpt shows the three phases, from [`test_ekf_synthetic.py`, lines 399–423]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L399-L423):

```python title="test_ekf_synthetic.py: phases 3 to 5" linenums="399"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:399:423"
```

- **Phase 3, a repeated stamp:** the second sample has the stamp of the valid one, offset 0. The validator has to drop it as not newer than the last valid sample, check 5 of the [five checks](../../robot_localization.md#the-five-checks).
- **Phase 4, from the future:** the second sample is stamped `FUTURE_OFFSET`, 0.5 s, after the step's time, 25 times the validator's `max_future`. Check 4 has to drop it.
- **Phase 5, out of order:** the second sample is stamped `OLDER_OFFSET`, 0.01 s, before the step's time. This stamp is well within `max_age`, so check 4 passes. Check 5 has to drop the sample, because its stamp is older than the valid sample's of the same step.

The assertions are the same in the three phases. No kept output is above 0.51 rad/s, and the last output is 0.5 rad/s within 0.01 rad/s. So the valid samples keep reaching the filter. The validator's counts in phase 7 show that the validator dropped the 150 samples that report 2.0 rad/s.

### Phase 6: the clock goes back

The test sets the clock back from 107 s to `RESTART`, 50 s, as a reset of Gazebo's world does in the simulation. The validator starts its order check over, and the EKF starts over from the new time, because [`ekf.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/ekf.yaml) sets `reset_on_time_jump`. The excerpt shows the phase, from [`test_ekf_synthetic.py`, lines 425–448]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L425-L448):

```python title="test_ekf_synthetic.py: phase 6" linenums="425"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:425:448"
```

1. The phase sets `self.now` to `RESTART`, publishes the clock and sleeps `RESET_DELIVERY`, 0.5 s of wall time, so that the jump reaches the nodes.
2. The phase runs up to `RESET_STEPS`, 5, steps with `step_allowing_no_output()` and stops at the first step with an output. The filter can stay without an output in the first steps after the jump. The comment in the excerpt says why. No output within 5 steps fails the test.
3. The phase runs `AFTER_RESET_STEPS`, 100, steps with valid samples, 2 s of ROS time.
4. The first of these outputs has to be stamped before `START`, 100 s. So the filter works in the new time.
5. The last output has to be 0.5 rad/s within 0.01 rad/s, as in phase 1.

### Phase 7: the validator's counts

The validator counts every sample that it forwards and every sample that it drops, by reason, and reports the counts on the topic `/diagnostics` once per second of ROS time. The test kept its own counts in `expect()`. Phase 7 compares the two. The excerpt shows the phase, from [`test_ekf_synthetic.py`, lines 450–490]({{ ekf_src_url }}/diffbot_control/test/test_ekf_synthetic.py#L450-L490):

```python title="test_ekf_synthetic.py: phase 7" linenums="450"
--8<-- "{{ ekf_src_raw }}/diffbot_control/test/test_ekf_synthetic.py:450:490"
```

- **`wait_for_validator_diagnostics()`:** notes in `array_count_before` how many messages of the topic `/diagnostics` the test has so far. Then it runs steps without IMU samples until the clock is `DIAGNOSTICS_WAIT`, 1.5 s, further. That is longer than the 1 s between two reports, so a report from after the last IMU sample arrives. The method returns the keys and values of the validator's latest status.
- **`validator_values()`:** looks through the messages that arrived after the first `array_count_before`, the newest first, and finds the validator's status by its name. The name contains `VALIDATOR_STATUS_NAME`, "IMU sample validator". The test writes this text and the keys out and doesn't import them from the script, so the test pins what a user reads on the topic.
- **`assert_diagnostic_counts()`:** the value of `FORWARDED_KEY`, "Samples forwarded", has to be `self.expected_forward_count`. For each reason, the value of "Dropped, " plus the reason has to be the test's count. The last assertion compares the number of values with the number of expected reasons plus one, so the status has no other reason to drop a sample.

## A run

In a run at the commit `3ef4a91` of [diffbot#154](https://github.com/ros-mobile-robots/diffbot/pull/154), the rostest command took 20 s, the test passed, and it printed these lines:

```text
fusion: wheels 0.4 rad/s, IMU 0.5 rad/s, estimate 0.4987 rad/s, forward 0.1000 m/s
stale IMU: estimate from 0.4966 to 0.4005 rad/s
after the clock went back to 50 s: estimate 0.4987 rad/s at 52.02 s
validator diagnostics: {'Samples forwarded': '402', 'Dropped, from the future': '50', 'Dropped, not newer than the last one': '100', 'Dropped, too old': '50'}
```

- **The estimate:** 0.4987 rad/s at the end of phase 1 and of phase 6, and 0.4005 rad/s at the end of phase 2. These are the three dots in the figure.
- **The forwarded samples:** 402 are the 150 of phase 1, the 3 · 50 of phases 3 to 5, and the 102 of phase 6. In this run, the filter published again in the second step after the clock went back, so phase 6 had 2 + 100 steps.
- **The dropped samples:** 50 too old from phase 2, 100 not newer from phases 3 and 5, and 50 from the future from phase 4.

## What the test doesn't show

- **The robot's wheel odometry:** on the robot, the wheel odometry arrives at most 10 times per second. In this test, both inputs arrive in every step, 50 times per second of ROS time.
- **Real sensors:** the inputs have no noise, no bias and no delay. How the real gyroscope behaves is a measurement on the robot, as [Limits](../../robot_localization.md#limits) says.
- **The pose and the transform:** the test checks the forward speed $v_x$ and the yaw rate $\omega_z$ of the filter's output, and its two frames. It doesn't check the position, the heading $\theta$ or the transform `odom → base_footprint` on the topic `/tf`. The [test in Gazebo](gazebo.md) compares the heading with the simulated robot's true turn.
- **Remo's URDF model:** the test loads DiffBot's URDF model, whose `imu_link` has the axes of `base_footprint`. The test in Gazebo runs with both robots' URDF models.
- **The validator's limits themselves:** the samples to drop lie far outside the limits, 0.5 s against 0.1 s and 0.02 s. The [validator's unit tests](validator.md) probe the limits to the microsecond.
