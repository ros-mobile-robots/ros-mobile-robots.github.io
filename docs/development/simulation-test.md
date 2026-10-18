# Simulation Test

[`diffbot_gazebo`]({{ diffbot_repo_url }}/diffbot_gazebo) has a simulation test: it drives DiffBot and Remo in [Gazebo Classic 11](https://classic.gazebosim.org/), the simulator that ROS Noetic uses, and checks where they went. Both [CI workflows](ci.md#diffbot) run it on every pull request ([diffbot#110](https://github.com/ros-mobile-robots/diffbot/issues/110)). It catches mistakes in the robot model and the controller configuration, which the simulation shares with the real robot:

- **Left and right wheel joints swapped:** the robot turns right when it's told to turn left.
- **A wheel radius in the controller configuration that doesn't match the robot model:** the robot's odometry reports a different distance than the robot drove.
- **A wrong joint name:** the controller doesn't start, and nothing is published.

You need the [dev container](dev-container.md) with its workspace `~/catkin_ws` built and sourced, as the container's setup leaves it. This page first shows how to run the test and read its results, then how it works and its code step by step, then why the results differ from the commands, and finally how to add a test of your own.

## Run it

In the dev container, in `~/catkin_ws`, run all tests of the workspace as CI does ([dev container workflow](ci.md#dev-container-workflow)):

```console
catkin build --catkin-make-args run_tests
catkin_test_results build
```

The first command builds the workspace and runs every registered test, but it doesn't fail when a test fails. The second one sums up the result files and fails if any test failed.

To run the test for one robot, use its test file, `sim_drive_diffbot.test` or `sim_drive_remo.test`:

```console
rostest diffbot_gazebo sim_drive_diffbot.test
```

With `--text`, rostest prints the nodes' output in the terminal instead of log files, but it doesn't collect the test results then: it can report success although a check failed. Use `--text` to watch a run, and the command without it to check the result.

## Read the results

rostest ends with a summary. A passing run of `sim_drive_diffbot.test`:

```text
[diffbot_gazebo.rosunit-drive_diffbot/test_forward][passed]
[diffbot_gazebo.rosunit-drive_diffbot/test_turn][passed]
[diffbot_gazebo.rosunit-publish_diffbot/test_publish][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 3
 * ERRORS: 0
 * FAILURES: 0
```

The test file runs two test nodes ([step 1][s1]): `drive_diffbot`, which has the two test methods `test_forward` and `test_turn`, and `publish_diffbot`, which has one test. A failed check shows `[FAILURE]` instead of `[passed]`, with the check's message, which names the measured value, the expected one and the tolerance; the result is then `FAIL`. An error means that a test couldn't get to its checks, for example because no odometry arrived.

`catkin_test_results build` counts the tests in all result files. For this test, it reports 10: for each robot, rostest's own result file lists the two test nodes, and the nodes' files list their two drive tests and the one publish test.

**The measured motions:** `measure_motion()` logs both measurements of each drive, and rostest copies the node's log into its result file `rosunit-drive_diffbot.xml`. The file is in `~/.ros/test_results/diffbot_gazebo/` when you run rostest yourself, and in `build/diffbot_gazebo/test_results/diffbot_gazebo/` when catkin runs the tests:

```console
grep -o '\(odometry\|true pose\): [^<&]*' ~/.ros/test_results/diffbot_gazebo/rosunit-drive_diffbot.xml
```

```text
odometry: forward 0.580 m, left 0.001 m, turned 0.005 rad
true pose: forward 0.590 m, left 0.001 m, turned 0.004 rad
odometry: forward 0.006 m, left 0.002 m, turned 1.370 rad
true pose: forward 0.019 m, left -0.016 m, turned 1.263 rad
```

Each drive is measured twice: with the robot's odometry and with its true pose in Gazebo ([How it works](#how-it-works)). The first two lines are `test_forward`, which commands 0.6 m straight ahead; the last two are `test_turn`, which commands 1.5 rad on the spot. `forward`, `left` and `turned` describe the motion as seen from where the robot started ([`relative_pose()`][s-rel]). The numbers aren't exactly the commanded ones: the checks accept a range around each expected value ([the checks][s-checks]), and [Why the results differ from the commands](#why-the-results-differ-from-the-commands) explains where the differences come from.

## How it works

### What runs in the simulation

The test file starts Gazebo, the robot and its controllers, and then the test nodes. The first tab of the figure shows them during the test; the second tab shows the same controllers on the real robot ([On the real robot](#on-the-real-robot)).

--8<-- "includes/simulation-test-figures.html"

Topic names are shortened in the figure: `cmd_vel` and `odom` are under `/diffbot/mobile_base_controller/`, `joint_states`, `wheel_cmd_velocities` and `measured_joint_states` under `/diffbot/`, and `model_states` under `/gazebo/`. The figure shows both test nodes, although rostest runs them one after another ([step 1][s1]).

The robot's controllers come from [ros_control](http://wiki.ros.org/ros_control). ros_control isn't a node of its own: it's a set of C++ libraries, used inside the process that has access to the joints, here the Gazebo server. The robot's URDF (Unified Robot Description Format), the XML file that describes its links and joints, loads the [`gazebo_ros_control`](http://wiki.ros.org/gazebo_ros_control) plugin ([`robot.gazebo.xacro`]({{ diffbot_repo_url }}/diffbot_description/urdf/include/robot.gazebo.xacro)). The plugin creates the hardware interface and the controller manager, and the controller manager loads the controllers:

- The **hardware interface**, `DefaultRobotHWSim`, gives the controllers each wheel joint's position and velocity, and takes a velocity command for each wheel. It reads the values from the simulated joints, for the wheel joints that have a transmission in the URDF, and drives each joint to its commanded velocity with a PID controller, with the gains from `pid.yaml`.
- The **controller manager** updates the controllers. In each simulation step, the plugin reads the joints (`readSim()`), lets the controller manager update the controllers, and applies the commands (`writeSim()`).
- The **controllers** read and write the hardware interface's values directly, in the same process, not over topics. `joint_state_controller` copies the joint values to `joint_states`. `mobile_base_controller`, a [diff_drive_controller](http://wiki.ros.org/diff_drive_controller), turns the velocities from `cmd_vel` into a velocity for each wheel, and computes the odometry from the wheel joint positions. [Robot Motion](../theory/state-estimation/robot-motion.md#in-diff_drive_controller) shows how: with the velocity motion model of *Probabilistic Robotics*.

### Three views of the motion

The test compares what the robot should do with what it did, seen in two ways:

| View | Topic | Published by | What it is |
|:--|:--|:--|:--|
| The command | `/diffbot/mobile_base_controller/cmd_vel` | `test_drive.py` | A linear and an angular velocity. Sent for 3 s, they give the expected motion, for example 0.2 m/s for 3 s is 0.6 m |
| The odometry | `/diffbot/mobile_base_controller/odom` | diff_drive_controller | The pose that the controller computes from the wheel angles, with the wheel radius and separation from [`diffbot_control.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/diffbot_control.yaml) |
| The true pose | `/gazebo/model_states` | Gazebo's ROS interface, not ros_control | The pose of every model in Gazebo's world, from the physics simulation |

`/diffbot/joint_states`, published by `joint_state_controller`, has each wheel's angle (rad) and velocity (rad/s). `test_drive.py` doesn't use it: to get a pose from the wheel angles, it would have to repeat the controller's calculation, with the same wheel radius and separation, and so it would repeat the same configuration mistakes.

The true pose comes from the physics instead, so it shows where the simulated robot really went, including the slip of the wheels. That's how the test catches swapped wheel joints. The controller sends the left wheel's command to the right wheel, so the robot turns right. But it also reads the right wheel's angle as the left one's, so its odometry still reports the commanded left turn.

"True" means the pose of the robot model in Gazebo, not of the real robot. A passing test shows that the robot model and the controller configuration work together in the simulation; it doesn't calibrate the real robot.

### On the real robot

Switch the figure to the second tab, *Real robot*. The blue box stays the same: the controller manager with the two controllers, configured by `diffbot_control.yaml`. Only the hardware interface and what's below it change.

On the real robot, the [`diffbot_base`]({{ diffbot_repo_url }}/diffbot_base/src/diffbot_base.cpp) node takes Gazebo's place. It creates the hardware interface [`DiffBotHWInterface`]({{ diffbot_repo_url }}/diffbot_base/src/diffbot_hw_interface.cpp) and a controller manager, and runs the loop itself: `read()`, update the controllers, `write()`. Its [launch file]({{ diffbot_repo_url }}/diffbot_base/launch/diffbot.launch) loads the same `diffbot_control.yaml` and spawns the same two controllers. This hardware interface talks to the Teensy over ROS topics:

- It subscribes to `measured_joint_states`, the wheel angles and velocities that the Teensy publishes, and `read()` passes the latest values to the controllers.
- `write()` publishes the wheel velocity commands on `wheel_cmd_velocities`, and the Teensy's firmware runs a PID loop per wheel to reach them ([Base Hardware Interface](../packages/diffbot_base/index.md)).
- The [rosserial](http://wiki.ros.org/rosserial) node relays both topics over USB.

So the simulation test covers the robot model and the controller configuration that both share. It doesn't run `diffbot_base`, rosserial or the firmware; tests for these without the robot are planned in [diffbot#114](https://github.com/ros-mobile-robots/diffbot/issues/114). [ROS Control, An overview](https://fjp.at/posts/ros/ros-control/) explains ros_control in more detail.

## Code walkthrough

Two levels run the test. rostest runs the test file's test nodes one after another, each with its own launch ([step 1][s1]). Inside the drive test node, unittest runs `setUpClass()` once and then the test methods ([steps 3][s3] to [5][s5]).

`test_forward()` calls `measure_motion()`, which calls `drive()`, `stop()` and, once for the odometry and once for the true pose, `relative_pose()`. Then `test_forward()` checks the two motions it got back. While this runs, the subscriber callbacks keep the latest odometry pose and true pose up to date in the background. `test_turn()` uses the same `measure_motion()`.

The code blocks show the files at the diffbot commit that this page pins, with their comments, and the line numbers are the files'. The link above each block opens the same lines on GitHub.

### 1. The test file

A `.test` file is a launch file for rostest. [`sim_drive_diffbot.test`]({{ diffbot_src_url }}/diffbot_gazebo/test/sim_drive_diffbot.test):

```xml title="sim_drive_diffbot.test"
--8<-- "{{ diffbot_src_raw }}/diffbot_gazebo/test/sim_drive_diffbot.test"
```

rostest turns each `<test>` element into a test of its own and runs them one after another, in alphabetical order of their names: first `drive_diffbot`, then `publish_diffbot`. For each one, it starts a ROS master, launches everything in the file that isn't a test node, here `sim_drive.launch` ([step 2][s2]), and runs the test node until it ends. Then it reads the node's results and stops everything again ([`runner.py`](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rostest/src/rostest/runner.py)). So each test node gets a freshly started Gazebo, and the two don't affect each other. `time-limit` is how long rostest waits for a node before it counts the test as failed.

`sim_drive_remo.test` differs only in the robot model, the port and the test names.

### 2. The launch file

[`sim_drive.launch`]({{ diffbot_src_url }}/diffbot_gazebo/test/sim_drive.launch) starts the robot in Gazebo and its controllers:

```xml title="sim_drive.launch"
--8<-- "{{ diffbot_src_raw }}/diffbot_gazebo/test/sim_drive.launch"
```

- [`diffbot_view.launch`]({{ diffbot_repo_url }}/diffbot_gazebo/launch/diffbot_view.launch) starts the Gazebo server on the [empty world]({{ diffbot_repo_url }}/diffbot_gazebo/worlds/diffbot_empty.world), which has a ground plane for the robot to stand on and a light. It runs with simulation time: Gazebo publishes its clock, and all nodes use it instead of the computer's clock (`use_sim_time`). On a slow computer the test takes longer, but the robot drives the same distance. Then `spawn_model` adds the robot from its URDF to the world, at x = −2 m, y = −0.5 m, while the simulation is already running.
- [`diffbot_control.launch`]({{ diffbot_repo_url }}/diffbot_control/launch/diffbot_control.launch) loads `diffbot_control.yaml`, the controller configuration that the real robot uses too, and `pid.yaml`, the gains of Gazebo's wheel velocity controllers. Its `controller_spawner` asks the controller manager to load and start the two controllers. It also starts `robot_state_publisher`, which the test doesn't use. `rqt_robot_steering`, a window with sliders to drive the robot, stays off, because it needs a display.

[How it works](#how-it-works) shows how these parts work together.

### 3. Connecting to the robot

rostest starts [`test_drive.py`]({{ diffbot_src_url }}/diffbot_gazebo/test/test_drive.py) as a ROS node. Its last line, `rostest.rosrun('diffbot_gazebo', 'test_drive', TestDrive)`, runs the [unittest](https://docs.python.org/3/library/unittest.html) class `TestDrive` and writes the results to a file that rostest reads. unittest calls `setUpClass()` once, then the test methods. [`test_drive.py`, lines 113–133]({{ diffbot_src_url }}/diffbot_gazebo/test/test_drive.py#L113-L133):

```python title="test_drive.py: TestDrive.setUpClass()" linenums="113"
--8<-- "{{ diffbot_src_raw }}/diffbot_gazebo/test/test_drive.py:113:133"
```

- `rospy.init_node()` registers the node with the ROS master. The constants at the top of the file, such as `CMD_VEL_TOPIC`, hold the full topic names.
- The publisher sends velocity commands, `Twist` messages, to the controller.
- The two subscribers call `on_odom()` for each odometry message, and `on_model_states()` for each `model_states` message. Gazebo publishes `model_states` in every simulation step, so `queue_size=1` keeps only the newest message instead of falling behind.
- `rospy.wait_for_message()` waits for the first message on a topic. If none comes within the timeout, it raises an exception and the tests fail, for example when the controller doesn't start because of a wrong joint name.

The callbacks, [lines 135–145]({{ diffbot_src_url }}/diffbot_gazebo/test/test_drive.py#L135-L145):

```python title="test_drive.py: the subscriber callbacks" linenums="135"
--8<-- "{{ diffbot_src_raw }}/diffbot_gazebo/test/test_drive.py:135:145"
```

rospy calls them in background threads whenever a message arrives, so `odom_pose` and `true_pose` always hold the latest pose. `msg.name.index(MODEL_NAME)` finds the robot's position in the list of model names, and its pose is at the same position in `msg.pose`.

Both convert the ROS pose to a [`Pose2D`]({{ diffbot_src_url }}/diffbot_gazebo/test/test_drive.py#L47-L52), a [named tuple](https://docs.python.org/3/library/typing.html#typing.NamedTuple) with $x$, $y$ and the heading $\theta$, as on [Robot Representations](../theory/modeling-control/robot-representations.md#robot-pose). [Lines 69–78]({{ diffbot_src_url }}/diffbot_gazebo/test/test_drive.py#L69-L78):

```python title="test_drive.py: to_pose2d()" linenums="69"
--8<-- "{{ diffbot_src_raw }}/diffbot_gazebo/test/test_drive.py:69:78"
```

A [quaternion](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation) has four numbers that describe any rotation in 3D, without the special cases of three angles ([tf2 quaternion tutorial](http://wiki.ros.org/tf2/Tutorials/Quaternions)). ROS's axes and angles follow [REP 103](https://www.ros.org/reps/rep-0103.html): x forward, y to the left, z up, and the yaw grows when the robot turns to the left.

### 4. `test_forward()`

The constants at the top of the file name the commands, [lines 36–44]({{ diffbot_src_url }}/diffbot_gazebo/test/test_drive.py#L36-L44):

```python title="test_drive.py: the commands" linenums="36"
--8<-- "{{ diffbot_src_raw }}/diffbot_gazebo/test/test_drive.py:36:44"
```

`test_forward()`, with the comment before the two tests, [lines 179–198]({{ diffbot_src_url }}/diffbot_gazebo/test/test_drive.py#L179-L198):

```python title="test_drive.py: test_forward()" linenums="179"
--8<-- "{{ diffbot_src_raw }}/diffbot_gazebo/test/test_drive.py:179:198"
```

It has two parts. `measure_motion()` drives the robot at 0.2 m/s for 3 s without turning, and returns how the robot moved, measured twice: `odom_motion` with the odometry, and `true_motion` with the true pose. Each one is a `RelativePose`, with how far the robot moved forward and to the left, and how much it turned ([`relative_pose()`][s-rel]). The rest are the checks ([The checks][s-checks]). The call names its arguments, so it reads without looking up `measure_motion()`.

#### `measure_motion()`

[Lines 164–177]({{ diffbot_src_url }}/diffbot_gazebo/test/test_drive.py#L164-L177):

```python title="test_drive.py: measure_motion()" linenums="164"
--8<-- "{{ diffbot_src_raw }}/diffbot_gazebo/test/test_drive.py:164:177"
```

It keeps the latest odometry pose and true pose as the start poses, drives ([`drive()`][s-drive]), stops ([`stop()`][s-stop]), and computes, for each source, where the robot ended up relative to where it started ([`relative_pose()`][s-rel]). The logged lines are the ones in [Read the results](#read-the-results).

#### `drive()`

[Lines 147–157]({{ diffbot_src_url }}/diffbot_gazebo/test/test_drive.py#L147-L157):

```python title="test_drive.py: drive()" linenums="147"
--8<-- "{{ diffbot_src_raw }}/diffbot_gazebo/test/test_drive.py:147:157"
```

- A `Twist` holds a linear and an angular velocity, each as a 3D vector. A differential drive robot can only drive along its x axis, `linear.x` in m/s, and turn about its z axis, `angular.z` in rad/s, positive to the left. The controller ignores the other values.
- `rospy.Time.now()` and `rospy.Rate` follow Gazebo's clock ([step 2][s2]), so `duration` and `COMMAND_RATE` are in simulation time. Why the command is sent 20 times per second is in the comment above `COMMAND_RATE`.

#### `stop()`

[Lines 159–162]({{ diffbot_src_url }}/diffbot_gazebo/test/test_drive.py#L159-L162):

```python title="test_drive.py: stop()" linenums="159"
--8<-- "{{ diffbot_src_raw }}/diffbot_gazebo/test/test_drive.py:159:162"
```

`Twist()` is a command with all velocities 0. The controller brakes as gently as it speeds up: from 0.2 m/s at 0.6 m/s², the robot needs 0.33 s to stop. One second of simulation time is enough for the robot to stand still, and for the odometry and Gazebo to publish its end pose.

#### `relative_pose()`

The odometry and Gazebo give poses in different frames, so the test compares the motion between two poses instead of the poses: the end pose relative to the start pose, in the robot's frame at the start.

--8<-- "includes/simulation-test-relative-pose.html"

With the displacement $\Delta x = x_1 - x_0$ and $\Delta y = y_1 - y_0$ in the frame of the poses, rotating it by $-\theta_0$ gives it in the robot's frame at the start. A robot's x axis points forward and its y axis to the left ([REP 103](https://www.ros.org/reps/rep-0103.html)), which gives the two parts their names:

$$
\begin{bmatrix} \text{forward} \\ \text{left} \end{bmatrix}
= \begin{bmatrix} \cos\theta_0 & \sin\theta_0 \\ -\sin\theta_0 & \cos\theta_0 \end{bmatrix}
\begin{bmatrix} \Delta x \\ \Delta y \end{bmatrix}
$$

and the angle it turned is $\text{turned} = \theta_1 - \theta_0$.

The matrix is the transpose of the rotation matrix $R(\theta_0)$, which turns a vector from the robot's frame into the frame of the poses; the transpose turns it back. This is the relative pose, a standard operation on poses: with homogeneous transforms, it is $T_0^{-1} T_1$ (Peter Corke, *Robotics, Vision and Control*, chapter 2, [Amazon.com][amazon_book_corke_rvc_us], [Amazon.de][amazon_book_corke_rvc_ger], affiliate links). Moving or rotating the frame of the poses changes both poses the same way, so the relative pose stays the same, and the motion from the odometry and from the true pose can be compared.

`RelativePose` holds the three numbers, [lines 55–66]({{ diffbot_src_url }}/diffbot_gazebo/test/test_drive.py#L55-L66):

```python title="test_drive.py: RelativePose" linenums="55"
--8<-- "{{ diffbot_src_raw }}/diffbot_gazebo/test/test_drive.py:55:66"
```

`relative_pose()` computes them, with `wrap()` for the angle, [lines 81–110]({{ diffbot_src_url }}/diffbot_gazebo/test/test_drive.py#L81-L110):

```python title="test_drive.py: wrap() and relative_pose()" linenums="81"
--8<-- "{{ diffbot_src_raw }}/diffbot_gazebo/test/test_drive.py:81:110"
```

`forward` and `left` are the two rows of the matrix. [atan2](https://en.wikipedia.org/wiki/Atan2)$(\sin\alpha, \cos\alpha)$ in `wrap()` gives the angle $\alpha$ back in $-\pi$ to $\pi$, using the signs of both values.

*Probabilistic Robotics* describes the motion between two poses differently, as a rotation, a translation and a rotation: the odometry motion model, which filters such as amcl use to add noise to each part ([Robot Motion](../theory/state-estimation/robot-motion.md#odometry-motion-model)). The test needs no noise, and it checks `forward`, `left` and `turned` directly, so it uses the relative pose.

#### The checks

`test_forward()` expects the robot to drive `FORWARD_VELOCITY * DURATION`, 0.2 m/s for 3 s, so 0.6 m straight ahead. This is the simplest case of the velocity motion model, $v \cdot t$ with no turn ([Robot Motion](../theory/state-estimation/robot-motion.md#velocity-motion-model)).

`assertAlmostEqual(measured, expected, delta=d)` passes when the two differ by at most `d`. Each tolerance lies between what the robots measure in Gazebo and what a mistake would give. The loop checks the odometry and the true pose the same way, and the last check compares them:

| Check | Expected | Measured | Fails when |
|:--|:--|:--|:--|
| `forward` | 0.6 ± 0.1 m | 0.57 to 0.62 m | The robot doesn't move, or drives much too slow or too fast |
| `left` | 0 ± 0.1 m | About 1 mm | It drives off to the side |
| `turned` | 0 ± 0.2 rad | 0.005 rad or less | Its heading changes, for example because one wheel turns slower |
| Odometry and true `forward` | Within 0.05 m | About 0.01 m apart | The controller's wheel radius doesn't match the model: 20 % too big gave 0.09 m |

[Why the results differ from the commands](#why-the-results-differ-from-the-commands) explains why the robots drive 0.57 to 0.62 m instead of 0.6 m.

### 5. `test_turn()`

[Lines 200–215]({{ diffbot_src_url }}/diffbot_gazebo/test/test_drive.py#L200-L215). The highlighted lines are the ones that differ from `test_forward()`:

```python title="test_drive.py: test_turn()" linenums="200" hl_lines="3 9 11 13 16"
--8<-- "{{ diffbot_src_raw }}/diffbot_gazebo/test/test_drive.py:200:215"
```

It commands the angular velocity instead of the linear one and expects 0.5 rad/s for 3 s, so 1.5 rad to the left; the controller's angular acceleration limit of 6 rad/s² hardly matters. On the spot means that the robot's center stays where it was. [`math.hypot`](https://docs.python.org/3/library/math.html#math.hypot)`(forward, left)` is its distance from the start, the length of the hypotenuse of a right triangle with these two sides. The test starts wherever `test_forward()` left the robot; that doesn't matter, because it checks only the motion from there.

| Check | Expected | Measured | Fails when |
|:--|:--|:--|:--|
| `turned` | 1.5 ± 1.0 rad | DiffBot about 1.3 rad, Remo 0.92 to 1.34 rad | The robot turns right, for example with swapped wheel joints, or doesn't turn |
| Distance from the start | 0 ± 0.15 m | Odometry below 0.01 m, true pose 0.02 to 0.04 m | It drives off instead of turning on the spot |
| Odometry and true `turned` | Within 0.25 rad | 0.02 to 0.11 rad apart | The wheel radius doesn't match the model: 20 % too big gave up to 0.31 rad |

The robots turn clearly less than 1.5 rad, and Remo's turn varies between runs, so the tolerance is wide until the simulation's wheel controllers are tuned ([Why the results differ from the commands](#why-the-results-differ-from-the-commands)).

### 6. `publishtest`

When `test_drive.py` has ended, rostest stops everything and starts `sim_drive.launch` again for the second test node, `publish_diffbot` ([step 1][s1]). [`publishtest`](http://wiki.ros.org/rostest/Nodes#publishtest) checks the topics in its parameters in the test file: `joint_states` and the odometry must each get a message within 90 s. So it checks that both controllers started; `test_drive.py` doesn't use `joint_states`.

## Why the results differ from the commands

The commands give an ideal expectation: 0.6 m straight ahead, 1.5 rad on the spot. The robots in Gazebo come close, but not exactly, and the odometry and the true pose differ a little. Three questions explain the numbers in [Read the results](#read-the-results) and in the check tables.

### Why the measured motion differs from the command

- **Timing:** `drive()` sends the command for 3 s of simulation time, but the last command stays in effect until `stop()`, nominally up to 50 ms longer: 1 cm or 0.025 rad more (the comment above `COMMAND_RATE`).
- **Acceleration and braking:** diff_drive_controller limits the acceleration (`diffbot_control.yaml`: 0.6 m/s² and 6 rad/s²), and it limits braking the same way, because `min_acceleration` defaults to the negative maximum. Speeding up to 0.2 m/s takes 0.33 s and covers 3.3 cm less than an instant start would. Braking after the zero command takes just as long and covers the same 3.3 cm more. `measure_motion()` measures after `stop()` has waited until the robot stands still, so the two cancel. The angular limit hardly matters for the turn.
- **Wheel speed:** in the simulation, each wheel's velocity is controlled by `DefaultRobotHWSim`'s PID controller with the gains from `pid.yaml`, which has only a P gain. A controller with only a P gain can leave a speed error, especially under load. This is clearly visible when Remo turns on the spot: its wheels reach only about 80 % of the commanded speed, and their speed jitters ([diffbot#116](https://github.com/ros-mobile-robots/diffbot/issues/116)). DiffBot's odometry, which counts the wheel turns, shows 0.57 to 0.59 m instead of 0.6 m, and 1.37 rad instead of 1.5 rad. Slower wheels can contribute to that, as can what happens while the robot starts and stops, but the test doesn't measure these effects separately.

### Why the odometry and the true pose differ

The odometry only knows how far the wheels turned, and converts that with the wheel radius and separation from `diffbot_control.yaml`. The true pose comes from the simulated robot. Two things make them differ:

- **The configured geometry doesn't match the URDF.** Both robots use `diffbot_control.yaml`, with a wheel radius of 0.0325 m and a wheel separation of 0.145 m. The URDFs have a wheel radius of 0.0331 m, and a separation of 0.135 m for DiffBot and 0.139 m for Remo. The simulated wheel rolls 1.8 % farther per turn than the odometry assumes, so the odometry's 0.580 m become 0.590 m.
- **The odometry can't see slip.** With the configured geometry, DiffBot's wheel turns give 1.37 rad, which is what its odometry reports. With the URDF's geometry, the same wheel turns would turn the robot about 1.50 rad, but the simulated robot turned only 1.26 rad: about 16 % of the wheels' motion slipped. When the robot drives straight, the wheels hardly slip.

The checks that compare the odometry and the true pose allow these differences, but not those of a wheel radius that is 20 % off.

### Why the robots and repeated runs differ

- **The models differ a lot.** Both robots share the controller configuration and the wheel controller gains, but DiffBot's wheels weigh 0.5 kg each and use Gazebo's default contact settings. Remo's wheels weigh 32 g, with very stiff contacts and no contact damping (`robot.gazebo.xacro` in [remo_description]({{ remo_repo_url }}/urdf/include/robot.gazebo.xacro)).
- **The variation differs a lot.** Over four runs, DiffBot's turn varied by about 0.01 rad, Remo's between 0.92 and 1.34 rad.
- **A likely reason, not yet proven:** Gazebo's physics computes the same result from the same inputs, but the test node's commands reach the simulation asynchronously, a few simulation steps earlier or later from run to run. Remo's light model with stiff contacts would turn such small differences into visibly different results. A run of ROS and Gazebo isn't guaranteed to be repeatable step for step.

[diffbot#118](https://github.com/ros-mobile-robots/diffbot/issues/118) matches the geometry, masses and contact settings to each robot, and [diffbot#116](https://github.com/ros-mobile-robots/diffbot/issues/116) then tunes the wheel controllers and narrows the turn's tolerance.

## How to add a test

As an example, this adds a test that drives backward. It runs for both robots, because both test files run `test_drive.py`.

1. **Write the test method.** Add it to the `TestDrive` class in [`test_drive.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_drive.py), next to `test_forward()`. The highlighted lines are the ones that differ from `test_forward()`:

    ```python title="test_backward() (example)" hl_lines="3 4"
    def test_backward(self) -> None:
        odom_motion, true_motion = self.measure_motion(
            linear_velocity=-FORWARD_VELOCITY, angular_velocity=0.0, duration=DURATION)
        expected_forward = -FORWARD_VELOCITY * DURATION
        for motion in (odom_motion, true_motion):
            self.assertAlmostEqual(motion.forward, expected_forward, delta=0.1)
            self.assertAlmostEqual(motion.left, 0.0, delta=0.1)
            self.assertAlmostEqual(motion.turned, 0.0, delta=0.2)
        self.assertAlmostEqual(odom_motion.forward, true_motion.forward, delta=0.05)
    ```

    A negative linear velocity drives backward, so `forward` is negative: −0.6 m are expected. The tolerances are copied from `test_forward()` for a start. The method's name must start with `test`, so that unittest runs it. unittest runs it first, before `test_forward()`, because of the alphabetical order; that's fine, because each test checks only its own motion.

2. **Run it** for both robots, as in [Run it](#run-it):

    ```console
    rostest diffbot_gazebo sim_drive_diffbot.test
    rostest diffbot_gazebo sim_drive_remo.test
    ```

    The summary now lists `test_backward` too, for example `[diffbot_gazebo.rosunit-drive_diffbot/test_backward][passed]`.

3. **Look at the measured values** in the result file, as in [Read the results](#read-the-results). The first two lines are now `test_backward()`'s motions. In our run, DiffBot drove −0.57 m by the odometry and −0.58 m by the true pose, and turned 0.1 rad to the right on the way, more than when it drives forward. Remo drove −0.58 and −0.59 m, almost without turning.

4. **Choose the tolerances.** Each tolerance should lie between what both robots measure and what a mistake would give. To see what a mistake gives, make one on purpose: for example, make `wheel_radius` in [`diffbot_control.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/diffbot_control.yaml) 20 % bigger, run the test again, look at the values, and undo the change. Then set each tolerance between the two, and write in a comment what you measured.

5. **Keep the file's style:** [PEP 8](https://peps.python.org/pep-0008/) with ROS 2's line length of 99 characters ([ament_flake8's settings](https://github.com/ament/ament_lint/blob/rolling/ament_flake8/ament_flake8/configuration/ament_flake8.ini)), [type hints](https://docs.python.org/3/library/typing.html), and named constants instead of bare numbers in the commands.

A test that needs another world or robot needs its own `.test` file. Register it with `add_rostest` in the package's [`CMakeLists.txt`]({{ diffbot_repo_url }}/diffbot_gazebo/CMakeLists.txt), and give it its own `gazebo_port` (the comment in [`sim_drive.launch`][s2] explains why).

[s1]: #1-the-test-file
[s2]: #2-the-launch-file
[s3]: #3-connecting-to-the-robot
[s5]: #5-test_turn
[s-drive]: #drive
[s-stop]: #stop
[s-rel]: #relative_pose
[s-checks]: #the-checks
