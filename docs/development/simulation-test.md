# Simulation Test

[`diffbot_gazebo`]({{ diffbot_repo_url }}/diffbot_gazebo) has a simulation test: it drives DiffBot and Remo in Gazebo and checks where they went. Both [CI workflows](ci.md#diffbot) run it on every pull request ([diffbot#110](https://github.com/ros-mobile-robots/diffbot/issues/110)). It catches mistakes in the robot model and the controller configuration, which the simulation shares with the real robot:

- **Left and right wheel joints swapped:** the robot turns right when it's told to turn left.
- **A wheel radius in the controller configuration that doesn't match the robot model:** the robot's odometry reports a different distance than the robot drove.
- **A wrong joint name:** the controller doesn't start, and nothing is published.

This page follows one run of the test, in the order things happen. rostest starts Gazebo and the controllers. `test_drive.py` drives the robot forward, then turns it on the spot, and checks each motion twice: with the robot's odometry and with its true pose in Gazebo. Then the page explains the two motion models behind the numbers, what is different on the real robot, how to run the test, and how to add a check.

## A test run, step by step

The test consists of four files in [`diffbot_gazebo/test`]({{ diffbot_repo_url }}/diffbot_gazebo/test):

| File | What it does |
|:--|:--|
| [`sim_drive_diffbot.test`]({{ diffbot_repo_url }}/diffbot_gazebo/test/sim_drive_diffbot.test), [`sim_drive_remo.test`]({{ diffbot_repo_url }}/diffbot_gazebo/test/sim_drive_remo.test) | One [rostest](http://wiki.ros.org/rostest) file per robot: what to launch, and which test nodes to run |
| [`sim_drive.launch`]({{ diffbot_repo_url }}/diffbot_gazebo/test/sim_drive.launch) | Starts Gazebo with the robot, and the robot's controllers |
| [`test_drive.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_drive.py) | Drives the robot and checks where it went |

In the package's [`CMakeLists.txt`]({{ diffbot_repo_url }}/diffbot_gazebo/CMakeLists.txt), `add_rostest` registers each `.test` file, so `catkin build --catkin-make-args run_tests` runs it. The steps follow `sim_drive_diffbot.test`; Remo's file differs only in the robot model, the port and the test names. The code excerpts leave out the comments and docstrings that the files have.

### 1. rostest reads the test file

A `.test` file is a launch file for rostest. `sim_drive_diffbot.test`:

```xml
<launch>
    <include file="$(find diffbot_gazebo)/test/sim_drive.launch">
        <arg name="model" value="diffbot"/>
        <arg name="gazebo_port" value="11351"/>
    </include>

    <test test-name="publish_diffbot" pkg="rostest" type="publishtest" time-limit="120">
        <rosparam>
            topics:
              - name: /diffbot/joint_states
                timeout: 90
                negative: False
              - name: /diffbot/mobile_base_controller/odom
                timeout: 90
                negative: False
        </rosparam>
    </test>

    <test test-name="drive_diffbot" pkg="diffbot_gazebo" type="test_drive.py" time-limit="180"/>
</launch>
```

- `<include>` adds `sim_drive.launch` (step 2) with two arguments: the robot model, and `gazebo_port`, a port of its own for this file's Gazebo server. catkin runs the test files in parallel, and two Gazebo servers on the same port can't start.
- Each `<test>` element is a test node: `publish_diffbot` runs `publishtest` (step 10), and `drive_diffbot` runs `test_drive.py` (steps 3 to 9). `time-limit` is how long rostest waits for the node before it counts the test as failed.

rostest turns each test node into a test of its own and runs them one after another, in alphabetical order of their names: first `drive_diffbot`, then `publish_diffbot`. For each one, it starts a ROS master, launches everything in the file that isn't a test node, here `sim_drive.launch`, and runs the test node until it ends. Then it reads the node's results and stops everything again ([`runner.py`](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rostest/src/rostest/runner.py)). So each test node gets a freshly started Gazebo, and the two don't affect each other.

### 2. sim_drive.launch starts Gazebo and the controllers

```xml
<launch>
    <arg name="model" doc="model type [diffbot, remo]"/>
    <arg name="gazebo_port" doc="port of this test's Gazebo server"/>
    <env name="GAZEBO_MASTER_URI" value="http://localhost:$(arg gazebo_port)"/>

    <include file="$(find diffbot_gazebo)/launch/diffbot_view.launch">
        <arg name="model" value="$(arg model)"/>
        <arg name="gui" value="false"/>
        <arg name="headless" value="true"/>
        <arg name="world_name" value="$(find diffbot_gazebo)/worlds/diffbot_empty.world"/>
    </include>

    <include file="$(find diffbot_control)/launch/diffbot_control.launch">
        <arg name="model" value="$(arg model)"/>
        <arg name="rqt_robot_steering" value="false"/>
    </include>
</launch>
```

- `GAZEBO_MASTER_URI` tells the Gazebo server which port to use.
- [`diffbot_view.launch`]({{ diffbot_repo_url }}/diffbot_gazebo/launch/diffbot_view.launch) starts the Gazebo server:
    - Without the GUI window (`gui`) and without rendering (`headless`), so it needs no display.
    - On the [empty world]({{ diffbot_repo_url }}/diffbot_gazebo/worlds/diffbot_empty.world): a ground plane for the robot to stand on, and a light.
    - With simulation time: Gazebo publishes its clock, and all nodes use it instead of the computer's clock (`use_sim_time`). On a slow computer the test takes longer, but the robot drives the same distance.

    Then `spawn_model` adds the robot from its URDF to the world, at x = −2 m, y = −0.5 m. The simulation is already running at that point, it isn't paused.

- [`diffbot_control.launch`]({{ diffbot_repo_url }}/diffbot_control/launch/diffbot_control.launch) loads two configuration files: [`diffbot_control.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/diffbot_control.yaml), the controller configuration that the real robot uses too, and `pid.yaml`, the gains of Gazebo's wheel velocity controllers. Its `controller_spawner` asks the controller manager to load and start the two controllers. It also starts `robot_state_publisher`, which the test doesn't use. `rqt_robot_steering`, a window with sliders to drive the robot, stays off: it needs a display, and the test sends the commands itself.

Now Gazebo, the robot and its controllers run as in the first tab of the figure:

--8<-- "includes/simulation-test-figures.html"

Topic names are shortened in the figure: `cmd_vel` and `odom` are under `/diffbot/mobile_base_controller/`, `joint_states`, `wheel_cmd_velocities` and `measured_joint_states` under `/diffbot/`, and `model_states` under `/gazebo/`. The figure shows both test nodes, which rostest runs one after another (step 1). The second tab shows the real robot, see [Simulation and real robot](#simulation-and-real-robot).

[ros_control](http://wiki.ros.org/ros_control) isn't a node of its own. It's a set of C++ libraries, used inside the process that has access to the joints, here the Gazebo server. The robot's URDF loads the [`gazebo_ros_control`](http://wiki.ros.org/gazebo_ros_control) plugin ([`robot.gazebo.xacro`]({{ diffbot_repo_url }}/diffbot_description/urdf/include/robot.gazebo.xacro)). The plugin creates the hardware interface and the controller manager, and the controller manager loads the controllers:

- The **hardware interface**, `DefaultRobotHWSim`, gives the controllers each wheel joint's position and velocity, and takes a velocity command for each wheel. It reads the values from the simulated joints, for the wheel joints that have a transmission in the URDF, and drives each joint to its commanded velocity with a PID controller, with the gains from `pid.yaml`.
- The **controller manager** updates the controllers. In each simulation step, the plugin reads the joints (`readSim()`), lets the controller manager update the controllers, and applies the commands (`writeSim()`).
- The **controllers** read and write the hardware interface's values directly, in the same process, not over topics. `joint_state_controller` copies the joint values to `joint_states`. `mobile_base_controller`, a [diff_drive_controller](http://wiki.ros.org/diff_drive_controller), turns the velocities from `cmd_vel` into a velocity for each wheel, and computes the odometry from the wheel joint positions.

So three topics describe the robot's motion, each at a different level:

| Topic | Published by | Contains |
|:--|:--|:--|
| `/diffbot/joint_states` | `joint_state_controller` | Each wheel joint's angle (rad) and velocity (rad/s) |
| `/diffbot/mobile_base_controller/odom` | diff_drive_controller | The odometry: the pose that the controller computes from the wheel angles, with the wheel radius and separation from `diffbot_control.yaml` |
| `/gazebo/model_states` | Gazebo's ROS interface, not ros_control | The true pose of every model in Gazebo's world, from the physics simulation |

`test_drive.py` compares the odometry with the true pose. It doesn't use `joint_states`: to get a pose from the wheel angles, it would have to repeat the controller's calculation, with the same wheel radius and separation, and so it would repeat the same configuration mistakes. The true pose comes from the physics instead, so it shows where the robot really went, including the slip of the wheels. That's how the test catches swapped wheel joints. The controller sends the left wheel's command to the right wheel, so the robot turns right. But it also reads the right wheel's angle as the left one's, so its odometry still reports the commanded left turn. Only the true pose shows the difference, and there is none on the real robot.

### 3. test_drive.py connects to the robot

rostest starts [`test_drive.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_drive.py) as a ROS node. Its last line, `rostest.rosrun('diffbot_gazebo', 'test_drive', TestDrive)`, runs the [unittest](https://docs.python.org/3/library/unittest.html) class `TestDrive` and writes the results to a file that rostest reads. unittest calls `setUpClass()` once, then the test methods in alphabetical order: `test_forward()`, then `test_turn()`.

```python
class TestDrive(unittest.TestCase):

    cmd_vel: rospy.Publisher
    odom_pose: Pose2D
    true_pose: Pose2D

    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node('test_drive')
        cls.cmd_vel = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        rospy.Subscriber(ODOM_TOPIC, Odometry, cls.on_odom)
        rospy.Subscriber(MODEL_STATES_TOPIC, ModelStates, cls.on_model_states, queue_size=1)
        rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=120)
        rospy.wait_for_message(MODEL_STATES_TOPIC, ModelStates, timeout=30)

    @classmethod
    def on_odom(cls, msg: Odometry) -> None:
        cls.odom_pose = planar(msg.pose.pose)

    @classmethod
    def on_model_states(cls, msg: ModelStates) -> None:
        if MODEL_NAME in msg.name:
            cls.true_pose = planar(msg.pose[msg.name.index(MODEL_NAME)])
```

- `cmd_vel`, `odom_pose` and `true_pose` belong to the class, not to a `TestDrive` object. unittest creates a new object for each test method, while `setUpClass()` starts the node, its publisher and its subscribers only once. `@classmethod` passes the class to a method as `cls`.
- `rospy.init_node()` registers the node with the ROS master.
- The publisher sends velocity commands, `Twist` messages, to the controller's `cmd_vel` topic. The constants at the top of the file, such as `CMD_VEL_TOPIC`, hold the full topic names.
- The two subscribers call `on_odom()` for each odometry message, and `on_model_states()` for each `model_states` message. Gazebo publishes `model_states` in every simulation step, so `queue_size=1` keeps only the newest message instead of falling behind.
- `rospy.wait_for_message()` waits for the first message on each topic. It waits up to 120 s for the odometry, because Gazebo and the controllers take a while to start, especially in CI, and then up to 30 s for `model_states`. If no message comes, it raises an exception and the tests fail, for example when the controller doesn't start because of a wrong joint name.
- `on_odom()` keeps the latest odometry pose, converted to 2D by `planar()`.
- `on_model_states()`: a `model_states` message has two lists, the names of all models in the world and their poses, in the same order. Here these are the ground plane and the robot, which is called `diffbot` (`MODEL_NAME`) for both robots. Gazebo starts publishing before the robot is spawned, so the first messages don't contain it, and the `if` skips them. `msg.name.index(MODEL_NAME)` finds the robot's position in the list of names, and its pose is at the same position in `msg.pose`.

`planar()` turns a ROS pose into a `Pose2D`, a [named tuple](https://docs.python.org/3/library/typing.html#typing.NamedTuple) with x, y and the heading θ, as on [Robot Representations](../theory/modeling-control/robot-representations.md#robot-pose):

```python
def planar(pose: Pose) -> Pose2D:
    q = pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return Pose2D(pose.position.x, pose.position.y, yaw)
```

ROS messages give an orientation as a [quaternion](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation) (x, y, z, w): four numbers that describe any rotation in 3D, without the special cases of three angles ([tf2 quaternion tutorial](http://wiki.ros.org/tf2/Tutorials/Quaternions)). `euler_from_quaternion` from tf converts it to roll, pitch and yaw, the rotations about the x, y and z axes ([REP 103](https://www.ros.org/reps/rep-0103.html)). On flat ground, roll and pitch stay about 0, and yaw is the heading θ: the angle from the x axis, growing when the robot turns to the left.

### 4. test_forward measures a motion

The constants at the top of the file name the numbers that the tests use:

```python
DURATION = 3.0  # s
FORWARD_VELOCITY = 0.2  # m/s
TURN_VELOCITY = 0.5  # rad/s
COMMAND_RATE = 20  # Hz
MIN_TRANS = 0.01  # m
```

`COMMAND_RATE` comes up in step 5 and `MIN_TRANS` in step 7. `test_forward()`:

```python
def test_forward(self) -> None:
    odom, truth = self.measure_motion(
        linear_velocity=FORWARD_VELOCITY, angular_velocity=0.0, duration=DURATION)
    expected_trans = FORWARD_VELOCITY * DURATION
    for motion in (odom, truth):
        self.assertAlmostEqual(motion.trans, expected_trans, delta=0.1)
        self.assertAlmostEqual(motion.rot1, 0.0, delta=0.15)
        self.assertAlmostEqual(turned(motion), 0.0, delta=0.2)
    self.assertAlmostEqual(odom.trans, truth.trans, delta=0.05)
```

It has two parts. `measure_motion()` drives the robot at 0.2 m/s for 3 s without turning, and returns the motion twice: measured with the odometry (`odom`) and with the true pose (`truth`). The rest checks the two motions (step 8). The call names its arguments, so it reads without looking up `measure_motion()`. The method itself:

```python
def measure_motion(self, linear_velocity: float, angular_velocity: float,
                   duration: float) -> Tuple[Motion, Motion]:
    odom_start, true_start = self.odom_pose, self.true_pose
    self.drive(linear_velocity, angular_velocity, duration)
    self.stop()
    odom = relative_motion(odom_start, self.odom_pose)
    truth = relative_motion(true_start, self.true_pose)
    for source, motion in (('odometry', odom), ('true pose', truth)):
        rospy.loginfo('%s: rot1 %.3f rad, trans %.3f m, rot2 %.3f rad', source, *motion)
    return odom, truth
```

1. It keeps the latest odometry pose and true pose as the start poses. While the robot drives, the callbacks keep updating `odom_pose` and `true_pose`.
2. `drive()` sends the command (step 5).
3. `stop()` stops the robot (step 6).
4. `relative_motion()` computes the motion from each start pose to the matching end pose (step 7).
5. It logs both motions, so the test results show the measured values even when a check fails.

### 5. drive() sends the command

```python
def drive(self, linear_velocity: float, angular_velocity: float, duration: float) -> None:
    cmd = Twist()
    cmd.linear.x = linear_velocity
    cmd.angular.z = angular_velocity
    rate = rospy.Rate(COMMAND_RATE)
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end_time and not rospy.is_shutdown():
        self.cmd_vel.publish(cmd)
        rate.sleep()
```

- A `Twist` holds a linear and an angular velocity, each as a 3D vector. A differential drive robot can only drive along its x axis, `linear.x` in m/s, and turn about its z axis, `angular.z` in rad/s, positive to the left. The controller ignores the other values.
- The loop sends the command 20 times per second (`COMMAND_RATE`) until `end_time`. Sending it once isn't enough: diff_drive_controller stops the robot when no command arrives for 0.5 s, its default `cmd_vel_timeout`.
- The loop checks the end time once per cycle, so the last command stays in effect up to one cycle, 50 ms, too long: at most 1 cm or 0.025 rad in these tests. That's small against the tolerances in step 8.
- `rospy.Time.now()` and `rospy.Rate` follow Gazebo's clock (step 2), so `duration` is simulation time.

### 6. stop() stops the robot

```python
def stop(self) -> None:
    self.cmd_vel.publish(Twist())
    rospy.sleep(1.0)
```

- `Twist()` is a command with all velocities 0.
- The controller limits braking like speeding up: `diffbot_control.yaml` sets a maximum acceleration of 0.6 m/s², and diff_drive_controller uses the same value for braking unless a `min_acceleration` is set. From 0.2 m/s, the robot needs 0.33 s to stop.
- `rospy.sleep(1.0)` waits 1 s of simulation time, so the robot stands still and both the odometry and Gazebo have published its end pose.

### 7. relative_motion() compares two poses

The odometry and Gazebo give poses in different frames. The odometry starts at 0, 0, 0 where the controller started, while Gazebo spawned the robot at x = −2 m, y = −0.5 m in its world frame. So the test doesn't compare the poses, but the motion between them. It describes the motion with the odometry motion model from *Probabilistic Robotics* by Thrun, Burgard and Fox, section 5.4 ([Amazon.com][amazon_book_probabilistic_robotics_us], [Amazon.de][amazon_book_probabilistic_robotics_ger], affiliate links): a rotation δ<sub>rot1</sub> towards the end position, a straight translation δ<sub>trans</sub> to it, and a rotation δ<sub>rot2</sub> to the end heading.

--8<-- "includes/simulation-test-moved.html"

For the start pose (x₀, y₀, θ₀) and the end pose (x₁, y₁, θ₁), the book computes them as:

$$
\begin{aligned}
\delta_\text{rot1} &= \operatorname{atan2}(y_1 - y_0,\; x_1 - x_0) - \theta_0 \\
\delta_\text{trans} &= \sqrt{(x_1 - x_0)^2 + (y_1 - y_0)^2} \\
\delta_\text{rot2} &= \theta_1 - \theta_0 - \delta_\text{rot1}
\end{aligned}
$$

These three numbers don't depend on the frame: moving or rotating the frame changes both poses the same way. So the motion from the odometry and the motion from the true pose can be compared. In `test_drive.py`:

```python
def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_motion(start_pose: Pose2D, end_pose: Pose2D) -> Motion:
    x0, y0, theta0 = start_pose
    x1, y1, theta1 = end_pose
    trans = math.hypot(x1 - x0, y1 - y0)
    rot1 = wrap(math.atan2(y1 - y0, x1 - x0) - theta0) if trans >= MIN_TRANS else 0.0
    rot2 = wrap(theta1 - theta0 - rot1)
    return Motion(rot1, trans, rot2)


def turned(motion: Motion) -> float:
    return wrap(motion.rot1 + motion.rot2)
```

- `x0, y0, theta0 = start_pose` unpacks the named tuple into its three values.
- `trans` is δ<sub>trans</sub>, the straight-line distance: the length of the hypotenuse of a right triangle with the sides Δx = x₁ − x₀ and Δy = y₁ − y₀. [`math.hypot`](https://docs.python.org/3/library/math.html#math.hypot) computes exactly this square root.
- `rot1` is δ<sub>rot1</sub>. [atan2](https://en.wikipedia.org/wiki/Atan2)(Δy, Δx) is the angle of the vector (Δx, Δy) from the x axis, in −π to π. Unlike atan(Δy / Δx), it uses the signs of both values, so it tells forward from backward and works for Δx = 0. Subtracting θ₀ makes it relative to the direction the robot faced. When the robot turns on the spot, its position hardly changes, and the direction of such a short translation is noise. So below 1 cm (`MIN_TRANS`), δ<sub>rot1</sub> is 0, as in the odometry model of ROS's [amcl](http://wiki.ros.org/amcl) ([`amcl_odom.cpp`](https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/src/amcl/sensors/amcl_odom.cpp)).
- `rot2` is δ<sub>rot2</sub>, the rest of the heading change.
- `wrap()` keeps the angles in −π to π. θ jumps from π to −π when the robot turns past 180°, which would make a small left turn look like almost a full turn to the right. Sine and cosine stay the same when the angle changes by 2π, and atan2 gives the angle back in −π to π. amcl normalizes its angles the same way.
- `turned()` is the total turn, δ<sub>rot1</sub> + δ<sub>rot2</sub> = θ₁ − θ₀.

### 8. test_forward checks the motion

`test_forward()` expects the robot to drive `FORWARD_VELOCITY * DURATION` = 0.2 m/s · 3 s = 0.6 m straight ahead. The controller's acceleration limit costs 3.3 cm while the robot speeds up, and gives them back while it brakes (step 6). `drive()` adds up to 1 cm (step 5). The robots drove 0.58 to 0.62 m.

`assertAlmostEqual(measured, expected, delta=d)` passes when the two differ by at most d. Each tolerance lies between what the robots measure in Gazebo and what a mistake would give. The loop checks the odometry and the true pose the same way, and the last check compares them:

| Check | Expected | Measured | Fails when |
|:--|:--|:--|:--|
| δ<sub>trans</sub> | 0.6 ± 0.1 m | 0.58 to 0.62 m | The robot doesn't move, or drives much too slow or too fast |
| δ<sub>rot1</sub> | 0 ± 0.15 rad | 0.000 to 0.002 rad | It drives off to the side: 0.15 rad is 9 cm at 0.6 m |
| Total turn | 0 ± 0.2 rad | 0.01 rad or less | Its heading changes, for example because one wheel turns slower |
| Odometry and true δ<sub>trans</sub> | Within 0.05 m | About 0.01 m apart | The controller's wheel radius doesn't match the model: 20 % too big gave 0.09 m |

### 9. test_turn turns on the spot

`test_turn()` runs the same way, with the angular velocity instead of the linear one:

```python
def test_turn(self) -> None:
    odom, truth = self.measure_motion(
        linear_velocity=0.0, angular_velocity=TURN_VELOCITY, duration=DURATION)
    expected_turn = TURN_VELOCITY * DURATION
    for motion in (odom, truth):
        self.assertAlmostEqual(turned(motion), expected_turn, delta=1.0)
        self.assertAlmostEqual(motion.trans, 0.0, delta=0.15)
    self.assertAlmostEqual(turned(odom), turned(truth), delta=0.25)
```

It starts wherever `test_forward()` left the robot. That doesn't matter, because it checks only the motion from there.

It expects 0.5 rad/s · 3 s = 1.5 rad to the left, on the spot; the angular acceleration limit of 6 rad/s² hardly matters. It checks the total turn, not δ<sub>rot1</sub> and δ<sub>rot2</sub> on their own. In Gazebo, the robot's center drifts 2 to 4 cm while it turns, which is more than `MIN_TRANS`, so δ<sub>rot1</sub> becomes the direction of this drift. In one run, DiffBot's true pose gave δ<sub>rot1</sub> = −0.68 rad and δ<sub>rot2</sub> = 1.94 rad, a total turn of 1.26 rad.

In Gazebo the robots turn less than 1.5 rad: DiffBot about 1.3 rad, Remo 0.92 to 1.2 rad. Gazebo's wheel velocity controller (`pid.yaml`, step 2) has only a P gain, which leaves Remo's wheels at about 80 % of the commanded speed, and the wheels slip. So the tolerance of the turn is wide for now; [diffbot#116](https://github.com/ros-mobile-robots/diffbot/issues/116) tunes the gains and then narrows it to ±0.3 rad.

| Check | Expected | Measured | Fails when |
|:--|:--|:--|:--|
| Total turn | 1.5 ± 1.0 rad | DiffBot about 1.3 rad, Remo 0.92 to 1.2 rad | The robot turns right, for example with swapped wheel joints, or doesn't turn |
| δ<sub>trans</sub> | 0 ± 0.15 m | Odometry below 0.01 m, true pose 0.02 to 0.04 m | It drives off instead of turning on the spot |
| Odometry and true total turn | Within 0.25 rad | 0.02 to 0.11 rad apart, because the wheels [slip](../theory/modeling-control/odometry.md#wheel-slip) a little | The wheel radius doesn't match the model: 20 % too big gave up to 0.31 rad |

### 10. publishtest checks the topics

When `test_drive.py` has ended, rostest stops everything and starts `sim_drive.launch` again for the second test node, `publish_diffbot` (step 1). [`publishtest`](http://wiki.ros.org/rostest/Nodes#publishtest) is a test node that comes with rostest. It passes when each topic in its parameters gets a message within its `timeout`, here 90 s for `joint_states` and the odometry. `negative: True` would check the opposite, that nothing is published. So it checks that both controllers started; `test_drive.py` doesn't use `joint_states`.

### 11. The results

When catkin runs the tests, each test node writes its results to an XML file in `build/diffbot_gazebo/test_results/diffbot_gazebo/`, for example `rosunit-drive_diffbot.xml` with both test methods and the logged motions. rostest reads them and adds a summary file for the `.test` file. `catkin build --catkin-make-args run_tests` doesn't fail when a test fails; `catkin_test_results build` sums up all result files and fails if any test failed, and that fails the [CI job](ci.md#dev-container-workflow).

## Two motion models from Probabilistic Robotics

The test uses two motion models from the same book. diff_drive_controller computes the odometry with the velocity motion model, and the test compares poses with the odometry motion model. They don't compete: the odometry motion model is meant for odometry like the one that diff_drive_controller computes with the velocity motion model.

**The velocity motion model in diff_drive_controller.** In each control cycle, [`Odometry::update()`](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/diff_drive_controller/src/odometry.cpp) gets the wheel joint positions in rad, `left_pos` and `right_pos`:

```cpp
const double left_wheel_cur_pos  = left_pos  * left_wheel_radius_;
const double right_wheel_cur_pos = right_pos * right_wheel_radius_;

const double left_wheel_est_vel  = left_wheel_cur_pos  - left_wheel_old_pos_;
const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;

left_wheel_old_pos_  = left_wheel_cur_pos;
right_wheel_old_pos_ = right_wheel_cur_pos;

const double linear  = (right_wheel_est_vel + left_wheel_est_vel) * 0.5 ;
const double angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation_;

integrate_fun_(linear, angular);
```

- A wheel's angle times its radius R is the distance it has rolled. The difference to the last cycle is the distance in this cycle, d<sub>l</sub> and d<sub>r</sub>. Despite their names, `*_est_vel`, `linear` and `angular` are distances and an angle, not velocities.
- `linear` is the distance Δs that the robot's center moved in this cycle, the mean of both wheels. `angular` is the angle Δθ it turned: the difference of the wheels' distances divided by the wheel separation b. These are the kinematics of a differential drive robot ([Odometry](../theory/modeling-control/odometry.md#kinematic-model), where the wheel separation is 2L):

$$
\Delta s = \frac{d_r + d_l}{2}, \qquad \Delta\theta = \frac{d_r - d_l}{b}
$$

`integrate_fun_` is `integrateExact()` by default:

```cpp
if (fabs(angular) < 1e-6)
  integrateRungeKutta2(linear, angular);
else
{
  const double heading_old = heading_;
  const double r = linear/angular;
  heading_ += angular;
  x_       +=  r * (sin(heading_) - sin(heading_old));
  y_       += -r * (cos(heading_) - cos(heading_old));
}
```

The controller assumes that the robot drove on a circular arc of radius r = Δs / Δθ in this cycle. That's the motion of the velocity motion model, section 5.3 of the book: a robot that drives at constant linear and angular velocities v and ω for a time Δt ends at

$$
\begin{aligned}
x' &= x - \frac{v}{\omega}\sin\theta + \frac{v}{\omega}\sin(\theta + \omega\,\Delta t) \\
y' &= y + \frac{v}{\omega}\cos\theta - \frac{v}{\omega}\cos(\theta + \omega\,\Delta t) \\
\theta' &= \theta + \omega\,\Delta t
\end{aligned}
$$

With Δs = v Δt and Δθ = ω Δt, v / ω is the radius r, θ + ω Δt is the new heading, and the equations are the code's three lines. When the robot drives almost straight, r becomes huge, so the controller moves it straight instead, in the direction of the mean heading θ + Δθ / 2 (`integrateRungeKutta2()`). The book adds noise to the model, to describe how uncertain the motion is; the controller uses it without noise. The [Odometry](../theory/modeling-control/odometry.md) page derives a simpler update, a straight step in the direction θ, which is close to it for short steps.

The tests' expected values are this model's two simplest cases. With ω = 0 the robot drives straight, Δs = v t. With v = 0 it turns on the spot, Δθ = ω t.

**The odometry motion model in the test.** `relative_motion()` splits the difference between two poses into δ<sub>rot1</sub>, δ<sub>trans</sub> and δ<sub>rot2</sub> (step 7). It doesn't say how the robot moved in between: the robot drives straight, on an arc or turns on the spot, but never rotates, drives straight and rotates again. The three numbers only describe where it ended up, relative to where it started. The book uses this model for robots that report odometry rather than velocities: it takes two consecutive odometry poses as the robot's control input, and adds noise to the three numbers. ROS amcl uses it like this with the odometry of diff_drive_controller. The test uses the same split without noise, because it doesn't depend on the frame (step 7).

## Simulation and real robot

Switch the figure in [step 2](#2-sim_drivelaunch-starts-gazebo-and-the-controllers) to the second tab, *Real robot*. The blue box stays the same: the controller manager with the two controllers, configured by [`diffbot_control.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/diffbot_control.yaml). Only the hardware interface and what's below it change.

On the real robot, the [`diffbot_base`]({{ diffbot_repo_url }}/diffbot_base/src/diffbot_base.cpp) node takes Gazebo's place. It creates the hardware interface [`DiffBotHWInterface`]({{ diffbot_repo_url }}/diffbot_base/src/diffbot_hw_interface.cpp) and a controller manager, and runs the loop itself: `read()`, update the controllers, `write()`. Its [launch file]({{ diffbot_repo_url }}/diffbot_base/launch/diffbot.launch) loads the same `diffbot_control.yaml` and spawns the same two controllers. This hardware interface talks to the Teensy over ROS topics:

- It subscribes to `measured_joint_states`, the wheel angles and velocities that the Teensy publishes, and `read()` passes the latest values to the controllers.
- `write()` publishes the wheel velocity commands on `wheel_cmd_velocities`, and the Teensy's firmware runs a PID loop per wheel to reach them ([Base Hardware Interface](../packages/diffbot_base/index.md)).
- The [rosserial](http://wiki.ros.org/rosserial) node relays both topics over USB.

So the simulation test covers the robot model and the controller configuration that both share. It doesn't run `diffbot_base`, rosserial or the firmware. Tests for `diffbot_base` and the firmware without the robot are planned in [diffbot#114](https://github.com/ros-mobile-robots/diffbot/issues/114). [ROS Control, An overview](https://fjp.at/posts/ros/ros-control/) explains ros_control in more detail.

## Run it

Run it in the [dev container](dev-container.md), in `~/catkin_ws`: all tests with the two commands from the [dev container workflow](ci.md#dev-container-workflow), or one robot:

```console
rostest diffbot_gazebo sim_drive_diffbot.test
```

With `--text`, rostest prints the nodes' output in the terminal instead of log files, but it doesn't collect the test results then: it can report success although a check failed. Use `--text` to watch a run, and the command without it to check the result.

## Add a check

Add a test method to [`test_drive.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_drive.py): call `self.measure_motion()` and check the two motions it returns. It runs for both robots. The file follows [PEP 8](https://peps.python.org/pep-0008/) with ROS 2's line length of 99 characters ([ament_flake8's settings](https://github.com/ament/ament_lint/blob/rolling/ament_flake8/ament_flake8/configuration/ament_flake8.ini)), and its functions have [type hints](https://docs.python.org/3/library/typing.html).

A new `.test` file is registered with `add_rostest` in the package's [`CMakeLists.txt`]({{ diffbot_repo_url }}/diffbot_gazebo/CMakeLists.txt). Give it its own `gazebo_port`, like the two existing ones (step 1).
