# Simulation Test

[`diffbot_gazebo`]({{ diffbot_repo_url }}/diffbot_gazebo) has a simulation test: it drives DiffBot and Remo in Gazebo and checks where they went. Both [CI workflows](ci.md#diffbot) run it on every pull request ([diffbot#110](https://github.com/ros-mobile-robots/diffbot/issues/110)). It catches mistakes in the robot model and the controller configuration, which the simulation shares with the real robot:

- **Left and right wheel joints swapped:** the robot turns right when it's told to turn left.
- **A wheel radius in the controller configuration that doesn't match the robot model:** the robot's odometry reports a different distance than the robot drove.
- **A wrong joint name:** the controller doesn't start, and nothing is published.

This page follows one run of the test, in the order things happen. rostest starts Gazebo and the controllers. `test_drive.py` drives the robot forward, then turns it on the spot, and checks each motion twice: with the robot's odometry and with its true pose in Gazebo. Then the page explains what is different on the real robot, how to run the test, and how to add a test of your own.

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

- `<include>` adds `sim_drive.launch` ([step 2][s2]) with two arguments: the robot model, and `gazebo_port`, a port of its own for this file's Gazebo server. catkin runs the test files in parallel, and two Gazebo servers on the same port can't start.
- Each `<test>` element is a test node: `publish_diffbot` runs `publishtest` ([step 10][s10]), and `drive_diffbot` runs `test_drive.py` ([steps 3][s3] to [9][s9]). `time-limit` is how long rostest waits for the node before it counts the test as failed.

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

Topic names are shortened in the figure: `cmd_vel` and `odom` are under `/diffbot/mobile_base_controller/`, `joint_states`, `wheel_cmd_velocities` and `measured_joint_states` under `/diffbot/`, and `model_states` under `/gazebo/`. The figure shows both test nodes, which rostest runs one after another ([step 1][s1]). The second tab shows the real robot, see [Simulation and real robot](#simulation-and-real-robot).

[ros_control](http://wiki.ros.org/ros_control) isn't a node of its own. It's a set of C++ libraries, used inside the process that has access to the joints, here the Gazebo server. The robot's URDF loads the [`gazebo_ros_control`](http://wiki.ros.org/gazebo_ros_control) plugin ([`robot.gazebo.xacro`]({{ diffbot_repo_url }}/diffbot_description/urdf/include/robot.gazebo.xacro)). The plugin creates the hardware interface and the controller manager, and the controller manager loads the controllers:

- The **hardware interface**, `DefaultRobotHWSim`, gives the controllers each wheel joint's position and velocity, and takes a velocity command for each wheel. It reads the values from the simulated joints, for the wheel joints that have a transmission in the URDF, and drives each joint to its commanded velocity with a PID controller, with the gains from `pid.yaml`.
- The **controller manager** updates the controllers. In each simulation step, the plugin reads the joints (`readSim()`), lets the controller manager update the controllers, and applies the commands (`writeSim()`).
- The **controllers** read and write the hardware interface's values directly, in the same process, not over topics. `joint_state_controller` copies the joint values to `joint_states`. `mobile_base_controller`, a [diff_drive_controller](http://wiki.ros.org/diff_drive_controller), turns the velocities from `cmd_vel` into a velocity for each wheel, and computes the odometry from the wheel joint positions. [Robot Motion](../theory/state-estimation/robot-motion.md#in-diff_drive_controller) shows how: with the velocity motion model of *Probabilistic Robotics*.

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
        cls.odom_pose = to_pose2d(msg.pose.pose)

    @classmethod
    def on_model_states(cls, msg: ModelStates) -> None:
        if MODEL_NAME in msg.name:
            cls.true_pose = to_pose2d(msg.pose[msg.name.index(MODEL_NAME)])
```

- `cmd_vel`, `odom_pose` and `true_pose` belong to the class, not to a `TestDrive` object. unittest creates a new object for each test method, while `setUpClass()` starts the node, its publisher and its subscribers only once. `@classmethod` passes the class to a method as `cls`.
- `rospy.init_node()` registers the node with the ROS master.
- The publisher sends velocity commands, `Twist` messages, to the controller's `cmd_vel` topic. The constants at the top of the file, such as `CMD_VEL_TOPIC`, hold the full topic names.
- The two subscribers call `on_odom()` for each odometry message, and `on_model_states()` for each `model_states` message. Gazebo publishes `model_states` in every simulation step, so `queue_size=1` keeps only the newest message instead of falling behind.
- `rospy.wait_for_message()` waits for the first message on each topic. It waits up to 120 s for the odometry, because Gazebo and the controllers take a while to start, especially in CI, and then up to 30 s for `model_states`. If no message comes, it raises an exception and the tests fail, for example when the controller doesn't start because of a wrong joint name.
- `on_odom()` keeps the latest odometry pose, converted to 2D by `to_pose2d()`.
- `on_model_states()`: a `model_states` message has two lists, the names of all models in the world and their poses, in the same order. Here these are the ground plane and the robot, which is called `diffbot` (`MODEL_NAME`) for both robots. Gazebo starts publishing before the robot is spawned, so the first messages don't contain it, and the `if` skips them. `msg.name.index(MODEL_NAME)` finds the robot's position in the list of names, and its pose is at the same position in `msg.pose`.

`to_pose2d()` converts a ROS pose, `geometry_msgs/Pose`, to a `Pose2D`: a [named tuple](https://docs.python.org/3/library/typing.html#typing.NamedTuple) with x, y and the heading θ, as on [Robot Representations](../theory/modeling-control/robot-representations.md#robot-pose):

```python
def to_pose2d(pose: Pose) -> Pose2D:
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
```

`COMMAND_RATE` comes up in [step 5][s5]. `test_forward()`:

```python
def test_forward(self) -> None:
    odom_motion, true_motion = self.measure_motion(
        linear_velocity=FORWARD_VELOCITY, angular_velocity=0.0, duration=DURATION)
    expected_forward = FORWARD_VELOCITY * DURATION
    for motion in (odom_motion, true_motion):
        self.assertAlmostEqual(motion.forward, expected_forward, delta=0.1)
        self.assertAlmostEqual(motion.left, 0.0, delta=0.1)
        self.assertAlmostEqual(motion.turned, 0.0, delta=0.2)
    self.assertAlmostEqual(odom_motion.forward, true_motion.forward, delta=0.05)
```

It has two parts. `measure_motion()` drives the robot at 0.2 m/s for 3 s without turning, and returns how the robot moved, measured twice: `odom_motion` with the odometry, and `true_motion` with the true pose from Gazebo. Each tells how far the robot moved forward and to the left, and how much it turned ([step 7][s7]). The rest checks the two motions ([step 8][s8]). The call names its arguments, so it reads without looking up `measure_motion()`. The method itself:

```python
def measure_motion(self, linear_velocity: float, angular_velocity: float,
                   duration: float) -> Tuple[RelativePose, RelativePose]:
    odom_start, true_start = self.odom_pose, self.true_pose
    self.drive(linear_velocity, angular_velocity, duration)
    self.stop()
    odom_motion = relative_pose(odom_start, self.odom_pose)
    true_motion = relative_pose(true_start, self.true_pose)
    for source, motion in (('odometry', odom_motion), ('true pose', true_motion)):
        rospy.loginfo('%s: forward %.3f m, left %.3f m, turned %.3f rad', source, *motion)
    return odom_motion, true_motion
```

1. It keeps the latest odometry pose and true pose as the start poses. While the robot drives, the callbacks keep updating `odom_pose` and `true_pose`.
2. `drive()` sends the command ([step 5][s5]).
3. `stop()` stops the robot ([step 6][s6]).
4. `relative_pose()` computes, for the odometry and for the true pose, where the robot ended up relative to where it started ([step 7][s7]).
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
- The loop checks the end time once per cycle, so the last command stays in effect up to one cycle, 50 ms, too long: at most 1 cm or 0.025 rad in these tests. That's small against the tolerances in [step 8][s8].
- `rospy.Time.now()` and `rospy.Rate` follow Gazebo's clock ([step 2][s2]), so `duration` is simulation time.

### 6. stop() stops the robot

```python
def stop(self) -> None:
    self.cmd_vel.publish(Twist())
    rospy.sleep(1.0)
```

- `Twist()` is a command with all velocities 0.
- The controller limits braking like speeding up: `diffbot_control.yaml` sets a maximum acceleration of 0.6 m/s², and diff_drive_controller uses the same value for braking unless a `min_acceleration` is set. From 0.2 m/s, the robot needs 0.33 s to stop.
- `rospy.sleep(1.0)` waits 1 s of simulation time, so the robot stands still and both the odometry and Gazebo have published its end pose.

### 7. relative_pose() compares two poses

The odometry and Gazebo give poses in different frames. The odometry starts at 0, 0, 0 where the controller started, while Gazebo spawned the robot at x = −2 m, y = −0.5 m in its world frame. So the test doesn't compare the poses, but the motion between them: the end pose relative to the start pose, in the robot's frame at the start. That's how far the robot moved forward and to the left, seen from where it started, and the angle it turned.

--8<-- "includes/simulation-test-relative-pose.html"

The displacement Δx = x₁ − x₀, Δy = y₁ − y₀ is in the frame of the poses. Rotating it by −θ₀ gives it in the robot's frame at the start:

$$
\begin{bmatrix} \text{forward} \\ \text{left} \end{bmatrix}
= \begin{bmatrix} \cos\theta_0 & \sin\theta_0 \\ -\sin\theta_0 & \cos\theta_0 \end{bmatrix}
\begin{bmatrix} \Delta x \\ \Delta y \end{bmatrix},
\qquad
\text{turned} = \theta_1 - \theta_0
$$

The matrix is the transpose of the rotation matrix R(θ₀), which turns a vector from the robot's frame into the frame of the poses; its transpose turns it back. This is the relative pose, a standard operation on poses: with homogeneous transforms, it is T₀⁻¹ T₁ (Peter Corke, *Robotics, Vision and Control*, chapter 2, [Amazon.com][amazon_book_corke_rvc_us], [Amazon.de][amazon_book_corke_rvc_ger], affiliate links). Moving or rotating the frame of the poses doesn't change it, so the motion from the odometry and from the true pose can be compared. In `test_drive.py`:

```python
class RelativePose(NamedTuple):
    forward: float
    left: float
    turned: float


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_pose(start_pose: Pose2D, end_pose: Pose2D) -> RelativePose:
    dx = end_pose.x - start_pose.x
    dy = end_pose.y - start_pose.y
    cos_theta = math.cos(start_pose.theta)
    sin_theta = math.sin(start_pose.theta)
    forward = dx * cos_theta + dy * sin_theta
    left = -dx * sin_theta + dy * cos_theta
    turned = wrap(end_pose.theta - start_pose.theta)
    return RelativePose(forward, left, turned)
```

- `RelativePose` is a named tuple like `Pose2D`: forward and left in m, turned in rad, positive to the left.
- `dx` and `dy` are the displacement in the frame of the poses.
- `forward` is the part of the displacement along the robot's heading at the start, the unit vector (cos θ₀, sin θ₀). `left` is its part along the robot's left side, (−sin θ₀, cos θ₀). These are the two rows of the matrix.
- `turned` is the difference of the headings. `wrap()` keeps it in −π to π: θ jumps from π to −π when the robot turns past 180°, which would make a small left turn look like almost a full turn to the right. Sine and cosine stay the same when the angle changes by 2π, and [atan2](https://en.wikipedia.org/wiki/Atan2) gives the angle back in −π to π.

*Probabilistic Robotics* describes the motion between two poses differently, as a rotation, a translation and a rotation: the odometry motion model, which filters such as amcl use to add noise to each part ([Robot Motion](../theory/state-estimation/robot-motion.md#odometry-motion-model)). The test needs no noise, and it checks forward, left and turned directly, so it uses the relative pose.

### 8. test_forward checks the motion

`test_forward()` expects the robot to drive `FORWARD_VELOCITY * DURATION` = 0.2 m/s · 3 s = 0.6 m straight ahead, the velocity motion model's simplest case ([Robot Motion](../theory/state-estimation/robot-motion.md#velocity-motion-model)). The controller's acceleration limit costs 3.3 cm while the robot speeds up, and gives them back while it brakes ([step 6][s6]). `drive()` adds up to 1 cm ([step 5][s5]). The robots drove 0.57 to 0.62 m.

`assertAlmostEqual(measured, expected, delta=d)` passes when the two differ by at most d. Each tolerance lies between what the robots measure in Gazebo and what a mistake would give. The loop checks the odometry and the true pose the same way, and the last check compares them:

| Check | Expected | Measured | Fails when |
|:--|:--|:--|:--|
| `forward` | 0.6 ± 0.1 m | 0.57 to 0.62 m | The robot doesn't move, or drives much too slow or too fast |
| `left` | 0 ± 0.1 m | About 1 mm | It drives off to the side |
| `turned` | 0 ± 0.2 rad | 0.004 rad or less | Its heading changes, for example because one wheel turns slower |
| Odometry and true `forward` | Within 0.05 m | About 0.01 m apart | The controller's wheel radius doesn't match the model: 20 % too big gave 0.09 m |

### 9. test_turn turns on the spot

`test_turn()` runs the same way, with the angular velocity instead of the linear one:

```python
def test_turn(self) -> None:
    odom_motion, true_motion = self.measure_motion(
        linear_velocity=0.0, angular_velocity=TURN_VELOCITY, duration=DURATION)
    expected_turn = TURN_VELOCITY * DURATION
    for motion in (odom_motion, true_motion):
        self.assertAlmostEqual(motion.turned, expected_turn, delta=1.0)
        self.assertAlmostEqual(math.hypot(motion.forward, motion.left), 0.0, delta=0.15)
    self.assertAlmostEqual(odom_motion.turned, true_motion.turned, delta=0.25)
```

It starts wherever `test_forward()` left the robot. That doesn't matter, because it checks only the motion from there.

It expects 0.5 rad/s · 3 s = 1.5 rad to the left; the angular acceleration limit of 6 rad/s² hardly matters. On the spot means that the robot's center stays where it was. [`math.hypot`](https://docs.python.org/3/library/math.html#math.hypot)(forward, left) is its distance from the start, the length of the hypotenuse of a right triangle with these two sides. In Gazebo, the center drifts 2 to 4 cm while the robot turns.

In Gazebo the robots turn less than 1.5 rad: DiffBot about 1.3 rad, Remo 0.92 to 1.34 rad. Gazebo's wheel velocity controller (`pid.yaml`, [step 2][s2]) has only a P gain, which leaves Remo's wheels at about 80 % of the commanded speed, and the wheels slip. So the tolerance of the turn is wide for now; [diffbot#116](https://github.com/ros-mobile-robots/diffbot/issues/116) tunes the gains and then narrows it to ±0.3 rad.

| Check | Expected | Measured | Fails when |
|:--|:--|:--|:--|
| `turned` | 1.5 ± 1.0 rad | DiffBot about 1.3 rad, Remo 0.92 to 1.34 rad | The robot turns right, for example with swapped wheel joints, or doesn't turn |
| Distance from the start | 0 ± 0.15 m | Odometry below 0.01 m, true pose 0.02 to 0.04 m | It drives off instead of turning on the spot |
| Odometry and true `turned` | Within 0.25 rad | 0.02 to 0.11 rad apart, because the wheels [slip](../theory/modeling-control/odometry.md#wheel-slip) a little | The wheel radius doesn't match the model: 20 % too big gave up to 0.31 rad |

### 10. publishtest checks the topics

When `test_drive.py` has ended, rostest stops everything and starts `sim_drive.launch` again for the second test node, `publish_diffbot` ([step 1][s1]). [`publishtest`](http://wiki.ros.org/rostest/Nodes#publishtest) is a test node that comes with rostest. It passes when each topic in its parameters gets a message within its `timeout`, here 90 s for `joint_states` and the odometry. `negative: True` would check the opposite, that nothing is published. So it checks that both controllers started; `test_drive.py` doesn't use `joint_states`.

### 11. The results

When catkin runs the tests, each test node writes its results to an XML file in `build/diffbot_gazebo/test_results/diffbot_gazebo/`, for example `rosunit-drive_diffbot.xml` with both test methods and the logged motions. rostest reads them and adds a summary file for the `.test` file. `catkin build --catkin-make-args run_tests` doesn't fail when a test fails; `catkin_test_results build` sums up all result files and fails if any test failed, and that fails the [CI job](ci.md#dev-container-workflow).

## Simulation and real robot

Switch the figure in [step 2][s2] to the second tab, *Real robot*. The blue box stays the same: the controller manager with the two controllers, configured by [`diffbot_control.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/diffbot_control.yaml). Only the hardware interface and what's below it change.

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

## How to add a test

As an example, this adds a test that drives backward. It runs for both robots, because both `.test` files run `test_drive.py`.

1. **Write the test method.** Add it to the `TestDrive` class in [`test_drive.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_drive.py), next to `test_forward()`:

    ```python
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

2. **Run it** for both robots, in the dev container:

    ```console
    rostest diffbot_gazebo sim_drive_diffbot.test
    rostest diffbot_gazebo sim_drive_remo.test
    ```

    rostest lists each test method with its result, for example `[diffbot_gazebo.rosunit-drive_diffbot/test_backward][passed]`.

3. **Look at the measured values.** `measure_motion()` logs both motions, and rostest copies the node's log into its results file, `~/.ros/test_results/diffbot_gazebo/rosunit-drive_diffbot.xml`:

    ```console
    grep -o '[a-z ]*: forward[^<&]*' ~/.ros/test_results/diffbot_gazebo/rosunit-drive_diffbot.xml
    ```

    The first two lines are `test_backward()`'s motions. In our run, DiffBot drove −0.57 m by the odometry and −0.58 m by the true pose, and turned 0.1 rad to the right on the way, more than when it drives forward. Remo drove −0.58 and −0.59 m, almost without turning.

4. **Choose the tolerances.** Each tolerance should lie between what both robots measure and what a mistake would give ([step 8][s8]). To see what a mistake gives, make one on purpose: for example, make `wheel_radius` in [`diffbot_control.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/diffbot_control.yaml) 20 % bigger, run the test again, look at the values, and undo the change. Then set each tolerance between the two, and write in a comment what you measured.

5. **Keep the file's style:** [PEP 8](https://peps.python.org/pep-0008/) with ROS 2's line length of 99 characters ([ament_flake8's settings](https://github.com/ament/ament_lint/blob/rolling/ament_flake8/ament_flake8/configuration/ament_flake8.ini)), [type hints](https://docs.python.org/3/library/typing.html), and named constants instead of bare numbers in the commands.

A test that needs another world or robot needs its own `.test` file. Register it with `add_rostest` in the package's [`CMakeLists.txt`]({{ diffbot_repo_url }}/diffbot_gazebo/CMakeLists.txt), and give it its own `gazebo_port` ([step 1][s1]).

[s1]: #1-rostest-reads-the-test-file
[s2]: #2-sim_drivelaunch-starts-gazebo-and-the-controllers
[s3]: #3-test_drivepy-connects-to-the-robot
[s5]: #5-drive-sends-the-command
[s6]: #6-stop-stops-the-robot
[s7]: #7-relative_pose-compares-two-poses
[s8]: #8-test_forward-checks-the-motion
[s9]: #9-test_turn-turns-on-the-spot
[s10]: #10-publishtest-checks-the-topics
