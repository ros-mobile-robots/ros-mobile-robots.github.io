# Simulation Test

[`diffbot_gazebo`]({{ diffbot_repo_url }}/diffbot_gazebo) has a simulation test: it drives DiffBot and Remo in Gazebo and checks where they went. Both [CI workflows](ci.md#diffbot) run it on every pull request ([diffbot#110](https://github.com/ros-mobile-robots/diffbot/issues/110)).

## What it checks

Each robot has a [rostest](http://wiki.ros.org/rostest) file, [`sim_drive_diffbot.test`]({{ diffbot_repo_url }}/diffbot_gazebo/test/sim_drive_diffbot.test) and [`sim_drive_remo.test`]({{ diffbot_repo_url }}/diffbot_gazebo/test/sim_drive_remo.test). Both include [`sim_drive.launch`]({{ diffbot_repo_url }}/diffbot_gazebo/test/sim_drive.launch), which starts:

- Gazebo without a GUI, on the [empty world]({{ diffbot_repo_url }}/diffbot_gazebo/worlds/diffbot_empty.world), with the robot from its URDF ([`diffbot_view.launch`]({{ diffbot_repo_url }}/diffbot_gazebo/launch/diffbot_view.launch)). Gazebo needs no display for this.
- The controllers from [`diffbot_control.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/diffbot_control.yaml), the same file the real robot uses ([`diffbot_control.launch`]({{ diffbot_repo_url }}/diffbot_control/launch/diffbot_control.launch), without the rqt_robot_steering window).

Then two tests run:

- [`publishtest`](http://wiki.ros.org/rostest/Nodes#publishtest) checks that `/diffbot/joint_states` and `/diffbot/mobile_base_controller/odom` are published.
- [`test_drive.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_drive.py) drives forward at 0.2 m/s for 3 s, then turns at 0.5 rad/s for 3 s. It checks each move twice: with the controller's odometry, and with the robot's true pose from Gazebo (`/gazebo/model_states`). Both must be in range, and they must agree.

The true pose is what makes the test useful. The odometry is computed from the wheel joints and the controller's configuration, so it repeats some configuration mistakes: with the left and right wheel swapped, the robot turns right while the odometry reports a left turn. A wrong wheel radius makes odometry and true distance differ. A wrong joint name stops the controller, so no odometry is published. The test fails in all three cases. The durations are simulation time, so the expected distances don't depend on how fast the computer runs the simulation.

## Gazebo and ros_control

[ros_control](http://wiki.ros.org/ros_control) isn't a node of its own. It's a set of C++ libraries, used inside the process that has access to the joints:

- A **hardware interface**, a class derived from `hardware_interface::RobotHW`, gives the controllers each wheel joint's position and velocity and takes a velocity command for it. The controllers read and write these values directly, in the same process, not over topics. How the hardware interface gets the values from the robot and passes on the commands depends on the robot: in Gazebo it reads and sets the simulated joints, on DiffBot it uses ROS topics to talk to the Teensy.
- A **controller manager** loads the controllers as plugins and updates them. The program that creates the hardware interface and the controller manager runs the loop: read the joints, update the controllers, write the commands.
- The **controllers** publish the topics that the test checks. `joint_state_controller` copies the joint values to `/diffbot/joint_states`. `mobile_base_controller`, a [diff_drive_controller](http://wiki.ros.org/diff_drive_controller), computes the odometry from the wheel joint positions, publishes it, and turns the velocities from `cmd_vel` into a velocity command for each wheel.

The two tabs show where these parts run, in the simulation test and on the real robot. When you switch between them, the blue box stays the same: the controller manager with the two controllers, configured by [`diffbot_control.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/diffbot_control.yaml). Only the hardware interface and what's below it change.

--8<-- "includes/simulation-test-figures.html"

Topic names are shortened in the figures: `cmd_vel` and `odom` are under `/diffbot/mobile_base_controller/`, `joint_states`, `wheel_cmd_velocities` and `measured_joint_states` under `/diffbot/`, and `model_states` under `/gazebo/`. `controller_spawner` isn't shown; it only asks the controller manager to load and start the two controllers.

### In the simulation

All three parts run inside the Gazebo server. The robot's URDF loads the [`gazebo_ros_control`](http://wiki.ros.org/gazebo_ros_control) plugin ([`robot.gazebo.xacro`]({{ diffbot_repo_url }}/diffbot_description/urdf/include/robot.gazebo.xacro)). The plugin creates the hardware interface, `DefaultRobotHWSim`, for the wheel joints that have a transmission in the URDF, and a controller manager. In each simulation step the plugin runs the loop: `readSim()`, the controller manager's update, `writeSim()`. So the controllers publish from the `/gazebo` node.

`model_states` doesn't go through ros_control: Gazebo's own ROS interface publishes the true pose of every model.

### On the real robot

The [`diffbot_base`]({{ diffbot_repo_url }}/diffbot_base/src/diffbot_base.cpp) node takes Gazebo's place. It creates the hardware interface [`DiffBotHWInterface`]({{ diffbot_repo_url }}/diffbot_base/src/diffbot_hw_interface.cpp) and a controller manager, and runs the loop itself: `read()`, update the controllers, `write()`. Its [launch file]({{ diffbot_repo_url }}/diffbot_base/launch/diffbot.launch) loads the same `diffbot_control.yaml` and spawns the same two controllers. This hardware interface talks to the Teensy over ROS topics: it subscribes to `measured_joint_states`, the wheel angles and velocities that the Teensy publishes, and `read()` passes the latest values to the controllers. `write()` publishes the wheel velocity commands on `wheel_cmd_velocities`, and the Teensy's firmware runs a PID loop per wheel to reach them ([Base Hardware Interface](../packages/diffbot_base/index.md)). The [rosserial](http://wiki.ros.org/rosserial) node relays both topics over USB.

So the simulation test covers the robot model and the controller configuration that both share. It doesn't run `diffbot_base`, rosserial or the firmware. Tests for them without the robot are planned in [diffbot#114](https://github.com/ros-mobile-robots/diffbot/issues/114). [ROS Control, An overview](https://fjp.at/posts/ros/ros-control/) explains ros_control in more detail.

## Code walkthrough

The test is three files in [`diffbot_gazebo/test`]({{ diffbot_repo_url }}/diffbot_gazebo/test) and two lines in the package's [`CMakeLists.txt`]({{ diffbot_repo_url }}/diffbot_gazebo/CMakeLists.txt): `add_rostest` registers each `.test` file, so `catkin build --catkin-make-args run_tests` runs it.

### The test files

A `.test` file is a launch file for rostest. rostest starts a ROS master of its own, launches everything in the file, runs each `<test>` node and collects its results, and then stops all nodes. [`sim_drive_diffbot.test`]({{ diffbot_repo_url }}/diffbot_gazebo/test/sim_drive_diffbot.test), without its comments:

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

- `sim_drive.launch` starts Gazebo and the controllers, as described under [What it checks](#what-it-checks). `gazebo_port` gives this file's Gazebo server a port of its own.
- `publishtest` is a test node that comes with rostest. It passes when each topic gets a message within its `timeout`, here 90 s, because Gazebo takes a while to start in CI. `negative: True` would check the opposite, that nothing is published.
- `test_drive.py` is the drive test below.
- `time-limit` is how long rostest waits for a test node before it counts the test as failed.

[`sim_drive_remo.test`]({{ diffbot_repo_url }}/diffbot_gazebo/test/sim_drive_remo.test) differs only in the model, the port and the test names.

### test_drive.py

[`test_drive.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_drive.py) is a Python unittest class that `rostest.rosrun()` runs as a ROS node.

**Setup:** `setUpClass` runs once, before the tests. It starts the node, a publisher for `cmd_vel`, and subscribers for the odometry and `/gazebo/model_states`. The two callbacks keep only the latest pose of each, as x, y and yaw (`planar()`; the yaw comes from the orientation quaternion, with `euler_from_quaternion` from tf). Then it waits up to 120 s for the first odometry, because Gazebo and the controllers start slowly in CI.

**Driving:** `drive()` remembers both poses, sends the velocity command for the given time, then a zero command, and waits a second for the robot to stop:

```python
rate = rospy.Rate(20)
end = rospy.Time.now() + rospy.Duration(seconds)
while rospy.Time.now() < end and not rospy.is_shutdown():
    self.cmd_vel.publish(cmd)
    rate.sleep()
self.cmd_vel.publish(Twist())
rospy.sleep(1.0)  # let the robot stop
```

It sends the command 20 times a second because the controller stops the robot when no command arrives for 0.5 s (diff_drive_controller's default `cmd_vel_timeout`). `rospy.Time` and `rospy.Rate` follow Gazebo's clock (`use_sim_time`), so on a slow computer the test takes longer, but the robot drives the same distance. Then `drive()` returns the move twice, once from the odometry and once from the true pose, each computed by `moved()`.

**Comparing the moves:** the odometry and Gazebo give poses in different frames. The odometry starts at 0, 0, 0 where the controller started, while Gazebo spawns the robot at x = −2 m, y = −0.5 m in its world frame. So the test doesn't compare the poses, but how the robot moved, seen from its pose at the start, (x₀, y₀, ψ₀): how far forward, how far to the left, and how far it turned.

--8<-- "includes/simulation-test-moved.html"

The move from the start to the end is Δx = x₁ − x₀ and Δy = y₁ − y₀, in the frame of the poses. Its part along the robot's heading, the unit vector (cos ψ₀, sin ψ₀), is the distance forward. Its part along the robot's left side, (−sin ψ₀, cos ψ₀), is the distance to the left. Together, the two dot products are a [rotation](https://en.wikipedia.org/wiki/Rotation_matrix) of the move by −ψ₀:

$$
\begin{pmatrix} \text{forward} \\ \text{left} \end{pmatrix} =
\begin{pmatrix} \cos\psi_0 & \sin\psi_0 \\ -\sin\psi_0 & \cos\psi_0 \end{pmatrix}
\begin{pmatrix} \Delta x \\ \Delta y \end{pmatrix}
$$

The angle turned is ψ₁ − ψ₀, wrapped to −π to π with `atan2(sin, cos)`: the yaw jumps from π to −π when the robot turns past 180°, which would make a small left turn look like almost a full turn to the right. In the code:

```python
dx, dy = x1 - x0, y1 - y0
forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
left = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
turned = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
```

**The checks:** `test_forward` drives at 0.2 m/s for 3 s, and `test_turn` turns on the spot at 0.5 rad/s for 3 s. Each checks both moves, from the odometry and from the true pose:

| | `test_forward` | `test_turn` |
|:--|:--|:--|
| **Range** | 0.4 to 0.75 m forward: 0.6 m commanded, a bit less with the controller's acceleration limit. Less than 0.1 m to the side and 0.2 rad turned | 0.5 to 2.0 rad to the left: 1.5 rad commanded, but the wheels don't always reach the commanded speed when turning on the spot (Remo turned about 1.1 rad). Less than 0.15 m moved |
| **Odometry and true pose agree** | Within 0.05 m. They're usually about 0.01 m apart; a wheel radius 20 % too big gave 0.09 m | Within 0.25 rad. The wheels slip a little (0.02 to 0.11 rad apart); a wheel radius 20 % too big gave up to 0.31 rad |

unittest runs the test methods in alphabetical order. Each test checks only its own move, from wherever the robot is when it starts, so the order doesn't matter.

## Run it

Run it in the [dev container](dev-container.md), in `~/catkin_ws`: all tests with the two commands from the [dev container workflow](ci.md#dev-container-workflow), or one robot:

```console
rostest diffbot_gazebo sim_drive_diffbot.test
```

With `--text`, rostest prints the nodes' output in the terminal instead of log files, but it doesn't collect the test results then: it can report success although a check failed. Use `--text` to watch a run, and the command without it to check the result.

## Add a check

Add a test method to [`test_drive.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_drive.py), a Python [unittest](https://docs.python.org/3/library/unittest.html) class: call `self.drive()` and check the two moves it returns. It runs for both robots. A new `.test` file is registered with `add_rostest` in the package's [`CMakeLists.txt`]({{ diffbot_repo_url }}/diffbot_gazebo/CMakeLists.txt). Give it its own `gazebo_port`, like the two existing ones: catkin runs the test files in parallel, and two Gazebo servers on the same port can't start.
