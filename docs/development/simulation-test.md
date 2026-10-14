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

So the simulation test covers the robot model and the controller configuration that both share. It doesn't run `diffbot_base`, rosserial or the firmware. [ROS Control, An overview](https://fjp.at/posts/ros/ros-control/) explains ros_control in more detail.

## Run it

Run it in the [dev container](dev-container.md), in `~/catkin_ws`: all tests with the two commands from the [dev container workflow](ci.md#dev-container-workflow), or one robot:

```console
rostest diffbot_gazebo sim_drive_diffbot.test
```

With `--text`, rostest prints the nodes' output in the terminal instead of log files, but it doesn't collect the test results then: it can report success although a check failed. Use `--text` to watch a run, and the command without it to check the result.

## Add a check

Add a test method to [`test_drive.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_drive.py), a Python [unittest](https://docs.python.org/3/library/unittest.html) class; it runs for both robots. A new `.test` file is registered with `add_rostest` in the package's [`CMakeLists.txt`]({{ diffbot_repo_url }}/diffbot_gazebo/CMakeLists.txt). Give it its own `gazebo_port`, like the two existing ones: catkin runs the test files in parallel, and two Gazebo servers on the same port can't start.
