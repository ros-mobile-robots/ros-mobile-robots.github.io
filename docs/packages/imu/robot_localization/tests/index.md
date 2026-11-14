# Tests of the Localization

Five test files check the EKF and the IMU sample validator of the [Localization chapter](../../robot_localization.md) without the robot. Each file tests at a different level: the validator's rules alone, the validator and the filter on a clock that the test steps, the whole robot in Gazebo, the launch files, and the publisher of the transform `odom → base_footprint`. Each test has a page that walks through its code, with a figure of what the test feeds and what it checks.

| Test file | What the test starts | What the test checks | Page |
|:--|:--|:--|:--|
| [`test_imu_validator.py`]({{ diffbot_repo_url }}/diffbot_control/test/test_imu_validator.py) | Nothing. Its 27 unit tests call the class `ImuValidator` directly, without a ROS master | Each reason to drop a sample, the limits `max_age` and `max_future`, a clock that goes back, and the diagnostics | [IMU Sample Validator](validator.md) |
| [`test_ekf_synthetic.py`]({{ diffbot_repo_url }}/diffbot_control/test/test_ekf_synthetic.py) | [`ekf.launch`]({{ diffbot_repo_url }}/diffbot_control/launch/ekf.launch) with DiffBot's URDF model, on a simulated clock that the test publishes | The estimate's yaw rate $\omega_z$ in seven phases of valid and invalid IMU samples, and the validator's counts on the topic `/diagnostics` | [EKF on a Stepped Clock](stepped-clock.md) |
| [`test_ekf.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_ekf.py) | DiffBot or Remo in Gazebo with `ekf:=true`, and a drive controller whose wheel separation is 25 % too large | That the filter's turn follows the robot's true turn, not the wheel odometry's, and that only the EKF publishes the transform `odom → base_footprint` | [EKF in Gazebo](gazebo.md) |
| [`test_ekf_switch.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_ekf_switch.py) | Nothing. roslaunch's loader reads the launch files without starting a node | Which nodes each launch file starts with `ekf` true, false and by default, the value of `enable_odom_tf`, and the odometry topic that the planners read | [The ekf Switch](switch.md) |
| [`test_odom_tf.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_odom_tf.py) | DiffBot or Remo in Gazebo without the EKF | That only the drive controller publishes the transform `odom → base_footprint` | [The Transform's Publisher](odom-tf.md) |

## How the tests run

catkin runs the five test files with the workspace's other tests. The packages register them in their `CMakeLists.txt`: `catkin_add_nosetests()` for the two files without ROS nodes, `add_rostest()` for the three that start nodes through a `.test` launch file. catkin's guides [Configuring Python nose tests](https://github.com/ros/catkin/blob/noetic-devel/doc/howto/format2/python_nose_configuration.rst) and [Configuring rostest](https://github.com/ros/catkin/blob/noetic-devel/doc/howto/format2/rostest_configuration.rst) explain the two macros.

- **All tests:** `catkin build --catkin-make-args run_tests` in the [dev container](../../../../development/ci.md#dev-container-workflow) builds the workspace and runs every test. CI runs the same tests on every pull request, as [Testing and CI](../../../../development/ci.md#ekf-tests) explains.
- **One test with ROS nodes:** `rostest` starts the test's launch file and the test, for example the stepped-clock test and Remo's test in Gazebo:

    ```console
    rostest diffbot_control ekf_synthetic.test
    rostest diffbot_gazebo sim_ekf_remo.test
    ```

- **One test file without ROS nodes:** Python's `unittest` runs the file from its folder:

    ```console
    roscd diffbot_control/test
    python3 -m unittest -v test_imu_validator
    ```

## What the tests don't show

The tests run without the robot. They don't show how the real gyroscope behaves, with its noise, its bias and its mounting. They don't show the delays on the robot, or the filter with the robot's slower wheel odometry: in these tests, both inputs arrive at 50 Hz. They don't show whether the filter helps on the floor. The [floor odometry test](https://github.com/ros-mobile-robots/diffbot/issues/138) will measure that on Remo.
