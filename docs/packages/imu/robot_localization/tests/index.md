# Tests of the Localization

Five test files check the EKF and the IMU sample validator of the [Localization chapter](../../robot_localization.md) without the robot. Each file tests at a different level: the validator's rules alone, the validator and the filter on a clock that the test steps, the whole robot in Gazebo, the launch files, and the publisher of the transform `odom → base_footprint`. Each test has a page that walks through its code, with a figure of what the test feeds and what it checks.

| Test file | What the test starts | What the test checks | Page |
|:--|:--|:--|:--|
| [`test_imu_validator.py`]({{ diffbot_repo_url }}/diffbot_control/test/test_imu_validator.py) | Nothing. Its 27 unit tests call the class `ImuValidator` directly, without a ROS master | Each reason to drop a sample, the limits `max_age` and `max_future`, a clock that goes back, and the diagnostics | [IMU Sample Validator](validator.md) |
| [`test_ekf_synthetic.py`]({{ diffbot_repo_url }}/diffbot_control/test/test_ekf_synthetic.py) | [`ekf.launch`]({{ diffbot_repo_url }}/diffbot_control/launch/ekf.launch) with DiffBot's URDF model, on a simulated clock that the test publishes, once for each of its two scenarios | The estimate's yaw rate $\omega_z$ in seven phases of valid and invalid IMU samples, and the validator's counts on the topic `/diagnostics`. At the robot's rates, the IMU at 50 Hz and the wheel odometry at 10 Hz: the estimate, and when the filter publishes with the IMU running and stopped | [EKF on a Stepped Clock](stepped-clock.md) |
| [`test_ekf.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_ekf.py) | DiffBot or Remo in Gazebo with `ekf:=true`, and a drive controller whose wheel separation is 25 % too large | That the filter's turn follows the robot's true turn, not the wheel odometry's, and that only the EKF publishes the transform `odom → base_footprint` | [EKF in Gazebo](gazebo.md) |
| [`test_ekf_switch.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_ekf_switch.py) | Nothing. roslaunch's loader reads the launch files without starting a node | Which nodes each launch file starts with `ekf` true, false and by default, the value of `enable_odom_tf`, and the odometry topic that the planners read | [The ekf Switch](switch.md) |
| [`test_odom_tf.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_odom_tf.py) | DiffBot or Remo in Gazebo without the EKF | That only the drive controller publishes the transform `odom → base_footprint` | [The Transform's Publisher](odom-tf.md) |

## How the tests run

The five test files are Python tests, written with Python's [`unittest`](https://docs.python.org/3/library/unittest.html) module: each file defines its tests as methods of a `unittest.TestCase` class. catkin runs the five files with the workspace's other tests. The packages register them in their `CMakeLists.txt` with two macros:

- **`catkin_add_nosetests()`:** for the two files without ROS nodes. The test runner [nose](https://nose.readthedocs.io/en/latest/) finds the `TestCase` classes in the file and runs their tests.
- **`add_rostest()`:** for the three files that start nodes. rostest starts the nodes of the test's `.test` launch file, then the test file.

catkin's guides [Configuring Python nose tests](https://github.com/ros/catkin/blob/noetic-devel/doc/howto/format2/python_nose_configuration.rst) and [Configuring rostest](https://github.com/ros/catkin/blob/noetic-devel/doc/howto/format2/rostest_configuration.rst) explain the two macros. There are three ways to run the tests:

- **All tests:** `catkin build --catkin-make-args run_tests` in the [dev container](../../../../development/ci.md#dev-container-workflow) builds the workspace and runs every test, the five files included. CI runs the same tests on every pull request, as [Testing and CI](../../../../development/ci.md#ekf-tests) explains.
- **One test with ROS nodes:** `rostest` starts the test's launch file and the test, for example the two scenarios of the stepped-clock test and Remo's test in Gazebo:

    ```console
    rostest diffbot_control ekf_synthetic.test
    rostest diffbot_control ekf_synthetic_robot_rates.test
    rostest diffbot_gazebo sim_ekf_remo.test
    ```

- **One test file without ROS nodes:** the command for all tests runs the two files without ROS nodes with nose. To run only one of them, for example while you change the validator, Python's `unittest` runs the file from its folder:

    ```console
    roscd diffbot_control/test
    python3 -m unittest -v test_imu_validator
    ```

## What the tests don't show

The tests run without the robot. They don't show four things:

- **The real gyroscope while the robot drives:** its noise, its bias and its mounting. The tests use the simulated IMU's samples, or samples that the stepped-clock test publishes itself.
- **How long the samples take to arrive on the robot:** the issue [IMU validator: measure the samples' age on the robot, then set max_age and max_future (diffbot#219)](https://github.com/ros-mobile-robots/diffbot/issues/219) measures it.
- **The rates on the robot:** the stepped-clock test runs the filter at the simulation's rates, both inputs at 50 Hz, and at the robot's nominal rates. On the robot, the IMU is configured to publish at 50 Hz, and the wheel odometry arrives at a nominal 10 Hz, because the control loop of the node `diffbot_base` runs the drive controller 10 times per second. The box under [`frequency`](../../robot_localization.md#the-other-settings) on the Localization page explains this. The rates on the robot are not measured yet: the issue [Base: measure the control loop's rate on the robot, then decide (diffbot#218)](https://github.com/ros-mobile-robots/diffbot/issues/218) measures them and decides whether the control loop runs faster.
- **Whether the filter improves the odometry on the floor:** the [floor odometry test (diffbot#138)](https://github.com/ros-mobile-robots/diffbot/issues/138) will measure this on Remo.
