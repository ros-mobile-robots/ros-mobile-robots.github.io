# Tests of the Localization

Five test files check the EKF and the IMU sample validator of the [Localization chapter](../../robot_localization.md) without the robot. Each file tests at a different level: the validator's rules alone, the validator and the filter on a clock that the test steps, the whole robot in Gazebo, the launch files, and the publisher of the transform `odom → base_footprint`. Each test has a page that walks through its code, with a figure of what the test feeds and what it checks.

## Which question each test answers

Each question below leads to the page of the test that answers it. The pages go from a single decision of the validator, through the validator and the EKF together, to the simulated robot:

1. **[Which IMU samples may reach the EKF?](validator.md)**
    - **Answer:** a sample passes only while the ROS clock runs, and only with the right frame, finite values, valid variances, a stamp close to the ROS time, and a stamp later than the last valid sample's.
    - **Where the test runs:** in Python alone. The unit tests call the validator's class directly, without a ROS master.
2. **[What does the EKF estimate when the IMU samples are valid, invalid or stop?](stepped-clock.md)**
    - **Answer:** the estimate's yaw rate $\omega_z$ follows the valid IMU samples, ignores the samples that the validator drops, and goes to the wheels' yaw rate when no valid sample arrives, also at the robot's rates.
    - **Where the test runs:** the validator and the EKF run as ROS nodes, on a clock that the test steps, without Gazebo.
3. **[Does the EKF recover a wrong turn in Gazebo?](gazebo.md)**
    - **Answer:** yes. With a wheel separation that is 25 % too large in the drive controller, the wheel odometry's turn falls behind the true turn, and the EKF's turn stays within 0.05 rad of the true turn.
    - **Where the test runs:** in Gazebo without a window, with DiffBot and with Remo.
4. **[Does `ekf:=true` start the EKF and hand the transform over to the EKF?](switch.md)**
    - **Answer:** the launch files that start the robot or the simulation start the EKF and the validator only with `ekf:=true`, and then turn the drive controller's transform off. The planners read the EKF's odometry then.
    - **Where the test runs:** roslaunch's loader reads the launch files. No node starts.
5. **[Which node publishes the transform `odom → base_footprint`?](odom-tf.md)**
    - **Answer:** exactly one node: the drive controller without the EKF, and the EKF with it.
    - **Where the test runs:** in Gazebo without a window, with and without the EKF.

## How the tests run

The five test files are Python tests, written with Python's [`unittest`](https://docs.python.org/3/library/unittest.html) module: each file defines its tests as methods of a `unittest.TestCase` class. catkin runs the five files with the workspace's other tests. The packages register them in their `CMakeLists.txt` with two macros:

- **`catkin_add_nosetests()`:** for the two files without ROS nodes. The test runner [nose](https://nose.readthedocs.io/en/latest/) finds the `TestCase` classes in the file and runs their tests.
- **`add_rostest()`:** for the three files that start nodes. rostest starts the nodes of the test's `.test` launch file, then the test file.

catkin's guides [Configuring Python nose tests](https://github.com/ros/catkin/blob/noetic-devel/doc/howto/format2/python_nose_configuration.rst) and [Configuring rostest](https://github.com/ros/catkin/blob/noetic-devel/doc/howto/format2/rostest_configuration.rst) explain the two macros. CI runs the same tests on every pull request, as [Testing and CI](../../../../development/ci.md#ekf-tests) explains.

## Run the tests

Run the commands in the [dev container](../../../../development/dev-container.md), in a shell in which its built Noetic workspace `~/catkin_ws` is sourced, as the container's setup leaves it. These two commands run every test of the workspace, as CI does:

```console
catkin build --catkin-make-args run_tests
catkin_test_results build
```

The first line builds the workspace and runs every registered test, the five files included. The second line sums up the result files and fails if any test failed. [Run it](../../../../development/simulation-test.md#run-it) on Simulation Test explains the two commands.

To run one test file, use its command from the list. Each item names the line that a passing run prints at its end. The test's page shows the whole output and what the test doesn't show.

- **[`test_imu_validator.py`]({{ diffbot_repo_url }}/diffbot_control/test/test_imu_validator.py), [IMU Sample Validator](validator.md#run-the-tests):** Python's `unittest` runs the file from its folder.

    ```console
    roscd diffbot_control/test
    python3 -m unittest -v test_imu_validator
    ```

    A pass ends with the line `OK`, after `Ran 27 tests`.

- **[`test_ekf_synthetic.py`]({{ diffbot_repo_url }}/diffbot_control/test/test_ekf_synthetic.py), [EKF on a Stepped Clock](stepped-clock.md#runs):** rostest runs one scenario for each `.test` file.

    ```console
    rostest diffbot_control ekf_synthetic.test
    rostest diffbot_control ekf_synthetic_robot_rates.test
    ```

    A pass of each command ends with the line `* RESULT: SUCCESS`.

- **[`test_ekf.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_ekf.py), [EKF in Gazebo](gazebo.md#run-it):** rostest starts the simulation of one robot with the EKF.

    ```console
    rostest diffbot_gazebo sim_ekf_diffbot.test
    rostest diffbot_gazebo sim_ekf_remo.test
    ```

    A pass of `sim_ekf_diffbot.test` prints `[diffbot_gazebo.rosunit-ekf_diffbot/test_ekf_follows_the_true_turn_with_a_wrong_wheel_separation][passed]`.

- **[`test_ekf_switch.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_ekf_switch.py), [The ekf Switch](switch.md#run-it):** Python's `unittest` runs the file from its folder.

    ```console
    roscd diffbot_gazebo/test
    python3 -m unittest -v test_ekf_switch
    ```

    A pass ends with the line `OK`, after `Ran 5 tests`.

- **[`test_odom_tf.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_odom_tf.py), [The Transform's Publisher](odom-tf.md#run-it):** rostest starts the simulation of one robot without the EKF and runs all five test nodes of the file, this test among them.

    ```console
    rostest diffbot_gazebo sim_drive_diffbot.test
    rostest diffbot_gazebo sim_drive_remo.test
    ```

    A pass of `sim_drive_diffbot.test` prints `[diffbot_gazebo.rosunit-odom_tf_diffbot/test_expected_node_is_the_only_odom_tf_publisher][passed]`.

## What the tests don't show

The tests run without the robot. They don't show four things:

- **The real gyroscope while the robot drives:** its noise, its bias and its mounting. The tests use the simulated IMU's samples, or samples that the stepped-clock test publishes itself.
- **How long the samples take to arrive on the robot:** the issue [IMU validator: measure the samples' age on the robot, then set max_age and max_future (diffbot#219)](https://github.com/ros-mobile-robots/diffbot/issues/219) measures it.
- **The rates on the robot:** the stepped-clock test runs the filter at the simulation's rates, both inputs at 50 Hz, and at the robot's nominal rates. On the robot, the IMU is configured to publish at 50 Hz, and the wheel odometry arrives at a nominal 10 Hz, because the control loop of the node `diffbot_base` runs the drive controller 10 times per second. The box under [`frequency`](../../robot_localization.md#the-other-settings) on the Localization page explains this. The rates on the robot are not measured yet: the issue [Base: measure the control loop's rate on the robot, then decide (diffbot#218)](https://github.com/ros-mobile-robots/diffbot/issues/218) measures them and decides whether the control loop runs faster.
- **Whether the filter improves the odometry on the floor:** the [floor odometry test (diffbot#138)](https://github.com/ros-mobile-robots/diffbot/issues/138) will measure this on Remo.
