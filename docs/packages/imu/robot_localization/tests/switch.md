# Test: The ekf Switch of the Launch Files

This page belongs to [Localization: Wheel Odometry and IMU (EKF)](../../robot_localization.md). It walks through [`test_ekf_switch.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_ekf_switch.py), the test of the `ekf` argument in the launch files.

The `ekf` argument has to do several things at once in several launch files, as [Which launch file starts what](../../robot_localization.md#which-launch-file-starts-what) shows. With `ekf:=true`, a launch file starts the EKF and the IMU sample validator, turns off the drive controller's transform `odom → base_footprint` and makes the planner read the EKF's odometry. Without it, none of this may happen. The test proves that the launch files are written this way, for both positions of the switch and for its default. The test reads the launch files and starts nothing. So it doesn't prove that the nodes run or that they work together. The [stepped-clock test](stepped-clock.md) and the [test in Gazebo](gazebo.md) start them.

`test_ekf_switch.py` has five tests:

| Test | Launch files | What it checks |
|:--|:--|:--|
| `test_bringup_starts_the_ekf_and_the_validator_only_with_ekf_true` | `diffbot_bringup`: `bringup.launch`, `bringup_with_laser.launch` | [Both positions of the switch](#both-positions-of-the-switch) |
| `test_simulation_starts_the_ekf_and_the_validator_only_with_ekf_true` | `diffbot_gazebo`: `diffbot.launch` | [Both positions of the switch](#both-positions-of-the-switch) |
| `test_planners_read_the_filtered_odometry_only_with_ekf_true` | `diffbot_navigation`: `move_base.launch`, `diffbot_navigation.launch`, `diffbot_hw.launch`. `diffbot_mbf`: `amcl_demo.launch` | [The planner's odometry topic](#the-planners-odometry-topic) |
| `test_planner_reads_an_explicit_odom_topic_also_with_ekf_true` | `diffbot_navigation`: `move_base.launch` | [The planner's odometry topic](#the-planners-odometry-topic) |
| `test_navigation_simulation_starts_the_ekf_for_its_planner_with_ekf_true` | `diffbot_navigation`: `diffbot.launch` | [The simulation with navigation](#the-simulation-with-navigation) |

The next table lists what the test asserts. Each group of rows is one kind of launch file. A row is one way in which the test loads the files of its group: on the left the launch arguments, on the right what the test asserts about the nodes and the parameters that roslaunch would start. The table uses three short forms:

- **The nodes:** the table leaves out the namespace `/diffbot` of the nodes `ekf_localization`, `imu_validator` and `robot_state_publisher`.
- **A planner reads a topic:** the planner's node is among the nodes, and the node's name `odom` is remapped to this topic.
- **The controller's odometry and the EKF's odometry:** the topics `/diffbot/mobile_base_controller/odom` and `/diffbot/odometry/filtered`, the test's `WHEEL_ODOMETRY_TOPIC` and `FILTERED_TOPIC`.

<div class="readable-table">
<table>
<thead>
<tr><th scope="col">Arguments</th><th scope="col">What the test asserts</th></tr>
</thead>
<tbody>
<tr><td colspan="2"><strong>The launch files that start the robot or the simulation:</strong> <code>bringup.launch</code> and <code>bringup_with_laser.launch</code> of <code>diffbot_bringup</code>, <code>diffbot.launch</code> of <code>diffbot_gazebo</code></td></tr>
<tr><td><code>ekf:=false</code>, and no <code>ekf</code> argument</td><td>Neither <code>ekf_localization</code> nor <code>imu_validator</code> is among the nodes.<br><code>enable_odom_tf</code> is <code>True</code>.</td></tr>
<tr><td><code>ekf:=true</code></td><td><code>ekf_localization</code> and <code>imu_validator</code> are among the nodes.<br><code>enable_odom_tf</code> is <code>False</code>.<br>The five parameters in <code>EKF_PARAMETERS</code> have their values.<br><code>robot_state_publisher</code> is among the nodes.</td></tr>
<tr><td colspan="2"><strong>The launch files that start a planner:</strong> <code>move_base.launch</code>, <code>diffbot_navigation.launch</code> and <code>diffbot_hw.launch</code> of <code>diffbot_navigation</code>, which start the planner <code>/move_base</code>, and <code>amcl_demo.launch</code> of <code>diffbot_mbf</code>, which starts the planner <code>/move_base_flex</code></td></tr>
<tr><td>no <code>ekf</code> argument</td><td>The planner reads the controller's odometry.</td></tr>
<tr><td><code>ekf:=true</code></td><td>The planner reads the EKF's odometry.</td></tr>
<tr><td><code>ekf:=true</code> and an explicit <code>odom_topic</code></td><td>Only <code>move_base.launch</code>, with <code>odom_topic:=/other/odom</code>: the planner reads the topic <code>/other/odom</code>.</td></tr>
<tr><td colspan="2"><strong>The launch file that starts the simulation and a planner:</strong> <code>diffbot.launch</code> of <code>diffbot_navigation</code>, loaded with <code>open_rviz:=false</code></td></tr>
<tr><td><code>ekf:=true</code></td><td><code>ekf_localization</code> and <code>imu_validator</code> are among the nodes.<br><code>enable_odom_tf</code> is <code>False</code>.<br>The planner <code>/move_base</code> reads the EKF's odometry.</td></tr>
</tbody>
</table>
</div>

## Reading a launch file without starting it

roslaunch has a loader that reads a launch file and everything that the file includes, and returns what roslaunch would start: the nodes, the parameters and the remappings. The test calls this loader and looks at the result. No ROS master and no node run.

The figure shows this flow from top to bottom, for one launch file and its arguments. The test's function `load()` gives them to roslaunch's loader, `load_config_default()`. `load()` returns the loader's result as a `LaunchConfiguration` with three fields, and the test's assertions look at these fields. The dashed box is what roslaunch would do after loading, which the test leaves out.

--8<-- "includes/localization-test-switch.html"

The excerpt shows the test's own result type and the function that calls the loader, from [`test_ekf_switch.py`, lines 55–74]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf_switch.py#L55-L74):

```python title="test_ekf_switch.py: LaunchConfiguration and load()" linenums="55"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf_switch.py:55:74"
```

- **`load()`:** takes a package, the name of a launch file in the package's `launch` folder and the launch arguments. `load('diffbot_gazebo', 'diffbot.launch', ekf='true')` stands for the command `roslaunch diffbot_gazebo diffbot.launch ekf:=true`.
- **`load_config_default()`:** is roslaunch's loader. It resolves the includes, the arguments and the conditions such as `if="$(arg ekf)"`, as roslaunch does before it starts anything.
- **`nodes`:** a dictionary from each node's full name, for example `/diffbot/ekf_localization`, to roslaunch's description of the node. The loader keeps a node's namespace and its name apart, so `load()` joins them.
- **`parameters`:** a dictionary from each parameter's full name to its value.
- **`description`:** the launch file with its arguments, as text. Every failure message starts with it.

??? example "The constants of `test_ekf_switch.py`, lines 31–50"

    The excerpt shows the names that the test looks for, from [`test_ekf_switch.py`, lines 31–50]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf_switch.py#L31-L50). The name of a constant ends in `_NODE` for a node, in `_TOPIC` for a topic and in `_PARAMETER` for a parameter. `SWITCHED_OFF` holds the two ways to run without the EKF, `ekf:=false` and no `ekf` argument. `EKF_PARAMETERS` holds five parameters of the EKF and of the validator with the values that [`ekf.yaml`]({{ diffbot_repo_url }}/diffbot_control/config/ekf.yaml) and [`ekf.launch`]({{ diffbot_repo_url }}/diffbot_control/launch/ekf.launch) give them.

    ```python title="test_ekf_switch.py: the constants" linenums="31"
    --8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf_switch.py:31:50"
    ```

## Both positions of the switch

The first two tests call `assert_ekf_switch()` for the three launch files that start the robot or the simulation. The excerpt shows the two tests, from [`test_ekf_switch.py`, lines 166–171]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf_switch.py#L166-L171):

```python title="test_ekf_switch.py: the bringup files and the simulation" linenums="166"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf_switch.py:166:171"
```

`assert_ekf_switch()` checks the switch in both positions. The excerpt shows `assert_ekf_on()`, `assert_ekf_off()` and `assert_ekf_switch()`, from [`test_ekf_switch.py`, lines 131–149]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf_switch.py#L131-L149):

```python title="test_ekf_switch.py: the switch on, off, and both" linenums="131"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf_switch.py:131:149"
```

**Off**, `assert_ekf_off()`, once with `ekf:=false` and once without the argument:

1. `assert_no_ekf_nodes_start()`: neither `EKF_NODE`, which is `/diffbot/ekf_localization`, nor `VALIDATOR_NODE`, which is `/diffbot/imu_validator`, is among the nodes.
2. `assert_controller_odom_tf(configuration, enabled=True)`: the parameter `ODOM_TF_PARAMETER`, which is `/diffbot/mobile_base_controller/enable_odom_tf`, is set, and its value is `True`. So the drive controller publishes the transform.

**On**, `assert_ekf_on()`, with `ekf:=true`:

1. `assert_ekf_nodes_start()`: both nodes are among the nodes, `EKF_NODE` from the package `robot_localization` with the executable `ekf_localization_node`, and `VALIDATOR_NODE` from `diffbot_control` with `imu_validator.py`.
2. `assert_controller_odom_tf(configuration, enabled=False)`: `enable_odom_tf` is set, and its value is `False`. So only the EKF publishes the transform.
3. `assert_ekf_reads_the_controller_and_the_validator()`: the five parameters in `EKF_PARAMETERS` have their values. The EKF works in the frame `odom` for the frame `base_footprint`, reads the topics `mobile_base_controller/odom` and `imu/data_validated`, and the validator expects the frame `imu_link`.
4. `assert_state_publisher_starts()`: `STATE_PUBLISHER_NODE`, which is `/diffbot/robot_state_publisher`, is among the nodes. The EKF needs the transform between `imu_link` and `base_footprint`, which TF has from `robot_state_publisher`.

??? example "The assertion methods on the nodes and the parameters, lines 85–129"

    The excerpt shows the methods that `assert_ekf_on()` and `assert_ekf_off()` call, from [`test_ekf_switch.py`, lines 85–129]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf_switch.py#L85-L129). `assert_controller_odom_tf()` uses `assertIs()`: a parameter that the launch file doesn't set gives `None`, which is neither `True` nor `False`, so the check fails for a missing parameter in both positions.

    ```python title="test_ekf_switch.py: the assertions on the nodes and the parameters" linenums="85"
    --8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf_switch.py:85:129"
    ```

The launch file has to write `enable_odom_tf` in both positions, because a parameter stays on the ROS parameter server as long as the ROS master runs. A `false` from a launch with the EKF would otherwise remain for a later launch without it.

## The planner's odometry topic

A planner reads the robot's velocity from an odometry topic. With the EKF on, the planner has to read the EKF's estimate, so that its velocity and the transform come from the same estimate. In a launch file, the planner's node gets the topic as a remapping of the name `odom`. The excerpt shows what the test asserts about this remapping, from [`test_ekf_switch.py`, lines 151–164]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf_switch.py#L151-L164):

```python title="test_ekf_switch.py: the planner's odometry" linenums="151"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf_switch.py:151:164"
```

- **`assert_planner_odometry()`:** the planner's node is among the nodes, and its name `odom` is remapped to `topic`.
- **`assert_planner_follows_the_switch()`:** without the `ekf` argument, the planner reads `WHEEL_ODOMETRY_TOPIC`, the topic `/diffbot/mobile_base_controller/odom`. With `ekf:=true`, it reads `FILTERED_TOPIC`, the topic `/diffbot/odometry/filtered`.

`assert_planner_odometry()` reads the remapping with the helper `remapping()`. The excerpt shows the helper, from [`test_ekf_switch.py`, lines 77–79]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf_switch.py#L77-L79):

```python title="test_ekf_switch.py: remapping()" linenums="77"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf_switch.py:77:79"
```

roslaunch's description of a node holds the node's remappings as pairs of the name and what the name is remapped to. `remapping()` returns the second part for `name`, or `None` when the node has no such remapping.

The excerpt shows the two tests that use these methods, from [`test_ekf_switch.py`, lines 173–186]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf_switch.py#L173-L186):

```python title="test_ekf_switch.py: the planners" linenums="173"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf_switch.py:173:186"
```

- **Three files with `move_base`:** `move_base.launch`, `diffbot_navigation.launch` and `diffbot_hw.launch` of `diffbot_navigation` start the planner `/move_base`, the test's `MOVE_BASE_NODE`.
- **One file with Move Base Flex:** `amcl_demo.launch` of `diffbot_mbf` starts the planner `/move_base_flex`, the test's `MOVE_BASE_FLEX_NODE`.
- **An explicit topic wins:** `move_base.launch` has an `odom_topic` argument. Its default is the EKF's topic with `ekf:=true` and the controller's topic without it. With `odom_topic:=/other/odom` and `ekf:=true`, the planner reads `/other/odom`.

These launch files start no EKF, so the test checks nothing else in them.

## The simulation with navigation

`diffbot_navigation`'s `diffbot.launch` starts the simulation and a planner together and passes `ekf` to both. The excerpt shows the test, from [`test_ekf_switch.py`, lines 188–193]({{ ekf_src_url }}/diffbot_gazebo/test/test_ekf_switch.py#L188-L193):

```python title="test_ekf_switch.py: the simulation with navigation" linenums="188"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_ekf_switch.py:188:193"
```

With `ekf:=true`, the EKF and the validator start, `enable_odom_tf` is `False`, and `/move_base` reads `FILTERED_TOPIC`, the topic `/diffbot/odometry/filtered`. `open_rviz:=false` leaves RViz out of what the loader reads.

## Run it

catkin runs the test with the repository's other tests. The excerpt shows where `diffbot_gazebo` registers the test, from [`CMakeLists.txt`, lines 225–226]({{ ekf_src_url }}/diffbot_gazebo/CMakeLists.txt#L225-L226):

```cmake title="diffbot_gazebo/CMakeLists.txt: the test's registration" linenums="225"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/CMakeLists.txt:225:226"
```

To run only this test, in the [dev container](../../../../development/dev-container.md), in the built workspace:

```console
cd ~/catkin_ws
source devel/setup.bash
cd src/diffbot/diffbot_gazebo/test
python3 -m unittest -v test_ekf_switch
```

The test needs no ROS master and no Gazebo. In the saved run at the commit `3ef4a91` of [diffbot#154](https://github.com/ros-mobile-robots/diffbot/pull/154), the test printed these five result lines:

```console
test_bringup_starts_the_ekf_and_the_validator_only_with_ekf_true (test_ekf_switch.TestEkfSwitch) ... ok
test_navigation_simulation_starts_the_ekf_for_its_planner_with_ekf_true (test_ekf_switch.TestEkfSwitch) ... ok
test_planner_reads_an_explicit_odom_topic_also_with_ekf_true (test_ekf_switch.TestEkfSwitch) ... ok
test_planners_read_the_filtered_odometry_only_with_ekf_true (test_ekf_switch.TestEkfSwitch) ... ok
test_simulation_starts_the_ekf_and_the_validator_only_with_ekf_true (test_ekf_switch.TestEkfSwitch) ... ok
```

The test loads the launch files of `diffbot_bringup`, `diffbot_navigation` and `diffbot_mbf`, so `diffbot_gazebo`'s [`package.xml`]({{ diffbot_repo_url }}/diffbot_gazebo/package.xml) lists these three packages as test dependencies.

## What the test doesn't show

- **Running nodes:** the loader reads the launch files. Nothing starts, so the test can't see whether a node runs, publishes or finds its transforms.
- **`robot_state_publisher` in every file:** the test asserts `robot_state_publisher` for the two bringup files and for `diffbot_gazebo`'s `diffbot.launch`. It doesn't assert the node for [`minimal.launch`]({{ diffbot_repo_url }}/diffbot_bringup/launch/minimal.launch), which doesn't start it.
- **`ekf:=false` for the planners:** for the planners' launch files, the test loads the default and `ekf:=true`.
- **Other ways to start the robot:** the [robot container](../../../../processing_units/robot-container/usage.md) starts `bringup.launch` without `ekf`. The test doesn't read the container's settings.
