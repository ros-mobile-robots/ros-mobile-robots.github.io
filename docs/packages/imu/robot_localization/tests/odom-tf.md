# Test: Who Publishes the Transform

This page belongs to [Localization: Wheel Odometry and IMU (EKF)](../../robot_localization.md). It walks through [`test_odom_tf.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_odom_tf.py) and its helper [`odom_tf_observer.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/odom_tf_observer.py).

The transform `odom → base_footprint` says where the robot is in the `odom` frame. Exactly one node may publish it on the topic `/tf`. With two publishers, the robot's pose would jump between their two estimates. Without the EKF, the drive controller publishes the transform. With the EKF, the EKF publishes it and the drive controller doesn't, as [How it fits together](../../robot_localization.md#how-it-fits-together) explains. The test proves in the simulation that exactly the expected node publishes the transform while the test watches. It doesn't check the transform's values, and it doesn't run on the real robot. [What the test doesn't show](#what-the-test-doesnt-show) lists the limits.

The check runs in two simulations:

| Simulation | Test | Expected publisher |
|:--|:--|:--|
| Without the EKF: `sim_drive_diffbot.test`, `sim_drive_remo.test` | `test_expected_node_is_the_only_odom_tf_publisher` in `test_odom_tf.py` | `/gazebo`, the node that runs the drive controller in the simulation |
| With the EKF: `sim_ekf_diffbot.test`, `sim_ekf_remo.test` | `test_ekf_is_the_only_odom_tf_publisher` in `test_ekf.py` | `/diffbot/ekf_localization` |

The figure shows both simulations, each in a drawing of its own. On the left of a drawing are the two possible publishers of the transform, the drive controller and the EKF. A solid green box with a ✓ publishes the transform, and a dashed box doesn't. On the right is the observer in the test node, which reads the topic `/tf` and counts the transforms `odom → base_footprint` by the node that published them. The list under a drawing names the node that the test expects and gives the observer's counts in a saved run.

--8<-- "includes/localization-test-odom-tf.html"

## Where the test runs

The simulation test files run `test_odom_tf.py` as their fifth test node. The excerpt shows the test node in DiffBot's file, from [`sim_drive_diffbot.test`, lines 41–51]({{ ekf_src_url }}/diffbot_gazebo/test/sim_drive_diffbot.test#L41-L51):

```xml title="sim_drive_diffbot.test: the test node of test_odom_tf.py" linenums="41"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/sim_drive_diffbot.test:41:51"
```

- **This simulation runs without the EKF:** `sim_drive_diffbot.test` doesn't pass `ekf`, so the drive controller's `enable_odom_tf` is `true`.
- **`publisher`:** the private parameter names the node that the test expects. In the simulation, the drive controller is not a node of its own. `gazebo_ros_control` loads the controller into Gazebo's process, whose node name is `/gazebo`.
- **A simulation of its own:** rostest starts the simulation again for each test node of the file, as [the test file on Simulation Test](../../../../development/simulation-test.md#1-the-test-file) explains. So the test sees the transforms of a fresh simulation.

`sim_drive_remo.test` has the same test node for the simulated Remo, named `odom_tf_remo`.

## The observer

`OdomTfObserver` subscribes to the topic `/tf` and counts the transforms `odom → base_footprint` for each publishing node. Both tests use it. The excerpt shows its constants, from [`odom_tf_observer.py`, lines 17–23]({{ ekf_src_url }}/diffbot_gazebo/test/odom_tf_observer.py#L17-L23):

```python title="odom_tf_observer.py: the constants" linenums="17"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/odom_tf_observer.py:17:23"
```

`OBSERVE` is 3 s of simulation time, Gazebo's clock. `POLL_INTERVAL` is 0.1 s of wall time, the computer's clock.

The excerpt shows how the observer subscribes and how it counts, from [`odom_tf_observer.py`, lines 33–50]({{ ekf_src_url }}/diffbot_gazebo/test/odom_tf_observer.py#L33-L50):

```python title="odom_tf_observer.py: the subscriber and the callback" linenums="33"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/odom_tf_observer.py:33:50"
```

- **`counts_by_publisher`:** a `Counter` from a node's name to the number of transforms `odom → base_footprint` that this node published.
- **The subscriber:** reads the topic `/tf`, whose messages are of the type `tf2_msgs/TFMessage`. One message can carry several transforms.
- **`callerid`:** a message on a topic doesn't name its sender. rospy attaches the connection's header to each message that it receives, and the header's `callerid` is the name of the node that published the message. This is how the observer knows who published a transform.
- **The two `if` conditions:** `on_tf()` counts a transform only if its parent frame `header.frame_id` is `PARENT_FRAME`, `odom`, and its `child_frame_id` is `CHILD_FRAME`, `base_footprint`. The topic `/tf` also carries the transforms of the robot's joints, which the observer ignores.
- **The lock:** rospy calls `on_tf()` in one thread for each publisher of the topic `/tf`, and the test reads the counts in another thread. The lock lets one thread at a time change or read the counter.

The excerpt shows the two methods that the tests call, from [`odom_tf_observer.py`, lines 52–66]({{ ekf_src_url }}/diffbot_gazebo/test/odom_tf_observer.py#L52-L66):

```python title="odom_tf_observer.py: counts() and wait_for_first_transform()" linenums="52"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/odom_tf_observer.py:52:66"
```

- **`counts()`:** returns a copy of the counts as a dictionary, for example `{'/gazebo': 155}`.
- **`wait_for_first_transform()`:** waits until the observer has counted a first transform and returns whether one came. The wait ends after `timeout` seconds of wall time, so it ends also when the simulation's clock doesn't run.

## The test

The excerpt shows the test class of `test_odom_tf.py`, from [`test_odom_tf.py`, lines 26–43]({{ ekf_src_url }}/diffbot_gazebo/test/test_odom_tf.py#L26-L43):

```python title="test_odom_tf.py: the test" linenums="26"
--8<-- "{{ ekf_src_raw }}/diffbot_gazebo/test/test_odom_tf.py:26:43"
```

1. `setUpClass()` creates the test's node and the observer. The observer counts from this moment on.
2. The test reads the expected node's name from its private parameter `publisher`.
3. It waits for a first transform `odom → base_footprint`, at most `STARTUP_TIMEOUT`, 120 s of wall time. Gazebo and the controller spawner need a while to start. Without a transform, the test fails with the message `nobody published odom → base_footprint`.
4. It waits `OBSERVE`, 3 s of simulation time, while the observer keeps counting.
5. It compares the names of the nodes in the counts with the expected one. `assertEqual([expected], list(counts))` passes only when the counts have exactly this one node. A second publisher fails the test, and so does a different one.

`test_ekf_is_the_only_odom_tf_publisher` in `test_ekf.py` does the same in the simulation with the EKF. It expects `/diffbot/ekf_localization`, and it also expects the controller's parameter `enable_odom_tf` to be `false`. [Who publishes the transform](gazebo.md#who-publishes-the-transform) on the Gazebo test's page shows this test.

The table lists what the observer counted in a run in the dev container's image, at the commit `3ef4a91` of [diffbot#154](https://github.com/ros-mobile-robots/diffbot/pull/154):

| Simulation | Simulated robot | Counted |
|:--|:--|:--|
| Without the EKF | DiffBot | `{'/gazebo': 155}` |
| Without the EKF | Remo | `{'/gazebo': 151}` |
| With the EKF | DiffBot | `{'/diffbot/ekf_localization': 375}` |
| With the EKF | Remo | `{'/diffbot/ekf_localization': 371}` |

Without the EKF, the count is about 3 s of transforms at the drive controller's 50 Hz. With the EKF, the observer of `test_ekf.py` also counts during the turn test, which runs first, so the count is larger.

## Run it

In the [dev container](../../../../development/dev-container.md), in the built workspace:

```console
rostest diffbot_gazebo sim_drive_diffbot.test
rostest diffbot_gazebo sim_drive_remo.test
```

Each command runs all five test nodes of the file. [Run it](../../../../development/simulation-test.md#run-it) and [Read the results](../../../../development/simulation-test.md#read-the-results) on Simulation Test explain rostest's options, its output and its result files. A passing run prints this result line for the test:

```console
[diffbot_gazebo.rosunit-odom_tf_diffbot/test_expected_node_is_the_only_odom_tf_publisher][passed]
```

The test prints the counts in one line, which rostest copies into the result file `rosunit-odom_tf_diffbot.xml`. In the saved run, the line was:

```text
odom → base_footprint published by: {'/gazebo': 155}
```

## What the test doesn't show

- **Transforms from before the test node started:** rostest starts the test node after the simulation's nodes, and the topic `/tf` doesn't repeat old messages. The observer doesn't count a transform that was published before it subscribed. The drive controller and the EKF usually start seconds later, after Gazebo has spawned the robot, but nothing guarantees this order.
- **A publisher that starts late:** the observer watches for `OBSERVE`, 3 s of simulation time, after the first transform. The test doesn't see a node that starts to publish the transform after that.
- **The transform's values:** the test counts who publishes. [The test in Gazebo](gazebo.md) compares the EKF's heading with the robot's true heading.
- **The real robot:** on the robot, the drive controller runs in the hardware interface's node, not in `/gazebo`. This test runs in the simulation only. The [switch test](switch.md) checks that the robot's launch files set `enable_odom_tf` for both positions of `ekf`.
