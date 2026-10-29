# Geometry Test

Each robot's wheel radius and wheel separation are written down twice: in its URDF, the robot's model, and in the controller's `geometry.yaml`. [`test_geometry_consistency.py`]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py) checks that the two agree, for DiffBot and Remo, without starting Gazebo or the robot. It also checks the wheel joints that the controller drives by name. CI runs it on every pull request, with the other [tests](ci.md#tests).

The [simulation test](simulation-test.md) drives the robots in Gazebo and would notice large differences only: a wheel separation that's off by less than about 7 % can pass its turn. Neither test can show that the URDF matches the real wheels. That needs a measurement on the robot ([diffbot#138](https://github.com/ros-mobile-robots/diffbot/issues/138) for Remo, [diffbot#161](https://github.com/ros-mobile-robots/diffbot/issues/161) for DiffBot).

## What it compares

For Remo, these are the values. The two values in each row must be the same:

<div class="comparison-table">
<table>
<thead><tr><th scope="col">Remo's URDF</th><td></td><th scope="col">Remo's <code>geometry.yaml</code></th></tr></thead>
<tbody>
<tr><td><code>front_left_wheel</code><br>radius 0.033 m</td><td>⟷</td><td><code>wheel_radius</code><br>0.033</td></tr>
<tr><td><code>front_right_wheel</code><br>radius 0.033 m</td><td>⟷</td><td><code>wheel_radius</code><br>0.033</td></tr>
<tr><td>Wheel joints<br>0.144 m apart</td><td>⟷</td><td><code>wheel_separation</code><br>0.144</td></tr>
</tbody>
</table>
</div>

- **The radii** are the wheels' collision shapes: the simple spheres that Gazebo uses for contacts, not the meshes you see.
- **The separation** is the straight-line distance between the two wheel joints, the points the wheels turn about. `front_left_wheel_joint` is at y = 0.072 m and `front_right_wheel_joint` at y = −0.072 m, with the same x and z.
- **The tolerance** is 0.1 mm: a larger difference fails.

xacro computes the URDF's values from the description package's YAML files, for Remo `front_wheel.yaml` and `base.yaml` in remo_description. The test compares these nominal values only. A calibration of the real robot belongs in diff_drive_controller's multipliers, such as `wheel_separation_multiplier`, which the test leaves alone.

## Run it

In the [dev container](dev-container.md), in the built workspace:

```console
cd ~/catkin_ws
source devel/setup.bash
python3 src/diffbot/diffbot_gazebo/test/test_geometry_consistency.py -v
```

It needs remo_description in the workspace, as the dev container has it, but it starts no ROS node and no Gazebo, so it takes a few seconds. In a workspace built into an install space, source `install/setup.bash` instead. The output:

```console
test_agreement (__main__.TestComparison) ... ok
test_effort_interface (__main__.TestComparison) ... ok
test_hardware_joints_missing (__main__.TestComparison) ... ok
test_hardware_joints_reversed (__main__.TestComparison) ... ok
test_nan_radius (__main__.TestComparison) ... ok
test_nan_separation (__main__.TestComparison) ... ok
test_no_right_wheel (__main__.TestComparison) ... ok
test_renamed_joints_and_links (__main__.TestComparison) ... ok
test_shared_transmission (__main__.TestComparison) ... ok
test_unknown_wheel_joint (__main__.TestComparison) ... ok
test_velocity_interface_second (__main__.TestComparison) ... ok
test_wheel_lists (__main__.TestComparison) ... ok
test_wrong_radius (__main__.TestComparison) ... ok
test_wrong_separation (__main__.TestComparison) ... ok
test_robot (__main__.TestModels) ... ok
test_simulation (__main__.TestModels) ... ok

----------------------------------------------------------------------
Ran 16 tests in 2.904s

OK
```

16 tests: 14 check the comparison itself with a small made-up model ([step 4](#4-the-comparisons-own-tests)), and 2 check the robots as their launch files load them, each for DiffBot and Remo.

**When the two disagree:** with Remo's `wheel_separation` changed to 0.145 m on purpose, both model tests fail for Remo with the same message. The first failure:

```console
FAIL: test_robot (__main__.TestModels) (model='remo')
----------------------------------------------------------------------
Traceback (most recent call last):
  File "src/diffbot/diffbot_gazebo/test/test_geometry_consistency.py", line 406, in test_robot
    self.check(model, dump_params('diffbot_base', 'diffbot.launch', model=model))
  File "src/diffbot/diffbot_gazebo/test/test_geometry_consistency.py", line 392, in check
    self.assertEqual(mismatches(urdf, params), [],
AssertionError: Lists differ: ['the wheel joints are 0.1440 m apart, but wheel_separation is 0.1450 m'] != []

First list contains 1 additional elements.
First extra element 0:
'the wheel joints are 0.1440 m apart, but wheel_separation is 0.1450 m'

- ['the wheel joints are 0.1440 m apart, but wheel_separation is 0.1450 m']
+ [] : remo: URDF and controller disagree

======================================================================
```

The first number comes from the URDF, the second from `geometry.yaml`. Don't copy one into the other: measure the robot to find out which one is right ([How to tune](../diffbot_gazebo.md#how-to-tune)).

**In CI**, `catkin build --catkin-make-args run_tests` and `catkin_test_results build` run all of the workspace's tests, the simulation tests with Gazebo included. [diffbot_gazebo's CMakeLists.txt]({{ geometry_src_url }}/diffbot_gazebo/CMakeLists.txt#L221-L224) registers this file with `catkin_add_nosetests`. It runs the file's unittest classes with [nose](https://nose.readthedocs.io/), a Python test runner, and writes the results where `catkin_test_results` counts them.

## How it works

The code excerpts show the files at the diffbot commit that this page pins, with their line numbers. The link above each block opens the same lines on GitHub.

### 1. The launch files supply the values

`TestModels` checks the real robots' configuration, [lines 386–406]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L386-L406):

```python title="test_geometry_consistency.py: TestModels" linenums="386"
--8<-- "{{ geometry_src_raw }}/diffbot_gazebo/test/test_geometry_consistency.py:386:406"
```

It checks each robot in `MODELS`, DiffBot and Remo ([line 43]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L43-L43)). `test_simulation()` combines the parameters of the two launch files that start the simulation: `diffbot_view.launch` loads the URDF, and `diffbot_control.launch` loads the controller's configuration. `test_robot()` takes the parameters of `diffbot.launch`, which loads both on the real robot. `check()` fails when `mismatches()` reports a problem.

`dump_params()` gets the parameters, [lines 60–81]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L60-L81):

```python title="test_geometry_consistency.py: dump_params()" linenums="60"
--8<-- "{{ geometry_src_raw }}/diffbot_gazebo/test/test_geometry_consistency.py:60:81"
```

`roslaunch --dump-params` reads a launch file and the files it includes, and prints the parameters that it would set, as YAML. It starts no node, so these are the values the launch files would load, not values read from a running robot. When roslaunch can't load a file, for example because remo_description is missing, it prints nothing and still ends without an error. So an empty result is an error too.

??? example "The controller's values, and the parameters that roslaunch reports"
    **The controller's values:** each robot's `geometry.yaml` sets them for `diff_drive_controller`. Its comment says where the URDF has the same values.

    === "DiffBot"

        ```yaml title="diffbot_control/config/diffbot/geometry.yaml"
        --8<-- "{{ geometry_src_raw }}/diffbot_control/config/diffbot/geometry.yaml"
        ```

    === "Remo"

        ```yaml title="diffbot_control/config/remo/geometry.yaml"
        --8<-- "{{ geometry_src_raw }}/diffbot_control/config/remo/geometry.yaml"
        ```

    **What roslaunch reports:** `roslaunch --dump-params` reads a launch file and the files it includes, and prints the parameters that it would set, without starting a node. The test reads six of them, highlighted below. For the geometry, it reads `robot_description`, the robot description generated by [xacro](../robot-description.md), which expands the model's XML macros, and `wheel_radius` and `wheel_separation` from `geometry.yaml`. For the joints, it reads the controller's `left_wheel` and `right_wheel`, and the hardware interface's `joints`.

    The outputs below are Remo's, checked in the [dev container](dev-container.md) with diffbot [`c00651e`](https://github.com/ros-mobile-robots/diffbot/commit/c00651e) (`noetic-devel`) and remo_description 2.0.0. Remo's `robot_description` has 912 lines, so it's cut after its first characters here. With `model:=diffbot`, the wheel radius is 0.0333 m and the wheel separation is 0.138 m.

    === "Real robot"

        On the robot, [`diffbot.launch`]({{ diffbot_repo_url }}/diffbot_base/launch/diffbot.launch) loads the URDF and `geometry.yaml` for the hardware interface and the controllers:

        ```console hl_lines="6 7 8 25 38 47 48 51"
        $ roslaunch --dump-params diffbot_base diffbot.launch model:=remo
        /diffbot/debug/base_controller: true
        /diffbot/debug/hardware_interface: false
        /diffbot/encoder_resolution: 542
        /diffbot/gain: 1.0
        /diffbot/hardware_interface/joints:
        - front_left_wheel_joint
        - front_right_wheel_joint
        /diffbot/joint_state_controller/extra_joints:
        - effort: 0.0
          name: rear_caster_rotation_joint
          position: 0.01
          velocity: 0.0
        - effort: 0.0
          name: rear_caster_wheel_joint
          position: 0.01
          velocity: 0.0
        /diffbot/joint_state_controller/publish_rate: 50
        /diffbot/joint_state_controller/type: joint_state_controller/JointStateController
        /diffbot/mobile_base_controller/angular/z/has_acceleration_limits: true
        /diffbot/mobile_base_controller/angular/z/has_velocity_limits: true
        /diffbot/mobile_base_controller/angular/z/max_acceleration: 6.0
        /diffbot/mobile_base_controller/angular/z/max_velocity: 8.0
        /diffbot/mobile_base_controller/base_frame_id: base_footprint
        /diffbot/mobile_base_controller/left_wheel: front_left_wheel_joint
        /diffbot/mobile_base_controller/linear/x/has_acceleration_limits: true
        /diffbot/mobile_base_controller/linear/x/has_velocity_limits: true
        /diffbot/mobile_base_controller/linear/x/max_acceleration: 0.6
        /diffbot/mobile_base_controller/linear/x/max_velocity: 0.3
        /diffbot/mobile_base_controller/pose_covariance_diagonal:
        - 0.001
        - 0.001
        - 0.001
        - 0.001
        - 0.001
        - 0.03
        /diffbot/mobile_base_controller/publish_rate: 50
        /diffbot/mobile_base_controller/right_wheel: front_right_wheel_joint
        /diffbot/mobile_base_controller/twist_covariance_diagonal:
        - 0.001
        - 0.001
        - 0.001
        - 0.001
        - 0.001
        - 0.03
        /diffbot/mobile_base_controller/type: diff_drive_controller/DiffDriveController
        /diffbot/mobile_base_controller/wheel_radius: 0.033
        /diffbot/mobile_base_controller/wheel_separation: 0.144
        /diffbot/motor_constant: 1.0
        /diffbot/pwm_limit: 1.0
        /diffbot/robot_description: "<?xml version=\"1.0\" ?>\n<!-- …
        /diffbot/trim: 0.0
        ```

    === "Simulation"

        In the simulation, two launch files share the work. [`diffbot_view.launch`]({{ diffbot_repo_url }}/diffbot_gazebo/launch/diffbot_view.launch) loads the URDF for Gazebo, and [`diffbot_control.launch`]({{ diffbot_repo_url }}/diffbot_control/launch/diffbot_control.launch) loads `geometry.yaml`. The test combines their parameters:

        ```console hl_lines="2 16 17 18 35 48 57 58"
        $ roslaunch --dump-params diffbot_gazebo diffbot_view.launch model:=remo
        /diffbot/robot_description: "<?xml version=\"1.0\" ?>\n<!-- …
        /gazebo/enable_ros_network: true
        /use_sim_time: true
        $ roslaunch --dump-params diffbot_control diffbot_control.launch model:=remo rqt_robot_steering:=false
        /diffbot/gazebo_ros_control/pid_gains/front_left_wheel_joint/antiwindup: true
        /diffbot/gazebo_ros_control/pid_gains/front_left_wheel_joint/d: 0.0
        /diffbot/gazebo_ros_control/pid_gains/front_left_wheel_joint/i: 0.6
        /diffbot/gazebo_ros_control/pid_gains/front_left_wheel_joint/i_clamp: 0.05
        /diffbot/gazebo_ros_control/pid_gains/front_left_wheel_joint/p: 0.03
        /diffbot/gazebo_ros_control/pid_gains/front_right_wheel_joint/antiwindup: true
        /diffbot/gazebo_ros_control/pid_gains/front_right_wheel_joint/d: 0.0
        /diffbot/gazebo_ros_control/pid_gains/front_right_wheel_joint/i: 0.6
        /diffbot/gazebo_ros_control/pid_gains/front_right_wheel_joint/i_clamp: 0.05
        /diffbot/gazebo_ros_control/pid_gains/front_right_wheel_joint/p: 0.03
        /diffbot/hardware_interface/joints:
        - front_left_wheel_joint
        - front_right_wheel_joint
        /diffbot/joint_state_controller/extra_joints:
        - effort: 0.0
          name: rear_caster_rotation_joint
          position: 0.01
          velocity: 0.0
        - effort: 0.0
          name: rear_caster_wheel_joint
          position: 0.01
          velocity: 0.0
        /diffbot/joint_state_controller/publish_rate: 50
        /diffbot/joint_state_controller/type: joint_state_controller/JointStateController
        /diffbot/mobile_base_controller/angular/z/has_acceleration_limits: true
        /diffbot/mobile_base_controller/angular/z/has_velocity_limits: true
        /diffbot/mobile_base_controller/angular/z/max_acceleration: 6.0
        /diffbot/mobile_base_controller/angular/z/max_velocity: 8.0
        /diffbot/mobile_base_controller/base_frame_id: base_footprint
        /diffbot/mobile_base_controller/left_wheel: front_left_wheel_joint
        /diffbot/mobile_base_controller/linear/x/has_acceleration_limits: true
        /diffbot/mobile_base_controller/linear/x/has_velocity_limits: true
        /diffbot/mobile_base_controller/linear/x/max_acceleration: 0.6
        /diffbot/mobile_base_controller/linear/x/max_velocity: 0.3
        /diffbot/mobile_base_controller/pose_covariance_diagonal:
        - 0.001
        - 0.001
        - 0.001
        - 0.001
        - 0.001
        - 0.03
        /diffbot/mobile_base_controller/publish_rate: 50
        /diffbot/mobile_base_controller/right_wheel: front_right_wheel_joint
        /diffbot/mobile_base_controller/twist_covariance_diagonal:
        - 0.001
        - 0.001
        - 0.001
        - 0.001
        - 0.001
        - 0.03
        /diffbot/mobile_base_controller/type: diff_drive_controller/DiffDriveController
        /diffbot/mobile_base_controller/wheel_radius: 0.033
        /diffbot/mobile_base_controller/wheel_separation: 0.144
        ```

### 2. The wheel joints

A joint connects two links of the URDF. A wheel joint is where the wheel turns. The controller doesn't know the wheels by their size, but by the names of these joints, in `diffbot_control.yaml`, [lines 27–28]({{ geometry_src_url }}/diffbot_control/config/diffbot_control.yaml#L27-L28):

```yaml title="diffbot_control.yaml: the controller's wheels" linenums="27"
--8<-- "{{ geometry_src_raw }}/diffbot_control/config/diffbot_control.yaml:27:28"
```

`wheel_joint_names()` reads them, [lines 149–155]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L149-L155). diff_drive_controller takes a name or a list of names for each side:

```python title="test_geometry_consistency.py: wheel_joint_names()" linenums="149"
--8<-- "{{ geometry_src_raw }}/diffbot_gazebo/test/test_geometry_consistency.py:149:155"
```

`mismatches()` compares everything and collects the problems as messages. Its first checks look for these joints in the URDF, [lines 197–219]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L197-L219):

```python title="test_geometry_consistency.py: mismatches(), the joints" linenums="197"
--8<-- "{{ geometry_src_raw }}/diffbot_gazebo/test/test_geometry_consistency.py:197:219"
```

`TOLERANCE`, `CONTROLLER`, `HARDWARE_JOINTS` and `VELOCITY_INTERFACES` are defined at the top of the file, [lines 43–52]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L43-L52). In DiffBot's model, the wheel macro creates each joint, [lines 89–94]({{ geometry_src_url }}/diffbot_description/urdf/include/common_macros.urdf.xacro#L89-L94). xacro replaces `${prefix}` with `front_left` or `front_right`, and the test reads the expanded URDF:

```xml title="common_macros.urdf.xacro: the wheel joint" linenums="89"
--8<-- "{{ geometry_src_raw }}/diffbot_description/urdf/include/common_macros.urdf.xacro:89:94"
```

A `continuous` joint turns without limits, as a wheel does. Remo's model has the same macro in [remo_description](https://github.com/ros-mobile-robots/remo_description/blob/2.0.0/urdf/include/common_macros.urdf.xacro).

**The velocity interface.** In ros_control, a hardware interface is how a controller reads and commands a joint. diff_drive_controller commands each wheel's velocity, so it needs a velocity interface. In the simulation, Gazebo's ros_control plugin gives it one for each joint from the joint's transmission in the URDF, [lines 96–105]({{ geometry_src_url }}/diffbot_description/urdf/include/common_macros.urdf.xacro#L96-L105):

```xml title="common_macros.urdf.xacro: the wheel's transmission" linenums="96" hl_lines="98 99 100"
--8<-- "{{ geometry_src_raw }}/diffbot_description/urdf/include/common_macros.urdf.xacro:96:105"
```

The plugin, `DefaultRobotHWSim`, [skips a transmission with more than one joint](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/cd9cc952402375be5f274f39b29399c4087aa839/gazebo_ros_control/src/default_robot_hw_sim.cpp#L90-L103) and [uses the first `hardwareInterface` of the joint](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/cd9cc952402375be5f274f39b29399c4087aa839/gazebo_ros_control/src/default_robot_hw_sim.cpp#L124-L146). `transmission_interfaces()` reads the transmissions the same way, [lines 168–185]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L168-L185):

```python title="test_geometry_consistency.py: transmission_interfaces()" linenums="168"
--8<-- "{{ geometry_src_raw }}/diffbot_gazebo/test/test_geometry_consistency.py:168:185"
```

`mismatches()` uses the result, [lines 220–226]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L220-L226):

```python title="test_geometry_consistency.py: mismatches(), the transmissions" linenums="220"
--8<-- "{{ geometry_src_raw }}/diffbot_gazebo/test/test_geometry_consistency.py:220:226"
```

The test checks the configuration these robots need, not every transmission that Gazebo accepts.

**The hardware interface's joints.** On the real robot, diffbot_base's hardware interface reads its joints from the same file, [lines 1–6]({{ geometry_src_url }}/diffbot_control/config/diffbot_control.yaml#L1-L6):

```yaml title="diffbot_control.yaml: the hardware interface's joints" linenums="1"
--8<-- "{{ geometry_src_raw }}/diffbot_control/config/diffbot_control.yaml:1:6"
```

They must be the controller's wheels, the left one first, [lines 232–237]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L232-L237):

```python title="test_geometry_consistency.py: mismatches(), the order" linenums="232"
--8<-- "{{ geometry_src_raw }}/diffbot_gazebo/test/test_geometry_consistency.py:232:237"
```

The simulation loads the same list without using it, so there it's a check of the shared configuration.

??? info "Why the order matters on the robot"
    The hardware interface registers each joint in the order of the list, [lines 82–99]({{ geometry_src_url }}/diffbot_base/src/diffbot_hw_interface.cpp#L82-L99):

    ```cpp title="diffbot_hw_interface.cpp: registering the joints" linenums="82"
    --8<-- "{{ geometry_src_raw }}/diffbot_base/src/diffbot_hw_interface.cpp:82:99"
    ```

    It sends the wheel velocity commands to the Teensy in the same order, [lines 178–186]({{ geometry_src_url }}/diffbot_base/src/diffbot_hw_interface.cpp#L178-L186):

    ```cpp title="diffbot_hw_interface.cpp: the wheel commands" linenums="178"
    --8<-- "{{ geometry_src_raw }}/diffbot_base/src/diffbot_hw_interface.cpp:178:186"
    ```

    The firmware takes the first value for the left wheel, [lines 472–478]({{ geometry_src_url }}/diffbot_firmware/lib/base_controller/base_controller.h#L472-L478):

    ```cpp title="base_controller.h: commandCallback()" linenums="472"
    --8<-- "{{ geometry_src_raw }}/diffbot_firmware/lib/base_controller/base_controller.h:472:478"
    ```

    With the list reversed, the left wheel's commands would drive the right motor.

### 3. Radius and separation

`collision_radius()` reads a wheel link's collision shape, a sphere or a cylinder, [lines 158–165]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L158-L165):

```python title="test_geometry_consistency.py: collision_radius()" linenums="158"
--8<-- "{{ geometry_src_raw }}/diffbot_gazebo/test/test_geometry_consistency.py:158:165"
```

`mismatches()` compares it with `wheel_radius` for every wheel, [lines 227–231]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L227-L231):

```python title="test_geometry_consistency.py: mismatches(), the radius" linenums="227"
--8<-- "{{ geometry_src_raw }}/diffbot_gazebo/test/test_geometry_consistency.py:227:231"
```

`within_tolerance()` checks that the difference is at most 0.1 mm, [lines 188–194]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L188-L194). It checks "within" rather than "too large", because a NaN, a value that isn't a number, compares false with everything, so it would never be too large:

```python title="test_geometry_consistency.py: within_tolerance()" linenums="188"
--8<-- "{{ geometry_src_raw }}/diffbot_gazebo/test/test_geometry_consistency.py:188:194"
```

**The joints' positions.** Each joint's origin in the URDF is given in its parent link's frame. To compare the two wheel joints, `joint_position()` moves a joint's position up the tree into the frame of the URDF's root link, `base_footprint`, [lines 129–146]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L129-L146):

```python title="test_geometry_consistency.py: joint_position()" linenums="129"
--8<-- "{{ geometry_src_raw }}/diffbot_gazebo/test/test_geometry_consistency.py:129:146"
```

Each step rotates the point into the parent's frame and adds the offset of the joint between them, [lines 122–126]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L122-L126):

```python title="test_geometry_consistency.py: transform()" linenums="122"
--8<-- "{{ geometry_src_raw }}/diffbot_gazebo/test/test_geometry_consistency.py:122:126"
```

For Remo's left wheel joint, none of the joints on the way is rotated, so only the offsets add up:

| Position in the frame of | Step | Position (m) |
|:--|:--|:--|
| `chassis_link` | the wheel joint's origin | (0.036, 0.072, −0.0336) |
| `base_link` | plus the chassis joint's origin (−0.036, 0, 0) | (0, 0.072, −0.0336) |
| `base_footprint` | plus the base link joint's origin (0, 0, 0.0666) | (0, 0.072, 0.033) |

The right wheel joint ends at (0, −0.072, 0.033), 0.144 m away. `joint_origin()` reads an origin's position and its roll, pitch and yaw, [lines 112–119]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L112-L119), and `rotation_matrix()` turns them into a rotation, [lines 92–109]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L92-L109). The last part of `mismatches()` compares the distance with `wheel_separation`, [lines 238–247]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L238-L247):

```python title="test_geometry_consistency.py: mismatches(), the separation" linenums="238"
--8<-- "{{ geometry_src_raw }}/diffbot_gazebo/test/test_geometry_consistency.py:238:247"
```

diff_drive_controller takes the separation of the first wheel on each side, so the test does too.

### 4. The comparison's own tests

`TestComparison` checks that `mismatches()` notices each kind of problem. It uses a small URDF with Remo's wheels, [lines 253–280]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L253-L280), and parameters that agree with it, [lines 282–290]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L282-L290):

```python title="test_geometry_consistency.py: TestComparison.controller_params()" linenums="282"
--8<-- "{{ geometry_src_raw }}/diffbot_gazebo/test/test_geometry_consistency.py:282:290"
```

Each test changes one thing and checks the message, for example a wrong separation, [lines 304–307]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py#L304-L307):

```python title="test_geometry_consistency.py: TestComparison.test_wrong_separation()" linenums="304"
--8<-- "{{ geometry_src_raw }}/diffbot_gazebo/test/test_geometry_consistency.py:304:307"
```

The others: a wrong radius, a list of wheels instead of a name, an unknown wheel joint, renamed joints and links, an effort interface instead of a velocity interface, the velocity interface second, both wheels in one transmission, a NaN radius and separation, no right wheel, and the hardware interface's joints reversed or missing.
