# IMU Simulation: The Robots' Own Plugin

In Gazebo, DiffBot and Remo load their own IMU plugin, not the plugin that comes with ROS. This page explains why, and what the robots' plugin does differently. The page belongs to [IMU: Simulation](index.md). The page is background for the interested reader: you don't need the page to use the simulated IMU.

## Why the robots have their own plugin

In Gazebo, a plugin turns the IMU sensor's values into ROS messages. ROS comes with one, [`libgazebo_ros_imu_sensor.so`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp) from the package [gazebo_plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/2.9.3/gazebo_plugins). The robots load [`libdiffbot_gazebo_ros_imu_sensor.so`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp) instead. This library is the diffbot repository's copy of that plugin, taken from [gazebo_plugins 2.9.3](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp), the version that the dev container installs. The package [`diffbot_gazebo_plugins`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins) builds the copy from `gazebo_ros_imu_sensor.cpp`.

The copy takes the same parameters, computes the same values and publishes the same message as the original. Two differences matter while the simulation runs:

1. **The copy doesn't call [`ros::spinOnce()`](#one-request-with-each-plugin):** with the original, this call can stop the simulation's clock for good while the controllers start. [Why the plugin doesn't spin](#why-the-plugin-doesnt-spin) explains how.
2. **The copy publishes again right after a reset of the simulation:** the original stays silent for as long as the simulation ran before the reset. [Publishing again after a reset](#publishing-again-after-a-reset) explains why.

[The copy's other changes](#the-copys-other-changes) don't change what the plugin publishes.

## Why the plugin doesn't spin

### The short answer

In Gazebo, one thread moves the simulation's clock forward: the physics thread. With [gazebo_plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/2.9.3/gazebo_plugins)' original IMU plugin, this thread can end up waiting for something that only it can do. Then nothing moves the clock anymore.

The original plugin calls [`ros::spinOnce()`](#one-request-with-each-plugin) right after it publishes a message. This call makes the physics thread run the service requests that wait in the node's global callback queue at that moment. The controller spawner's requests wait there. Such a request then waits for the controller manager's next update, and only the physics thread runs that update. So the physics thread waits for itself, and the simulation's clock stops for good. The robots' copy leaves the call out, because a plugin that only publishes has nothing to spin for.

[diffbot#185](https://github.com/ros-mobile-robots/diffbot/issues/185) has the diagnosis, with the backtraces of the stalled simulations.

### One request, with each plugin

The two drawings follow the same request of the controller spawner, once with each plugin. Switch between the two tabs to compare them: what both runs share stays in place. The blue path is the request, and the box named [`update()`](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L66) at the bottom of the left lane is the update that the request waits for. The numbered steps below each drawing explain its numbers. [The terms in the drawings](#the-terms-in-the-drawings) explains threads, callbacks and the callback queue in more detail.

--8<-- "includes/imu-sim-threads.html"

=== "With the original plugin"

    1. **The request arrives:** the controller spawner calls the controller manager's service [`switch_controller`](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L60). The spawner is a separate ROS node, which starts the robot's controllers. ROS doesn't run a service's function, its **callback**, at the moment the request arrives. The request waits in the Gazebo server's **global callback queue**, a waiting list, until a thread takes the request out and runs the callback.
    2. **The physics thread publishes and spins:** before a ROS spinner thread takes the request, the physics thread starts a simulation step and calls the plugin's [`UpdateChild()`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L92-L154). The plugin publishes its message and calls [`ros::spinOnce()`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L150).
    3. **The callback runs in the physics thread:** `ros::spinOnce()` runs the callbacks that wait in the global queue at that moment, in the thread that calls it. So `ros::spinOnce()` finds the request and runs the service's callback, in the physics thread.
    4. **The callback waits:** the callback notes which controllers to start, and waits until the controller manager's `update()` has taken the change.
    5. **Nothing can end the wait:** `update()` runs only in the physics thread. But the physics thread is still inside `UpdateChild()`, waiting in the callback. The physics thread waits for itself. This is a **deadlock**: `update()` never runs, the callback never returns, and the simulation step never ends. The simulation's clock stops for good, and the spawner never gets its answer.

=== "With the copy"

    1. **The request arrives:** the request waits in the global callback queue, as with the original plugin.
    2. **The physics thread publishes and returns:** the physics thread calls `UpdateChild()`. The copy publishes its message and returns, as [the header and publishing](measurement-to-message.md#the-header-and-publishing) shows. So the physics thread never runs a callback.
    3. **The callback runs in a spinner thread:** a ROS spinner thread takes the request and runs the callback. The callback notes the change and waits, in the spinner thread.
    4. **The physics thread goes on:** it calls `update()`, in this step or the next, and `update()` takes the change. The clock runs on.
    5. **The wait ends:** the callback returns, and the spawner gets its answer. The controllers run.

**Timing:** the stall depends on timing. The stall needs a request of the spawner in the queue at the moment that the original plugin spins, before a spinner thread has taken the request. So the stall hits only some starts of the simulation. [diffbot#185](https://github.com/ros-mobile-robots/diffbot/issues/185) counts how many starts stalled with the original and with the copy.

**No spinning in the copy:** the copy needs no spinning of its own, because it only publishes and has no callbacks. The spinner threads run the calls that wait in the global queue, the controller manager's among them.

The three collapsed boxes show the lines of code behind steps 2 and 4.

??? example "Step 2: the original plugin calls `ros::spinOnce()` after it publishes"

    The excerpt shows the end of the original plugin's `UpdateChild()`, from [`gazebo_ros_imu_sensor.cpp` in gazebo_plugins 2.9.3, lines 137–153](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L137-L153), under the Apache License 2.0. Line 146 publishes the message, and line 150, the highlighted one, spins the global queue in the thread that called `UpdateChild()`:

    ```cpp title="gazebo_ros_imu_sensor.cpp (gazebo_plugins 2.9.3): publish(), then ros::spinOnce()" linenums="137" hl_lines="14"
    --8<-- "https://raw.githubusercontent.com/ros-simulation/gazebo_ros_pkgs/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp:137:153"
    ```

??? example "Step 4: the callback of `switch_controller` waits for `update()`"

    The excerpt shows the wait in the controller manager's [`switchController()`](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L605-L636), from [`controller_manager.cpp` in ros_control 0.20.0, lines 605–636](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L605-L636), under the BSD license. Line 610 notes the change by setting [`switch_params_.do_switch`](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L610). The loop from line 616 on, the highlighted line, sleeps until `update()` has set `do_switch` back to false. With [`timeout == 0`](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L623), line 623 never lets the loop give up:

    ```cpp title="controller_manager.cpp (ros_control 0.20.0): the wait of switch_controller" linenums="605" hl_lines="12"
    --8<-- "https://raw.githubusercontent.com/ros-controls/ros_control/0.20.0/controller_manager/src/controller_manager.cpp:605:636"
    ```

??? example "Step 4: the callback of `load_controller` waits for `update()` too"

    The excerpt shows the wait in the controller manager's [`loadController()`](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L349-L359), from [`controller_manager.cpp` in ros_control 0.20.0, lines 349–359](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L349-L359). The callback switches to a new list of controllers and sleeps until `update()`, which the comment calls the realtime thread, no longer uses the old list. This loop has no time limit:

    ```cpp title="controller_manager.cpp (ros_control 0.20.0): the wait of load_controller" linenums="349"
    --8<-- "https://raw.githubusercontent.com/ros-controls/ros_control/0.20.0/controller_manager/src/controller_manager.cpp:349:359"
    ```

### The terms in the drawings

This section explains three terms, each with an example from the simulation, and one fact about the controller manager.

**Threads:** a program can do several things at the same time, each in a **thread**. A thread runs one function after the other. While a thread is inside a function that waits, it can't do anything else. The threads of one program share the program's data.

*Example:* the Gazebo server, [`gzserver`](https://classic.gazebosim.org/tutorials?tut=components), has a **physics thread**, which computes the simulation step by step. Each step moves the simulation's clock forward by 1 ms, the [`max_step_size`]({{ imu_sim_src_url }}/diffbot_gazebo/worlds/diffbot_empty.world#L15) in [line 15 of the test's world file]({{ imu_sim_src_url }}/diffbot_gazebo/worlds/diffbot_empty.world#L15). At the start of every step, the physics thread calls the update functions of the plugins, one after the other. Two of them matter here:

- [`UpdateChild()`](measurement-to-message.md#part-2-the-plugin-makes-the-message) of the IMU plugin, which publishes the IMU message.
- [`update()`](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L66) of the controller manager, which runs the controllers, such as the drive controller. The plugin [`gazebo_ros_control`](../../../development/simulation-test.md#what-runs-in-the-simulation) calls `update()`, in [line 251 of its source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros_control/src/gazebo_ros_control_plugin.cpp#L251). [What runs in the simulation](../../../development/simulation-test.md#what-runs-in-the-simulation) on Simulation Test shows how these parts work together.

Only the physics thread calls these two functions. And the simulation's clock moves on only when the physics thread finishes a step.

**Callbacks:** a **callback** is a function that a program gives to ROS, for ROS to run when something arrives: a message on a topic, or a request to a service.

*Example:* the controller manager offers the service [`switch_controller`](#one-request-with-each-plugin), which starts and stops controllers. The service's callback is the function that does this work. When another node calls the service, the callback runs, and the caller gets its answer when the callback returns.

**The callback queue:** ROS doesn't run a callback at the moment its request arrives. ROS puts the call into a **callback queue**, a waiting list. The queue itself runs nothing, and it isn't a thread. A thread has to take each call out of the queue and run it. ROS calls this **spinning**. The thread that spins is the thread that runs the callback.

*Example:* the Gazebo server is one ROS node, [`/gazebo`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros/src/gazebo_ros_api_plugin.cpp#L126). A node has a global callback queue. A service's calls wait there, unless the service was given a queue of its own. The controller manager's services use the global queue. The `gazebo_ros_control` plugin [passes its ordinary node handle](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros_control/src/gazebo_ros_control_plugin.cpp#L150-L194) to the controller manager. A node handle is how a ROS program offers topics and services. The controller manager [registers its services](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L45-L61) through that handle.

Two things can run the calls that wait in the global queue:

- **The ROS spinner threads** do nothing else. Each takes a waiting call out of the global queue and runs it. Gazebo's ROS interface, the plugin [`gazebo_ros_api_plugin`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros/src/gazebo_ros_api_plugin.cpp), starts one such thread per CPU core with a [`ros::AsyncSpinner`](https://docs.ros.org/en/noetic/api/roscpp/html/classros_1_1AsyncSpinner.html), in [line 148 of its source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros/src/gazebo_ros_api_plugin.cpp#L148). The [`gzserver`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros/scripts/gzserver#L21-L24) script of [`gazebo_ros`](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/2.9.3/gazebo_ros) loads this plugin into the server.
- **[`ros::spinOnce()`]({{ ros_comm_src_url }}/clients/roscpp/src/libros/init.cpp#L585)** spins the global queue once, in the thread that calls it. It runs the callbacks that wait in the global queue at that moment, and returns when they have returned. The ROS wiki describes [single-threaded spinning](http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning#Single-threaded_Spinning). In roscpp, `spinOnce()` calls [`callAvailable()`]({{ ros_comm_src_url }}/clients/roscpp/src/libros/init.cpp#L585-L588) on the global queue.

Not every service of the Gazebo server uses the global queue. Gazebo's ROS interface [registers its own services on a separate queue](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros/src/gazebo_ros_api_plugin.cpp#L545-L552), such as [`/gazebo/reset_simulation`](#publishing-again-after-a-reset). Its [`gazeboQueueThread()`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros/src/gazebo_ros_api_plugin.cpp#L257-L263) processes that queue. Neither the spinner threads nor `ros::spinOnce()` run it, and the rest of this explanation doesn't need it.

**The fact about the controller manager:** its services wait for `update()`. The controller spawner is a separate ROS node, which [`diffbot_control.launch`]({{ imu_sim_src_url }}/diffbot_control/launch/diffbot_control.launch#L39-L41) starts. The spawner loads and starts the controllers with two of the controller manager's services, [`load_controller`](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L58) and `switch_controller`. Their callbacks don't change the running controllers themselves, because `update()` may be using the controllers at that moment. The callbacks note the change and wait until `update()` has taken it, as the two collapsed boxes of [step 4](#one-request-with-each-plugin) show. `switch_controller` can give up after a time limit. The spawner asks for none: it passes 0 as the last argument, in [line 212 of its source](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/scripts/spawner#L212).

### Why not just wait?

The deadlock comes from the physics thread running a callback that waits for the physics thread. The first three ideas below wait differently and don't remove that dependency. The fourth removes it in another place:

- **A delay, or starting Gazebo paused:** a delay at the start might change which thread takes the spawner's request, but the unsafe path stays. Starting Gazebo paused doesn't help either. The controllers start only when [`update()`](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L66) takes the change, and the physics thread calls `update()` only while the simulation runs. So the request waits for the simulation to run, and then the same path is open. Later requests to the controller manager can take this path too.
- **A condition variable** lets a thread sleep until another thread signals it. The callback already waits. Waiting on a condition variable instead of polling would still block the physics thread, which is the only thread that can run `update()`.
- **A timeout:** with a positive timeout, `switch_controller` can return a failure about one second after the timeout expires. The wait in [step 4](#one-request-with-each-plugin) shows the check and the return. The spawner passes 0, which turns the timeout off. `load_controller` has no timeout at all. And a failed switch wouldn't start the controllers anyway.
- **A separate queue for the controller manager**, served by a thread of its own, would also keep the callbacks out of the physics thread. That would require changing the [ordinary node handle](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros_control/src/gazebo_ros_control_plugin.cpp#L149-L150) that [`gazebo_ros_control`](../../../development/simulation-test.md#what-runs-in-the-simulation) gives the controller manager.

The IMU plugin only publishes, so it has no callbacks of its own to run. Leaving out `ros::spinOnce()` is the smallest change that keeps the physics thread out of the queue. The packaged plugin has no option to turn off this call. The robots load their own library, so the fix doesn't require replacing [gazebo_plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/2.9.3/gazebo_plugins)' installed library.

## Publishing again after a reset

A reset of the simulation sets Gazebo's clock back to zero. [`/gazebo/reset_simulation`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros/src/gazebo_ros_api_plugin.cpp#L546) is the ROS service for a reset, one of [Gazebo's simulation control services](https://classic.gazebosim.org/tutorials?tut=ros_comm#Services:Simulationcontrol). For example: `rosservice call /gazebo/reset_simulation`.

Both plugins limit their rate. They compare the IMU sensor's update time, [`current_time`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L97) in the code, with [`last_time`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L153), the time of their last update:

- **The original** has only this rate limit. After a reset, `current_time` starts from zero again, and `last_time` still holds a time from before the reset. The difference of the two is negative, so the rate limit stops every call until the clock has passed its value from before the reset. The longer the simulation ran before the reset, the longer the IMU stays silent.
- **The copy** sets `last_time` to `current_time` when `current_time` is the earlier one. The rate limit then counts from the new time, and the plugin can publish again one period, 20 ms, later. [The checks before a call publishes](measurement-to-message.md#the-checks-before-a-call-publishes) shows these lines.

The excerpt shows the original's rate limit, from [`gazebo_ros_imu_sensor.cpp` in gazebo_plugins 2.9.3, lines 97–100](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L97-L100). With a `last_time` of 3 s and a `current_time` of 0.02 s, the difference in line 99 is −2.98 s, which is less than one period, so line 100 returns:

```cpp title="gazebo_ros_imu_sensor.cpp (gazebo_plugins 2.9.3): the rate limit" linenums="97"
--8<-- "https://raw.githubusercontent.com/ros-simulation/gazebo_ros_pkgs/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp:97:100"
```

The IMU sample validator and the EKF start over when the clock goes back, as [the IMU sample validator](../robot_localization.md#the-imu-sample-validator) and [the EKF's limits](../robot_localization.md#limits) on the Localization page explain. So both need the IMU's samples right after a reset. [The reset test](tests.md#the-reset-test) checks that the samples come.

The [`gazebo_ros_p3d`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_p3d.cpp) plugin of gazebo_plugins also [resets its stored time](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_p3d.cpp#L240-L244) when the clock goes back, as the copy does. `gazebo_ros_p3d` publishes a link's true pose and velocity in the simulation, as an odometry message. This page names `gazebo_ros_p3d` only as an example of the same handling. `gazebo_ros_p3d` can't stand in for the IMU plugin: it reads no IMU sensor, and it publishes no acceleration and no [`sensor_msgs/Imu`](index.md#what-the-simulated-imu-publishes) message.

## The plugin's library in Remo's URDF model

DiffBot's macro names the plugin's library itself, in [line 246 of `common_sensors.xacro`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L246). Remo's [xacro](position-and-axes.md#the-link-and-the-joint-in-the-robots-urdf-model) file takes the library's name as the xacro argument [`imu_plugin`](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/remo.urdf.xacro#L36) instead. [The default of Remo's `imu_plugin` argument](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/remo.urdf.xacro#L29-L36) is the library from [gazebo_plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/2.9.3/gazebo_plugins). Remo's xacro file [passes the argument](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/remo.urdf.xacro#L211-L216) to its IMU macro. The diffbot repository's simulation sets the argument when it runs xacro to make the robot's URDF model. The excerpt shows the xacro command, from [`diffbot_view.launch`, lines 6–11]({{ imu_sim_src_url }}/diffbot_gazebo/launch/diffbot_view.launch#L6-L11):

```xml title="diffbot_view.launch: the xacro command for the robot's URDF model" linenums="6" hl_lines="6"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/launch/diffbot_view.launch:6:11"
```

[`urdf_file`]({{ imu_sim_src_url }}/diffbot_gazebo/launch/diffbot_view.launch#L11) is the xacro command for the selected robot's URDF model. The command passes `imu_plugin:=libdiffbot_gazebo_ros_imu_sensor.so` for both robots. Remo's xacro file uses the value as the plugin's [`filename`](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/include/common_sensors.xacro#L256). DiffBot's xacro file doesn't declare `imu_plugin`, and xacro ignores the argument. So both robots load the same plugin in this simulation, and Remo's URDF model still works on its own, with its default library.

## The copy's other changes

The [change notice]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L17-L42) at the top of the file lists every change. Besides the two differences above, the copy has these changes, which don't change what the plugin publishes:

- The destructor uses the plugin's node handle only when [`Load()`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L80) has created it. `Load()` returns before that when the sensor isn't an IMU sensor, when [`frameName`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L240-L248) is missing or when ROS isn't initialized. The original then calls the node handle's [`shutdown()`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L272) through a pointer that was never set.
- The class is in the namespace [`diffbot_gazebo_plugins`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L68). So a Gazebo process that loads both libraries can't mix up this class with [gazebo_plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/2.9.3/gazebo_plugins)' class of the same name, as [the header's notice]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/include/diffbot_gazebo_plugins/gazebo_ros_imu_sensor.h#L17-L25) says.
- The optional profiler calls are left out. This package doesn't enable them.
- The lines are at most 99 characters long, and the file includes the header of every name it uses.

The source is under the Apache License 2.0, as the original is. The copy keeps the original's notices, and the package installs the license's text, as its [`package.xml`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/package.xml) and [`CMakeLists.txt`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/CMakeLists.txt) show. Gazebo looks for the library's file name on [`GAZEBO_PLUGIN_PATH`](https://classic.gazebosim.org/tutorials?tut=components) and then on [`LD_LIBRARY_PATH`](https://man7.org/linux/man-pages/man8/ld.so.8.html). The workspace's [`setup.bash`](https://github.com/ros/catkin/blob/0.8.12/doc/user_guide/environment.rst#L3-L10) puts the workspace's library folder on `LD_LIBRARY_PATH`, as [the comment in `CMakeLists.txt`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/CMakeLists.txt#L23-L25) says.
