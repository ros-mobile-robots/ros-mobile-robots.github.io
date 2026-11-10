# IMU Simulation: The Robots' Own Plugin

Why DiffBot and Remo load their own IMU plugin in Gazebo, and what it does differently from the plugin that comes with ROS. This page belongs to [IMU: Simulation](index.md). It's background for the interested reader: you don't need it to use the simulated IMU.

## Why the robots have their own plugin

In Gazebo, a plugin turns the IMU sensor's values into ROS messages. ROS comes with one, `libgazebo_ros_imu_sensor.so` from the package gazebo_plugins. The robots load `libdiffbot_gazebo_ros_imu_sensor.so` instead. This library is the diffbot repository's copy of that plugin, taken from [gazebo_plugins 2.9.3](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp), the version that the dev container installs. The package [`diffbot_gazebo_plugins`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins) builds the copy from [`gazebo_ros_imu_sensor.cpp`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp).

The copy takes the same parameters, computes the same values and publishes the same message as the original. Two differences matter while the simulation runs:

1. **The copy doesn't call `ros::spinOnce()`.** With the original, this call can stop the simulation's clock for good while the controllers start. [Why the plugin doesn't spin](#why-the-plugin-doesnt-spin) explains how.
2. **The copy publishes again right after a reset of the simulation.** The original stays silent for as long as the simulation ran before the reset. [Publishing again after a reset](#publishing-again-after-a-reset) explains why.

[The copy's other changes](#the-copys-other-changes) don't change what the plugin publishes.

## Why the plugin doesn't spin

### The short answer

gazebo_plugins' original IMU plugin calls `ros::spinOnce()` right after it publishes a message. This call makes Gazebo's physics thread, the thread that moves the simulation forward, run whatever ROS service request waits at that moment. A request of the controller spawner then waits for the controller manager's next update, and only the physics thread runs that update. So the physics thread waits for itself, and the simulation's clock stops for good. The robots' copy leaves the call out, because a plugin that only publishes has nothing to spin for.

[diffbot#185](https://github.com/ros-mobile-robots/diffbot/issues/185) has the diagnosis, with the backtraces of the stalled simulations. The rest of this section explains the terms and then follows one request, step by step.

### What to know first

Three terms, each with an example from the simulation, and one fact about the controller manager.

**Threads.** A program can do several things at the same time, each in a **thread**. A thread runs one function after the other. While a thread is inside a function that waits, it can't do anything else. The threads of one program share the program's data.

*Example:* the Gazebo server, `gzserver`, has a **physics thread**, which computes the simulation step by step. Each step moves the simulation's clock forward by 1 ms, the `max_step_size` in [line 15 of the test's world file]({{ imu_sim_src_url }}/diffbot_gazebo/worlds/diffbot_empty.world#L15). At the start of every step, the physics thread calls the update functions of the plugins, one after the other. Two of them matter here:

- `UpdateChild()` of the IMU plugin, which publishes the IMU message.
- `update()` of the controller manager, which runs the controllers, such as the drive controller. The plugin `gazebo_ros_control` calls `update()`, in [line 251 of its source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros_control/src/gazebo_ros_control_plugin.cpp#L251). [What runs in the simulation](../../../development/simulation-test.md#what-runs-in-the-simulation) on Simulation Test shows how these parts work together.

Only the physics thread calls these two functions. And the simulation's clock moves on only when the physics thread finishes a step.

**Callbacks.** A **callback** is a function that a program gives to ROS, for ROS to run when something arrives: a message on a topic, or a request to a service.

*Example:* the controller manager offers the service `switch_controller`, which starts and stops controllers. The service's callback is the function that does this work. When another node calls the service, the callback runs, and the caller gets its answer when the callback returns.

**The callback queue.** ROS doesn't run a callback at the moment its request arrives. ROS puts the call into the **callback queue**, a waiting list. The queue itself runs nothing, and it isn't a thread. A thread has to take each call out of the queue and run it. ROS calls this **spinning**. The thread that spins is the thread that runs the callback.

*Example:* the whole Gazebo server is one ROS node, `/gazebo`, and its plugins share the node's one queue, the global callback queue. Two things spin this queue in the Gazebo server:

- **The ROS spinner threads** do nothing else. Each takes a waiting call out of the queue and runs it. Gazebo's ROS interface, the plugin `gazebo_ros_api_plugin`, starts one such thread per CPU core with a `ros::AsyncSpinner`, in [line 148 of its source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros/src/gazebo_ros_api_plugin.cpp#L148). The `gzserver` script of gazebo_ros loads this plugin into the server, in [its lines 21–24](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros/scripts/gzserver#L21-L24).
- **`ros::spinOnce()`** spins once, in the thread that calls it. It runs the callbacks that wait in the queue at that moment, and returns when they have returned. The ROS wiki describes `ros::spinOnce()` under [single-threaded spinning](http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning#Single-threaded_Spinning), and [lines 585–588 of roscpp's `init.cpp`](https://github.com/ros/ros_comm/blob/1.17.4/clients/roscpp/src/libros/init.cpp#L585-L588) are its code.

The controller manager's callbacks wait in this global queue too. `gazebo_ros_control` creates the controller manager with an ordinary node handle of the `/gazebo` node, in [lines 150 and 193–194 of its source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros_control/src/gazebo_ros_control_plugin.cpp#L150-L194). A node handle is the object through which a ROS program offers its topics and services. The controller manager offers its services with this node handle, in [lines 45–61 of `controller_manager.cpp`](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L45-L61).

**One fact: the controller manager's services wait for `update()`.** The controller spawner is a separate ROS node, which [`diffbot_control.launch`]({{ imu_sim_src_url }}/diffbot_control/launch/diffbot_control.launch#L23-L25) starts. The spawner loads and starts the controllers with two of the controller manager's services, `load_controller` and `switch_controller`. Their callbacks don't change the running controllers themselves, because `update()` may be using the controllers at that moment. The callbacks note the change and wait until `update()` has taken it, in [lines 352–359](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L352-L359) and [lines 612–630 of `controller_manager.cpp`](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L612-L630). `switch_controller` can give up after a time limit. The spawner asks for none: it passes 0, in [line 212 of its source](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/scripts/spawner#L212).

### What goes wrong with the original plugin

The original plugin's `UpdateChild()` calls `ros::spinOnce()` after it publishes, in [line 150 of its source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L150). So the physics thread spins the global queue in every call that publishes. The figure follows one `switch_controller` request of the spawner. Its numbers are the steps below:

--8<-- "includes/imu-sim-threads-original.html"

1. The controller spawner calls `switch_controller`. The request arrives in the Gazebo server and waits in the callback queue.
2. Before a spinner thread takes the request, the physics thread starts a step and calls `UpdateChild()`. The plugin publishes its message and calls `ros::spinOnce()`.
3. `ros::spinOnce()` finds the request in the queue and runs the service's callback, in the physics thread.
4. The callback notes which controllers to start, and waits until `update()` has taken the change.
5. `update()` runs only in the physics thread. But the physics thread is still inside `UpdateChild()`, waiting in the callback. The physics thread waits for itself. This is a **deadlock**: `update()` never runs, the callback never returns, and the simulation step never ends. The simulation's clock stops for good, and the spawner never gets its answer.

The stall depends on timing. It needs a request of the spawner in the queue at the moment that the plugin spins, before a spinner thread has taken the request. So the stall hits only some starts of the simulation. [diffbot#185](https://github.com/ros-mobile-robots/diffbot/issues/185) counts how many starts stalled with the original and with the copy.

### What the copy does instead

The copy's `UpdateChild()` ends after it publishes, as [the header and the stamp](measurement-to-message.md#the-header-and-the-stamp) shows. So the physics thread never runs a callback. The same request, with the copy:

--8<-- "includes/imu-sim-threads-copy.html"

1. The request waits in the callback queue, as before.
2. The physics thread calls `UpdateChild()`. The copy publishes its message and returns.
3. A ROS spinner thread takes the request and runs the callback. The callback notes the change and waits, in the spinner thread.
4. The physics thread goes on. It calls `update()`, in this step or the next, and `update()` takes the change.
5. The callback returns, and the spawner gets its answer. The controllers run.

The copy needs no spinning of its own, because it only publishes: it has no callbacks. The spinner threads run every callback of the Gazebo server, the controller manager's too.

### Why not just wait?

The deadlock comes from the physics thread running a callback that waits for the physics thread. The first three ideas below wait differently and don't remove that dependency. The fourth removes it in another place:

- **A delay, or starting Gazebo paused.** A delay at the start might change which thread takes the spawner's request, but the unsafe path stays. Starting Gazebo paused doesn't help either. The controllers start only when `update()` takes the change, and the physics thread calls `update()` only while the simulation runs. So the request waits for the simulation to run, and then the same path is open. Later requests to the controller manager can take this path too.
- **A condition variable** lets a thread sleep until another thread signals it. The callback already waits. Waiting on a condition variable instead of polling would still block the physics thread, which is the only thread that can run `update()`.
- **A timeout.** `switch_controller` has an escape: with a positive timeout, its callback returns a failure about a second after the timeout, in [lines 612–636 of `controller_manager.cpp`](https://github.com/ros-controls/ros_control/blob/0.20.0/controller_manager/src/controller_manager.cpp#L612-L636). The spawner passes 0, which turns that escape off. `load_controller` has no timeout at all. And a failed switch wouldn't start the controllers anyway.
- **A separate queue for the controller manager**, served by a thread of its own, would also keep the callbacks out of the physics thread. That needs a change in `gazebo_ros_control`, which passes the controller manager an ordinary node handle on the global queue, in [lines 149–150 of its source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros_control/src/gazebo_ros_control_plugin.cpp#L149-L150).

The IMU plugin only publishes, so it has no callbacks of its own to run. Leaving out `ros::spinOnce()` is the smallest change that keeps the physics thread out of the queue. The packaged plugin has no option to turn off this call. The robots load their own library, so the fix doesn't require replacing gazebo_plugins' installed library.

## Publishing again after a reset

A reset of the simulation sets Gazebo's clock back to zero. `/gazebo/reset_simulation` is the ROS service for a reset, one of [Gazebo's simulation control services](https://classic.gazebosim.org/tutorials?tut=ros_comm#Services:Simulationcontrol). For example: `rosservice call /gazebo/reset_simulation`.

Both plugins limit their rate. They compare the sensor's time with `last_time`, the time of their last update:

- **The original** has only this rate limit, in [lines 97–100 of its source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L97-L100). After a reset, the sensor's time starts from zero again, and `last_time` still holds a time from before the reset. The difference of the two is negative, so the rate limit stops every call until the clock has passed its value from before the reset. The longer the simulation ran before the reset, the longer the IMU stays silent.
- **The copy** sets `last_time` to the sensor's time when the sensor's time is the earlier one. The rate limit then counts from the new time, and the first message after a reset follows one period, 20 ms, later. [When the plugin publishes](measurement-to-message.md#when-the-plugin-publishes) shows these lines.

The IMU sample validator and the EKF start over when the clock goes back, as [the IMU sample validator](../robot_localization.md#the-imu-sample-validator) and [the EKF's limits](../robot_localization.md#limits) on the Localization page explain. So both need the IMU's samples right after a reset. [The reset test](tests.md#the-reset-test) checks that the samples come.

gazebo_plugins itself has a plugin that treats a clock that went back as the copy does: `gazebo_ros_p3d`, in [lines 240–244 of its source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_p3d.cpp#L240-L244). `gazebo_ros_p3d` publishes a link's true pose and velocity in the simulation, as an odometry message. This page names `gazebo_ros_p3d` only as an example of the same handling. `gazebo_ros_p3d` can't stand in for the IMU plugin: it reads no IMU sensor, and it publishes no acceleration and no `sensor_msgs/Imu` message.

## The plugin's library in Remo's model

DiffBot's macro names the plugin's library itself, in [line 246 of `common_sensors.xacro`]({{ imu_sim_src_url }}/diffbot_description/urdf/include/common_sensors.xacro#L246). Remo's model takes the library's name as the xacro argument `imu_plugin` instead. The argument's default is gazebo_plugins' own library, in [lines 29–36 of `remo.urdf.xacro`](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/remo.urdf.xacro#L29-L36), and the model passes the argument to its macro, in [lines 211–216](https://github.com/ros-mobile-robots/remo_description/blob/2.5.0/urdf/remo.urdf.xacro#L211-L216). The diffbot repository's simulation sets the argument when it expands the model. [`diffbot_view.launch`, lines 6–11]({{ imu_sim_src_url }}/diffbot_gazebo/launch/diffbot_view.launch#L6-L11):

```xml title="diffbot_view.launch: the command that expands the model" linenums="6" hl_lines="6"
--8<-- "{{ imu_sim_src_raw }}/diffbot_gazebo/launch/diffbot_view.launch:6:11"
```

`urdf_file` is the xacro command for the selected robot's model. The command passes `imu_plugin:=libdiffbot_gazebo_ros_imu_sensor.so` to both models. Remo's model uses the value as the plugin's `filename`. DiffBot's model doesn't declare `imu_plugin`, and xacro ignores the argument. So both robots load the same plugin in this simulation, and Remo's model still works on its own, with its default library.

## The copy's other changes

The [change notice]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp#L17-L42) at the top of the file lists every change. Besides the two differences above, the copy has these changes, which don't change what the plugin publishes:

- The destructor uses the plugin's node handle only when `Load()` has created it. `Load()` returns before that when the sensor isn't an IMU sensor, when `frameName` is missing or when ROS isn't initialized. The original then calls the node handle's `shutdown()` through a pointer that was never set.
- The class is in the namespace `diffbot_gazebo_plugins`. So a Gazebo process that loads both libraries can't mix up this class with gazebo_plugins' class of the same name, as [the header's notice]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/include/diffbot_gazebo_plugins/gazebo_ros_imu_sensor.h#L17-L25) says.
- The optional profiler calls are left out. This package doesn't enable them.
- The lines are at most 99 characters long, and the file includes the header of every name it uses.

The source is under the Apache License 2.0, as the original is. The copy keeps the original's notices, and the package installs the license's text, as its [`package.xml`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/package.xml) and [`CMakeLists.txt`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/CMakeLists.txt) show. Gazebo looks for the library's file name on `GAZEBO_PLUGIN_PATH` and then on `LD_LIBRARY_PATH`. The workspace's `setup.bash` puts the workspace's library folder on `LD_LIBRARY_PATH`, as [the comment in `CMakeLists.txt`]({{ imu_sim_src_url }}/diffbot_gazebo_plugins/CMakeLists.txt#L23-L25) says.
