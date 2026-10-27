---
title:Autonomous Differential Drive Mobile Robot - Gazebo Package
excerpt: "ROS Gazebo Package for ROS Melodic running on a Raspberry Pi 4 for an autonomous 2WD Robot to act in an environment according to sensor information."
categories: [robotics]
tags: [2wd, differential drive, robot, ros, noetic, raspberry, pi, autonomous, ubuntu, focal, package, gazebo, simulation]
---

!!! tip "Simulation test"
    CI drives DiffBot and Remo in this simulation on every pull request, without a display. [Simulation Test](development/simulation-test.md) shows how Gazebo, `gazebo_ros_control` and the controllers work together.

!!! info "Simulators"
    DiffBot and Remo use Gazebo Classic 11 with ROS Noetic. [Simulators](simulators.md) explains its end of life, the new Gazebo for ROS 2, and other simulators and when to use them.

## Tuning the simulated robot

How a simulated robot drives depends on more than its kinematics: on its collision shapes, masses, contact settings and the gains of its simulated motors. [Wheel Dynamics in Simulation](theory/modeling-control/wheel-dynamics-simulation.md) explains what each of them does. This section lists DiffBot's and Remo's settings, why they have these values, and how to tune them.

### The current settings

The tables show the settings of the [diffbot repository](https://github.com/ros-mobile-robots/diffbot/tree/5631e7b5957b4d6a34a090e1d7032cca81c9e34f) at `5631e7b` and of [remo_description 2.0.0](https://github.com/ros-mobile-robots/remo_description/tree/2.0.0); the source links below are pinned to these revisions. The experiments on the theory page used earlier revisions, which its box [About the experiments](theory/modeling-control/wheel-dynamics-simulation.md) names. The **Basis** column says where a value comes from: *measured* on the robot, *derived* from other values, Gazebo's *default*, *selected* and tested in the simulation, a *placeholder*, or *not modeled*. The **Reason / limitation** column says what the value is for and what it doesn't cover.

#### Geometry and mass

| Setting | DiffBot | Remo | Basis | Reason / limitation |
|---|---|---|---|---|
| Wheel radius | 0.0333 m | 0.0330 m | measured | Half the mean of the two wheels' diameters. The loaded rolling radius isn't measured. |
| Wheel separation | 0.138 m | 0.144 m | measured | Center to center, as the kinematic model and odometry use it. |
| Wheel width | 0.025 m | 0.025 m | measured | Used for the inertia; the collision is a sphere. |
| Wheel collision | sphere with the wheel radius | sphere with the wheel radius | selected, tested on flat ground | Puts an upright wheel's ideal contact in its center plane, where the kinematic model assumes it. It doesn't reproduce a tire's footprint, or contacts on slopes and edges. |
| Wheel mass | 0.032 kg | 0.032 kg | measured | The same wheels on both robots. |
| Wheel inertia about the axle | $1.77 \cdot 10^{-5}$ kg m² | $1.74 \cdot 10^{-5}$ kg m² | derived: $\frac{1}{2} m r^2$ of a solid cylinder | Follows the mass and radius, independent of the collision shape. A real wheel with a heavy tire has somewhat more. |
| Caster mass | 38 g | 22 g | measured; DiffBot's split between its parts estimated | DiffBot's caster was weighed as one part (38 g); the model gives its base, hub and wheel 10, 10 and 18 g. Remo's parts were weighed one by one; its whole caster weighs 23 g. |
| Robot mass, model / weighed | 1145 g / about 1145 g, estimated | 1154 g / 1303 g | model: sum of the links; weighed | DiffBot: 705 g weighed with its AA battery pack, plus its lidar (190 g, RPLidar A2M8 datasheet) and its power bank (250 g, the value of Remo's model for the same power bank). Its base, a thin box, carries everything not modeled separately, so its center of mass and inertia are approximations. Remo's model lacks about 149 g, for example cables and screws. |

Sources: DiffBot's [`config/diffbot`](https://github.com/ros-mobile-robots/diffbot/tree/5631e7b5957b4d6a34a090e1d7032cca81c9e34f/diffbot_description/config/diffbot) (`front_wheel.yaml`, `caster_wheel.yaml`, `base.yaml`, `motor.yaml`, `sensors.yaml`), Remo's [`config/remo`](https://github.com/ros-mobile-robots/remo_description/tree/2.0.0/config/remo), [`geometry.yaml`](https://github.com/ros-mobile-robots/diffbot/tree/5631e7b5957b4d6a34a090e1d7032cca81c9e34f/diffbot_control/config), the wheel collision and `cylinder_inertia` in [DiffBot's](https://github.com/ros-mobile-robots/diffbot/blob/5631e7b5957b4d6a34a090e1d7032cca81c9e34f/diffbot_description/urdf/include/common_macros.urdf.xacro#L55-L70) and [Remo's](https://github.com/ros-mobile-robots/remo_description/blob/d966e189eab1c6e9b8844d9ddacff874755cea10/urdf/include/common_macros.urdf.xacro#L55-L70) `common_macros.urdf.xacro`.

#### Contacts

| Setting | DiffBot | Remo | Basis | Reason / limitation |
|---|---|---|---|---|
| Friction `mu1`, `mu2` | 1, 1 | 1, 1 | selected | A usual order of magnitude for rubber on a hard floor, not measured for these tires and floors. Gazebo uses the smaller coefficient of the wheel and the ground. Friction 200 changed nothing measurable in Remo's wheel tracking ([experiment E3](theory/modeling-control/wheel-dynamics-simulation.md#which-contact-setting-makes-the-wheels-miss-their-commands)). |
| `fdir1` | not set | not set | selected | Left unset to avoid a direction that turns with the sphere's frame and leaves the contact plane, as `0 0 1` would; a direction along the axle would stay in the contact plane, but Gazebo Classic uses only one of the two surfaces' `fdir1` in each contact. |
| Contact `kp`, `kd` | $10^{7}$ N/m, 1 N s/m | $10^{7}$ N/m, 1 N s/m | selected | With Gazebo's stiff default ($10^{12}$ N/m), both robots' light wheels missed their speed commands; DiffBot then turned only 0.33 to 0.87 of what its odometry said ([diffbot#137](https://github.com/ros-mobile-robots/diffbot/pull/137)). Damping 0 alone made no difference ([experiment E3](theory/modeling-control/wheel-dynamics-simulation.md#which-contact-setting-makes-the-wheels-miss-their-commands)). |
| `minDepth` | 0 | 0 | selected | With Remo's compliant contacts, a positive value let the wheels creep into the ground. Zero is the tested choice for these wheels, not a universal rule. |
| `maxVel` | 0.01 m/s | 0.01 m/s | default | Limits how fast a penetration is corrected. |
| Caster wheel contact | friction 1, $10^{7}$ N/m, 1 N s/m | friction 0.01, $10^{12}$ N/m, 0 N s/m, `minDepth` 0.01 | DiffBot: selected; Remo: set, not tested | DiffBot: with the stiff default, its 18 g caster wheel made the drive wheels' speeds jitter in right turns. Remo's low friction lets its caster slide; it wasn't varied in these experiments. |

Sources: DiffBot's drive-wheel contacts in [`common_macros.urdf.xacro`](https://github.com/ros-mobile-robots/diffbot/blob/5631e7b5957b4d6a34a090e1d7032cca81c9e34f/diffbot_description/urdf/include/common_macros.urdf.xacro#L72-L86) and its caster's in [`diffbot_caster_macro.urdf.xacro`](https://github.com/ros-mobile-robots/diffbot/blob/5631e7b5957b4d6a34a090e1d7032cca81c9e34f/diffbot_description/urdf/include/diffbot_caster_macro.urdf.xacro#L172-L180); Remo's [drive-wheel](https://github.com/ros-mobile-robots/remo_description/blob/d966e189eab1c6e9b8844d9ddacff874755cea10/urdf/include/robot.gazebo.xacro#L39-L66) and [caster](https://github.com/ros-mobile-robots/remo_description/blob/d966e189eab1c6e9b8844d9ddacff874755cea10/urdf/include/robot.gazebo.xacro#L68-L87) contacts, with the `friction` argument of [`remo.urdf.xacro`](https://github.com/ros-mobile-robots/remo_description/blob/d966e189eab1c6e9b8844d9ddacff874755cea10/urdf/remo.urdf.xacro#L192-L194).

#### Control and world

| Setting | DiffBot | Remo | Basis | Reason / limitation |
|---|---|---|---|---|
| Wheel velocity PID `p`, `i`, `d` | 0.02, 0.6, 0 | 0.03, 0.6, 0 | selected, tested at a 1 ms step | Both P are below the free-wheel limit of about 0.035 for their 32 g wheels. For DiffBot, P 0.01, 0.02 and 0.03 tracked equally well; 0.02 keeps more distance to the limit. Neither is shown to be optimal. |
| `i_clamp`, `antiwindup` | 0.05 N m, true | 0.05 N m, true | selected | The I term can supply up to 0.05 N m; without `i_clamp`, an I gain does nothing. |
| Wheel torque limit | none | none | not modeled | A real motor has a stall torque. |
| Physics | ODE, quick solver with 150 iterations, 1 ms step | the same | inherited from the world | The simulation test's `diffbot_empty.world`. Other worlds differ, for example `playground.world` with a 10 ms step, and the gains depend on the step. |

Sources: `pid.yaml` of [DiffBot](https://github.com/ros-mobile-robots/diffbot/blob/5631e7b5957b4d6a34a090e1d7032cca81c9e34f/diffbot_control/config/diffbot/pid.yaml) and [Remo](https://github.com/ros-mobile-robots/diffbot/blob/5631e7b5957b4d6a34a090e1d7032cca81c9e34f/diffbot_control/config/remo/pid.yaml), [the test world](https://github.com/ros-mobile-robots/diffbot/blob/5631e7b5957b4d6a34a090e1d7032cca81c9e34f/diffbot_gazebo/worlds/diffbot_empty.world#L13-L32).

How these values were found: [diffbot#119](https://github.com/ros-mobile-robots/diffbot/pull/119) (geometry), [diffbot#120](https://github.com/ros-mobile-robots/diffbot/pull/120) and [remo_description#2](https://github.com/ros-mobile-robots/remo_description/pull/2) (collisions and contacts), [diffbot#121](https://github.com/ros-mobile-robots/diffbot/pull/121) (gains), [diffbot#137](https://github.com/ros-mobile-robots/diffbot/pull/137) (DiffBot's masses, contacts and gain).

### How to tune

Tune the model in this order, so that a later step doesn't hide an error of an earlier one. One error can hide another: for example, wheels that sink into the ground can change where they touch it. So after each fix, go through the later steps again.

1. **Geometry and inertia first.** Measure the wheel diameters and their separation, center to center, and enter them in the URDF's `front_wheel.yaml` and the controller's `geometry.yaml`. Check the expanded URDF with `xacro`: the wheel joints' distance, the collision radius, and an inertia whose $\frac{1}{2} m r^2$ is about the joint axis.
2. **Record a baseline.** Note the world, its time step and solver (the gains depend on the time step), and run the [simulation test](development/simulation-test.md): the odometry and the true pose, driving straight and turning on the spot.
3. **Check the spawn and the settling.** The robot has to stay at its spawn pose without NaN velocities, and its wheels have to stay on the ground: a wheel link's height above the ground should stay at its radius over a minute (`/gazebo/link_states`). *Example:* with compliant contacts and a positive `minDepth`, Remo's wheels crept about 8 mm into the ground within a minute, and the turn test still passed. Only the height showed it.
4. **Wheel speed tracking.** Turn on the spot in both directions and drive straight, and compare the wheel joints' speeds in `/diffbot/joint_states` with the commanded ones: their mean and their standard deviation. A lasting error can come from the controller, a torque limit or the contact load; jitter can come from contact solving or an unstable loop. Keep P below the free-wheel limit $2J/h$. *Example:* with a P gain alone, Remo's wheels reached only about 80 % of the command when turning on the spot, because turning needs torque; an I term with `i_clamp` removed the error.
5. **Agreement with the true motion.** Compare the odometry with the true pose. *Example:* wheel speeds within 1 % of the command, but a true turn of only about 0.85 of the odometry's. Then test the effective contact separation, for example with a different collision shape, and keep the controller as it is.
6. **One change at a time.** After each change, repeat straight driving and turns in both directions, for both robots, and run each case more than once to see how much the results vary.

Don't compensate one error with another setting: a wrong geometry with gains, or an unstable velocity loop with heavier wheels than the real ones. The simulation test catches regressions, but passing it doesn't make the simulation accurate at other speeds, on other floors or slopes.

## Simulate DiffBot in Gazebo

As described in the [Creating your own Gazebo ROS Package](http://gazebosim.org/tutorials?tut=ros_roslaunch#CreatingyourownGazeboROSPackage), it is common in ROS to create a package that contains all the world files and launch files used with Gazebo. These files are located in a ROS package named `/MYROBOT_gazebo`. For DiffBot the package is named `diffbot_gazebo`. Another example that follows best pratices is `rrbot` which can be found in the [gazebo_ros_demos](https://github.com/ros-simulation/gazebo_ros_demos) repository.

```console
fjp@ubuntu:~/git/diffbot/ros/src$ catkin create pkg diffbot_gazebo
Creating package "diffbot_gazebo" in "/home/fjp/git/diffbot/ros/src"...
Created file diffbot_gazebo/package.xml
Created file diffbot_gazebo/CMakeLists.txt
Successfully created package files in /home/fjp/git/diffbot/ros/src/diffbot_gazebo.
```
 
The `diffbot_gazebo` package contains a launch file to launch a world in Gazebo and spawn the robot model, 
which is defined in the previously created `diffbot_description` package. 
For the launch files the convention is to have a folder named `launch` and for Gazebo world files a folder named `world` inside a package.
 
 ```console
 fjp@ubuntu:~/git/diffbot/ros/src/diffbot_gazebo$ mkdir launch world
 ```
 
 Inside the `launch` folder is the `diffbot.launch`.
 
```xml
<launch>
    <!-- these are the arguments you can pass this launch file, for example paused:=true -->
    <arg name="paused" default="false"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>

    <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find diffbot_gazebo)/worlds/diffbot.world"/>
        <arg name="debug" value="$(arg debug)" />
        <arg name="gui" value="$(arg gui)" />
        <arg name="paused" value="$(arg paused)"/>
        <arg name="use_sim_time" value="$(arg use_sim_time)"/>
        <arg name="headless" value="$(arg headless)"/>
    </include>
</launch>
```

In the `world` folder of the `diffbot_gazebo` package is the `diffbot.world` file:

```xml
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://gas_station</uri>
      <name>gas_station</name>
      <pose>-2.0 7.0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
```

With these files build the catkin workspace and source it to make the new `diffbot_gazebo` package visible to `roslaunch`:

```console
catkin build
source devel/setup.zsh
```

Then its possible to launch the `diffbot.launch` with:

```console
roslaunch diffbot_gazebo diffbot.launch
```

This will lead to the following output:

```console
roslaunch diffbot_gazebo diffbot.launch 
... logging to /home/fjp/.ros/log/6be90ef2-fdd8-11ea-9cb3-317fd602d5f2/roslaunch-tensorbook-393333.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://tensorbook:32837/

SUMMARY
========

PARAMETERS
 * /gazebo/enable_ros_network: True
 * /rosdistro: noetic
 * /rosversion: 1.15.8
 * /use_sim_time: True

NODES
  /
    gazebo (gazebo_ros/gzserver)
    gazebo_gui (gazebo_ros/gzclient)

ROS_MASTER_URI=http://localhost:11311

process[gazebo-1]: started with pid [393352]
process[gazebo_gui-2]: started with pid [393357]
[ INFO] [1600950165.494721382]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1600950165.495515766]: waitForService: Service [/gazebo/set_physics_properties] has not been advertised, waiting...
[ INFO] [1600950165.649461740]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1600950165.650277038]: waitForService: Service [/gazebo_gui/set_physics_properties] has not been advertised, waiting...
[ INFO] [1600950166.640929113]: waitForService: Service [/gazebo/set_physics_properties] is now available.
[ INFO] [1600950166.659917502, 0.007000000]: Physics dynamic reconfigure ready.
```

Also, the Gazebo simulator will open a new window with the objects defined in `diffbot.world` except for the Gas station because it is a model 
that has to be downloaded first, which is happening in the background. TODO: gas station is not showing this way.


<figure>
    <a href="{{ asset_dir }}/gazebo/empty-world.png"><img src="{{ asset_dir }}/gazebo/empty-world.png"></a>
    <figcaption>Empty world of DiffBot.</figcaption>
</figure>


To get the Gas station or other available models it is possible to clone the [gazebo models repository](https://github.com/osrf/gazebo_models) into your `/home/your_username/.gazebo` folder, e.g.:

```console
/home/fjp/.gazeb$ git clone osrf/gazebo_models
```
Then add this path inside Gazebo to insert these models into your world file.


### Using ROS launch to Spawn URDF Robots
 
 
 According to the [Gazebo roslaunch tutorial](http://gazebosim.org/tutorials?tut=ros_roslaunch#UsingroslaunchtoSpawnURDFRobots) the recommended way
 to spawn a robot into Gazebo is to use a launch file. Therefore, edit the `diffbot.launch` inside the `diffbot_gazebo` package by adding the following inside the `<launch> </launch` tag:
 
 ```console
    <!-- Load the URDF into the ROS Parameter Server -->
    <param name="robot_description"
        command="$(find xacro)/xacro --inorder '$(find diffbot_description)/urdf/diffbot.xacro'" />

    <!-- Run a python script to the send a service call to gazebo_ros to spawn a URDF robot -->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
        args="-urdf -model diffbot -param robot_description"/>
 ```

See also the complete [`diffbot.launch`](https://github.com/fjp/diffbot/blob/master/ros/src/diffbot_gazebo/launch/diffbot.launch) file.


This will open Gazebo simulator and show the DiffBot model:

<figure>
    <a href="{{ asset_dir }}/gazebo/diffbot.png"><img src="{{ asset_dir }}/gazebo/diffbot.png"></a>
    <figcaption>Empty world including DiffBot.</figcaption>
</figure>

### Moving the Robot

Note that the robot cannot be moved without having either a Gazebo plugin loaded or making use of [ROS Control](http://wiki.ros.org/ros_control)
and its Gazebo plugin [`gazebo_ros_control`](http://wiki.ros.org/gazebo_ros_control), see also the [Gazebo ROS Control Tutorial](http://gazebosim.org/tutorials/?tut=ros_control#Addthegazebo_ros_controlplugin). Using the ROS Control and its Gazebo plugin is done in case of DiffBot. 
An alternative would be to use the existing [`differential_drive_controller`](http://gazebosim.org/tutorials?tut=ros_gzplugins#DifferentialDrive) Gazebo plugin without having to rely on ROS Control. 
The next section explains the `diffbot_control` package in more detail and how to setup the 
[`diff_drive_controller`](http://wiki.ros.org/diff_drive_controller?distro=noetic)
from the [`ros_controllers`](http://wiki.ros.org/ros_controllers) package.


### Adding Sensors

To add sensors to a robot model make use of link and joint tags to define the desired location and shape, possibly using meshes.
For the simulation of these sensor there exist common Gazebo plugins that can be used. See, the [Tutorial: Using Gazebo plugins with ROS]
for existing plugins and more details how to use them. For a full list of plugins see also [`gazebo_ros_pkgs`](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/kinetic-devel/gazebo_plugins/src) which is a package or interface
for using ROS with the Gazebo simulator.


#### Camera

This section follows Gazebo tutorial [Adding a Camera](http://gazebosim.org/tutorials?tut=ros_gzplugins#Camera).

#### Laser (Lidar)

This section follows Gazebo tutorial [Adding a Laser GPU](http://gazebosim.org/tutorials?tut=ros_gzplugins#GPULaser).


#### Ultrasonic Ranger


See the source of the [`gazebo_ros_range`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_range.cpp) plugin.


#### Inertial Measurement Unit (IMU)

This section follows Gazebo tutorial [Adding an IMU](http://gazebosim.org/tutorials?tut=ros_gzplugins#IMU(GazeboRosImu). Note use GazeboRosImuSensor?


## Troubleshooting

A quick way to verify if the conversion from [xacro](http://wiki.ros.org/xacro) to [urdf](http://wiki.ros.org/urdf) to [sdf](http://sdformat.org/) is working is the following ([source: Tutorial URDF in Gazebo](http://gazebosim.org/tutorials/?tut=ros_urdf#VerifyingtheGazeboModelWorks)):
First convert the xacro model to a urdf model with the `xacro` command:

```console
xacro $(rospack find diffbot_description)/urdf/diffbot.urdf.xacro -o diffbot.urdf
```

This will output the urdf into a file named `diffbot.urdf` in the current working directory. For Remo, use `$(rospack find remo_description)/urdf/remo.urdf.xacro` and `remo.urdf`.

Then use the `gz` command to create a sdf:

```
# gazebo3 and above
gz sdf -p diffbot.urdf
```

Each link's `collision/surface` element in the output holds its contact settings. [From URDF to SDF](theory/modeling-control/wheel-dynamics-simulation.md#from-urdf-to-sdf) explains them, with Remo's wheel as an example.

In case the output looks like the following, there are most certainly missing [`<inertial>`](http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model#Inertia) tags in the `<link>` tag.
For the Gazebo simulator the `<inertial>` must be present, in order to simulate the dynamics of the robot. 
See also http://wiki.ros.org/urdf/XML/link and the [Gazebo tutorials on URDF](http://gazebosim.org/tutorials/?tut=ros_urdf).

```xml
<sdf version='1.7'>
  <model name='diffbot'/>
</sdf>
```
