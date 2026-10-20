---
title:Autonomous Differential Drive Mobile Robot - Gazebo Package
excerpt: "ROS Gazebo Package for ROS Melodic running on a Raspberry Pi 4 for an autonomous 2WD Robot to act in an environment according to sensor information."
categories: [robotics]
tags: [2wd, differential drive, robot, ros, noetic, raspberry, pi, autonomous, ubuntu, focal, package, gazebo, simulation]
---

!!! tip "Simulation test"
    CI drives DiffBot and Remo in this simulation on every pull request, without a display. [Simulation Test](development/simulation-test.md) shows how Gazebo, `gazebo_ros_control` and the controllers work together.

## Tuning the simulated robot

How a simulated robot drives depends on more than its kinematics: on its collision shapes, masses, contact settings and the gains of its simulated motors. [Wheel Dynamics in Simulation](theory/modeling-control/wheel-dynamics-simulation.md) explains what each of them does. This section lists DiffBot's and Remo's values and how to tune them.

### The current settings

| Setting | DiffBot | Remo | Where | Status |
|---|---|---|---|---|
| Wheel radius | 0.0333 m | 0.0330 m | `front_wheel.yaml` (URDF) and `geometry.yaml` (controller) | measured: half the mean of the two wheels' diameters |
| Wheel separation | 0.138 m | 0.144 m | `front_wheel.yaml` (`y_offset`) and `geometry.yaml` | measured, center to center |
| Wheel width | 0.025 m | 0.025 m | `front_wheel.yaml` (`length`) | measured |
| Wheel collision | sphere with the wheel radius | sphere with the wheel radius | `common_macros.urdf.xacro` | chosen in [diffbot#120](https://github.com/ros-mobile-robots/diffbot/pull/120) and [remo_description#2](https://github.com/ros-mobile-robots/remo_description/pull/2): cylinders touched the ground at their outer edges |
| Wheel mass | 0.5 kg | 0.032 kg | `front_wheel.yaml` | Remo: weighed; DiffBot: a placeholder, its wheels weigh 32 g too ([diffbot#118](https://github.com/ros-mobile-robots/diffbot/issues/118)) |
| Wheel inertia about the axle, $\frac{1}{2} m r^2$ | $2.8 \cdot 10^{-4}$ kg m² | $1.74 \cdot 10^{-5}$ kg m² | computed by `cylinder_inertia` in `common_macros.urdf.xacro` | follows the mass and radius |
| Friction `mu1`, `mu2` | 1, 1 | 1, 1 | Remo: `robot.gazebo.xacro`, with the `friction` argument of `remo.urdf.xacro` | DiffBot: Gazebo's defaults; Remo: chosen in remo_description#2, rubber on a floor |
| `fdir1` | none | none | | a sphere's `fdir1` would turn with the wheel |
| Contact `kp`, `kd` | $10^{12}$ N/m, 1 N s/m | $10^{7}$ N/m, 1 N s/m | Remo: `robot.gazebo.xacro` | DiffBot: defaults; Remo: chosen in remo_description#2, so its wheels follow their commands on the sphere |
| `minDepth`, `maxVel` | 0 m, 0.01 m/s | 0 m, 0.01 m/s | Remo: `robot.gazebo.xacro` | DiffBot: defaults; Remo: `minDepth` 0 in remo_description#2, so its wheels don't sink |
| Wheel velocity PID `p`, `i`, `d` | 0.06, 0.6, 0 | 0.03, 0.6, 0 | `diffbot_control/config/<model>/pid.yaml` | chosen in [diffbot#121](https://github.com/ros-mobile-robots/diffbot/pull/121); Remo's P below the free-wheel limit of about 0.035 |
| `i_clamp`, `antiwindup` | 0.05 N m, true | 0.05 N m, true | `pid.yaml` | chosen in diffbot#121 |
| Wheel torque limit | none | none | a `<limit effort="...">` on the wheel joint | not set |
| Physics | ODE, quick solver with 150 iterations, 1 ms step | the same | `diffbot_gazebo/worlds/diffbot_empty.world`, the simulation test's world; other worlds differ, for example `playground.world` with a 10 ms step | from the original worlds |

The files: [`front_wheel.yaml` (DiffBot)](https://github.com/ros-mobile-robots/diffbot/blob/493843b1ea10d51b11aad732f023014eae4d1249/diffbot_description/config/diffbot/front_wheel.yaml), [`front_wheel.yaml` (Remo)](https://github.com/ros-mobile-robots/remo_description/blob/ba0b2c567e41adcd4ef0e5f652e6d49c919c9462/config/remo/front_wheel.yaml), [`geometry.yaml`](https://github.com/ros-mobile-robots/diffbot/tree/493843b1ea10d51b11aad732f023014eae4d1249/diffbot_control/config), [DiffBot's wheel collision](https://github.com/ros-mobile-robots/diffbot/blob/493843b1ea10d51b11aad732f023014eae4d1249/diffbot_description/urdf/include/common_macros.urdf.xacro#L55-L70), [Remo's wheel collision](https://github.com/ros-mobile-robots/remo_description/blob/ba0b2c567e41adcd4ef0e5f652e6d49c919c9462/urdf/include/common_macros.urdf.xacro#L55-L70), [Remo's contacts](https://github.com/ros-mobile-robots/remo_description/blob/ba0b2c567e41adcd4ef0e5f652e6d49c919c9462/urdf/include/robot.gazebo.xacro#L39-L66), [`pid.yaml` (DiffBot)](https://github.com/ros-mobile-robots/diffbot/blob/493843b1ea10d51b11aad732f023014eae4d1249/diffbot_control/config/diffbot/pid.yaml), [`pid.yaml` (Remo)](https://github.com/ros-mobile-robots/diffbot/blob/493843b1ea10d51b11aad732f023014eae4d1249/diffbot_control/config/remo/pid.yaml) and [the test world](https://github.com/ros-mobile-robots/diffbot/blob/493843b1ea10d51b11aad732f023014eae4d1249/diffbot_gazebo/worlds/diffbot_empty.world#L13-L32).

Both robots' wheels weigh 32 g, and the robots ready to drive 705 g (DiffBot) and 1303 g (Remo). Remo's model has its weighed parts, 1154 g in all. DiffBot's model still has placeholders, 2640 g in all, with 0.5 kg per wheel. With its real 32 g wheels, DiffBot's wheel inertia drops to about Remo's, and its P gain of 0.06 would be above the free-wheel limit: the masses and the gains have to change together, in a follow-up of diffbot#118.

### How to tune

Tune the model in this order, so that a later step doesn't hide an error of an earlier one:

1. **Geometry and inertia first.** Measure the wheel diameters and their separation, center to center, and enter them in the URDF's `front_wheel.yaml` and the controller's `geometry.yaml`. Check the expanded URDF with `xacro`: the wheel joints' distance, the collision radius, and an inertia whose $\frac{1}{2} m r^2$ is about the joint axis.
2. **Record a baseline.** Note the world, its time step and solver (the gains depend on the time step), and run the [simulation test](development/simulation-test.md): the odometry and the true pose, driving straight and turning on the spot.
3. **Check the spawn.** The robot has to stay at its spawn pose without NaN velocities, and its wheels have to stay on the ground: a wheel link's height above the ground should stay at its radius over a minute (`/gazebo/link_states`).
4. **Wheel speed tracking.** Turn on the spot in both directions and drive straight, and compare the wheel joints' speeds in `/diffbot/joint_states` with the commanded ones: their mean and their standard deviation. A speed error that stays is a job for the I term; jitter points to the contacts. Keep P below the free-wheel limit $2J/h$.
5. **Agreement with the true motion.** Compare the odometry with the true pose. If the wheels follow their commands and the robot still turns differently, look at where and how the wheels touch the ground: the collision shape, friction and contact settings, not the gains.
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
xacro src/diffbot_description/urdf/diffbot.xacro -o diffbot.urdf
```

This will output the urdf into a file named `diffbot.urdf` in the current working directory.

Then use the `gz` command to create a sdf:

```
# gazebo3 and above
gz sdf -p MODEL.urdf
```

The output below is historical: it comes from DiffBot's model of 2020, with 2.5 kg wheels and cylinder collisions with a 0.04 m radius. It only shows what `gz sdf` prints. The current values are in [Tuning the simulated robot](#tuning-the-simulated-robot).

<details markdown="1"><summary>DiffBot's SDF in 2020.</summary>
```xml
<sdf version='1.7'>
  <model name='diffbot'>
    <link name='base_footprint'>
      <inertial>
        <pose>-0.012273 0 0.040818 0 -0 0</pose>
        <mass>5.5</mass>
        <inertia>
          <ixx>0.0387035</ixx>
          <ixy>0</ixy>
          <ixz>0.000552273</ixz>
          <iyy>0.0188626</iyy>
          <iyz>0</iyz>
          <izz>0.0561591</izz>
        </inertia>
      </inertial>
      <collision name='base_footprint_collision'>
        <pose>0 0 0 0 -0 0</pose>
        <geometry>
          <box>
            <size>0.001 0.001 0.001</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode/>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
      <collision name='base_footprint_fixed_joint_lump__base_link_collision_1'>
        <pose>0 0 0.04 0 -0 0</pose>
        <geometry>
          <box>
            <size>0.3 0.15 0.02</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode/>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
      <collision name='base_footprint_fixed_joint_lump__caster_link_collision_2'>
        <pose>-0.135 0 0.029 0 -0 0</pose>
        <geometry>
          <sphere>
            <radius>0.025</radius>
          </sphere>
        </geometry>
        <surface>
          <contact>
            <ode/>
          </contact>
          <friction>
            <ode/>
          </friction>
        </surface>
      </collision>
      <visual name='base_footprint_fixed_joint_lump__base_link_visual'>
        <pose>0 0 0.04 0 -0 0</pose>
        <geometry>
          <box>
            <size>0.3 0.15 0.02</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/White</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <visual name='base_footprint_fixed_joint_lump__caster_link_visual_1'>
        <pose>-0.115 0 0.029 0 -0 0</pose>
        <geometry>
          <sphere>
            <radius>0.025</radius>
          </sphere>
        </geometry>
      </visual>
      <velocity_decay/>
      <velocity_decay/>
      <gravity>1</gravity>
      <velocity_decay/>
    </link>
    <joint name='front_left_wheel_joint' type='revolute'>
      <pose relative_to='base_footprint'>0.105 -0.085 0.04 0 -0 0</pose>
      <parent>base_footprint</parent>
      <child>front_left_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1e+16</lower>
          <upper>1e+16</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
    <link name='front_left_wheel'>
      <pose relative_to='front_left_wheel_joint'>0 0 0 0 -0 0</pose>
      <inertial>
        <pose>0 0 0 0 -0 0</pose>
        <mass>2.5</mass>
        <inertia>
          <ixx>0.00108333</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00108333</iyy>
          <iyz>0</iyz>
          <izz>0.002</izz>
        </inertia>
      </inertial>
      <collision name='front_left_wheel_collision'>
        <pose>0 0 0 1.5708 -0 0</pose>
        <geometry>
          <cylinder>
            <length>0.02</length>
            <radius>0.04</radius>
          </cylinder>
        </geometry>
        <surface>
          <contact>
            <ode>
              <kp>1e+07</kp>
              <kd>1</kd>
            </ode>
          </contact>
          <friction>
            <ode>
              <mu>1</mu>
              <mu2>1</mu2>
              <fdir1>1 0 0</fdir1>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name='front_left_wheel_visual'>
        <pose>0 0 0 1.5708 -0 0</pose>
        <geometry>
          <cylinder>
            <length>0.02</length>
            <radius>0.04</radius>
          </cylinder>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Grey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <gravity>1</gravity>
      <velocity_decay/>
    </link>
    <joint name='front_right_wheel_joint' type='revolute'>
      <pose relative_to='base_footprint'>0.105 0.085 0.04 0 -0 0</pose>
      <parent>base_footprint</parent>
      <child>front_right_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1e+16</lower>
          <upper>1e+16</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
    <link name='front_right_wheel'>
      <pose relative_to='front_right_wheel_joint'>0 0 0 0 -0 0</pose>
      <inertial>
        <pose>0 0 0 0 -0 0</pose>
        <mass>2.5</mass>
        <inertia>
          <ixx>0.00108333</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00108333</iyy>
          <iyz>0</iyz>
          <izz>0.002</izz>
        </inertia>
      </inertial>
      <collision name='front_right_wheel_collision'>
        <pose>0 0 0 1.5708 -0 0</pose>
        <geometry>
          <cylinder>
            <length>0.02</length>
            <radius>0.04</radius>
          </cylinder>
        </geometry>
        <surface>
          <contact>
            <ode>
              <kp>1e+07</kp>
              <kd>1</kd>
            </ode>
          </contact>
          <friction>
            <ode>
              <mu>1</mu>
              <mu2>1</mu2>
              <fdir1>1 0 0</fdir1>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name='front_right_wheel_visual'>
        <pose>0 0 0 1.5708 -0 0</pose>
        <geometry>
          <cylinder>
            <length>0.02</length>
            <radius>0.04</radius>
          </cylinder>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Grey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <gravity>1</gravity>
      <velocity_decay/>
    </link>
    <plugin name='gazebo_ros_control' filename='libgazebo_ros_control.so'>
      <robotNamespace>/diffbot</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
    <static>0</static>
    <plugin name='differential_drive_controller' filename='libgazebo_ros_diff_drive.so'>
      <legacyMode>1</legacyMode>
      <rosDebugLevel>Debug</rosDebugLevel>
      <publishWheelTF>0</publishWheelTF>
      <robotNamespace>/</robotNamespace>
      <publishTf>1</publishTf>
      <publishWheelJointState>0</publishWheelJointState>
      <alwaysOn>1</alwaysOn>
      <updateRate>100.0</updateRate>
      <leftJoint>front_left_wheel_joint</leftJoint>
      <rightJoint>front_right_wheel_joint</rightJoint>
      <wheelSeparation>0.3</wheelSeparation>
      <wheelDiameter>0.08</wheelDiameter>
      <broadcastTF>1</broadcastTF>
      <wheelTorque>30</wheelTorque>
      <wheelAcceleration>1.8</wheelAcceleration>
      <commandTopic>cmd_vel</commandTopic>
      <odometryFrame>odom</odometryFrame>
      <odometryTopic>odom</odometryTopic>
      <robotBaseFrame>base_footprint</robotBaseFrame>
    </plugin>
  </model>
</sdf>
```
</details>



In case the output looks like the following, there are most certainly missing [`<inertial>`](http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model#Inertia) tags in the `<link>` tag.
For the Gazebo simulator the `<inertial>` must be present, in order to simulate the dynamics of the robot. 
See also http://wiki.ros.org/urdf/XML/link and the [Gazebo tutorials on URDF](http://gazebosim.org/tutorials/?tut=ros_urdf).

```xml
<sdf version='1.7'>
  <model name='diffbot'/>
</sdf>
```
