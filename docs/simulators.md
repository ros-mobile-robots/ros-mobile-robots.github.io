# Simulators

A simulator runs a robot model in a virtual world, with physics and simulated sensors. With it you can develop and test without the robot, try things that would be risky on the real one, and test every change automatically in CI, as the [Simulation Test](development/simulation-test.md) does. This page explains what DiffBot and Remo use, what changes with ROS 2, which other simulators exist and when they're the better choice, and where simulation reaches its limits.

## What DiffBot and Remo use: Gazebo Classic 11

On ROS Noetic, DiffBot and Remo run in [Gazebo Classic 11](https://classic.gazebosim.org/), the Gazebo that comes with ROS Noetic's desktop-full installation and with the [dev container](development/dev-container.md). These parts connect the robots' ROS software with Gazebo:

- [`diffbot_gazebo`](diffbot_gazebo.md) has the worlds and the launch files that start Gazebo with a robot.
- [`gazebo_ros`](https://classic.gazebosim.org/tutorials?tut=ros_overview) spawns the robot from its URDF and publishes the true pose of every model.
- [`gazebo_ros_control`](https://classic.gazebosim.org/tutorials?tut=ros_control) loads [ros_control](diffbot_control.md) into Gazebo, so the same controllers and configuration drive the simulated wheels as the real ones.
- The URDF's [`<gazebo>` tags](https://classic.gazebosim.org/tutorials?tut=ros_urdf) add what only the simulation needs, such as friction and contact settings, the lidar and camera plugins, and the ros_control plugin. The lidar is simulated on the CPU, with Gazebo Classic's `ray` sensor.
- The [Simulation Test](development/simulation-test.md) runs only the Gazebo server, without a GUI and without a GPU, on GitHub's standard runners.

!!! warning "Gazebo Classic has reached its end of life"
    [Gazebo 11 is the last major release](https://classic.gazebosim.org/), its support ended on 29 January 2025, and its [repository](https://github.com/gazebosim/gazebo-classic) is archived, with 11.15.1 as the final version. So it gets no more fixes. ROS Noetic, which reached its end of life on 31 May 2025, has no supported way to the new Gazebo either: the ROS 1 bridge `ros_ign` has binary packages only for Gazebo Citadel, which is also past its end of life, and newer releases such as Fortress need it built from source ([ros_gz, noetic branch](https://github.com/gazebosim/ros_gz/tree/noetic)).

So DiffBot and Remo stay with Gazebo Classic as long as they run ROS 1 Noetic. The project's [roadmap](development/index.md#roadmap) keeps the `noetic-devel` branch working, Gazebo Classic included, and moves to the new Gazebo with the ROS 2 port.

## The new Gazebo for ROS 2

The successor of Gazebo Classic is [Gazebo](https://gazebosim.org/), formerly called Ignition (its simulator is `gz-sim`). It's a rewrite, with new names for the tools and plugins. Each release pairs with a ROS 2 distribution ([ROS installation](https://gazebosim.org/docs/latest/ros_installation/), [releases](https://gazebosim.org/docs/latest/releases/)):

| Gazebo | Supported until | Recommended with |
|:--|:--|:--|
| Fortress (LTS) | May 2027 | ROS 2 Humble |
| Harmonic (LTS) | May 2029 | ROS 2 Jazzy |
| Ionic | December 2026 | ROS 2 Kilted |
| Jetty (LTS) | May 2031 | ROS 2 Lyrical (released May 2026) and Rolling |

Other combinations are possible with some effort, for example Humble with Harmonic. What changes for a robot like DiffBot or Remo:

- **ROS 2 packages:** [`ros_gz`](https://github.com/gazebosim/ros_gz) connects Gazebo's topics with ROS 2, and [`gz_ros2_control`](https://github.com/ros-controls/gz_ros2_control) replaces `gazebo_ros_control`, so the ros2_control controllers drive the simulated joints.
- **Physics:** the physics engine is [DART](https://dartsim.github.io/) by default, with Bullet as an alternative ([physics in Gazebo](https://gazebosim.org/api/sim/10/physics.html)). Gazebo Classic's default, [ODE](https://www.ode.org/), isn't available ([feature comparison](https://gazebosim.org/docs/latest/comparison/)). So friction and contact settings that were tuned for Gazebo Classic have to be checked again.
- **Lidar:** until 2026, the new Gazebo had only a rendering-based lidar (`gpu_lidar`), which measures distances by rendering the scene. A lidar that the physics engine computes on the CPU, like the `ray` sensor DiffBot and Remo use in Gazebo Classic, was added in 2026 ([gz-sensors#26](https://github.com/gazebosim/gz-sensors/issues/26)). As of October 2026, its sensor library is released for Jetty, but the simulator system that runs it isn't in a Jetty release yet ([backport](https://github.com/gazebosim/gz-sim/commit/475280fb38b4499a77e3771f2388a82587ba2d41)). Older releases, including Harmonic and Ionic, don't have it.
- **Cameras** need rendering, and so does the GPU lidar. A server without a display can still render with `--headless-rendering`, which uses [EGL](https://www.khronos.org/egl); without a GPU, Gazebo falls back to software rendering ([headless rendering](https://gazebosim.org/api/sim/10/headless_rendering.html)). Whether a test with a camera runs fast enough on GitHub's standard runners, which have no GPU, still has to be tried.
- **Migration:** the [migration guide](https://gazebosim.org/docs/latest/gazebo_classic_migration/) covers URDF and SDF files, plugins and ROS 2 packages.

## Alternatives

| Simulator | ROS | Physics | GPU | License |
|:--|:--|:--|:--|:--|
| [Webots](https://cyberbotics.com/) R2025a | ROS 2 with [`webots_ros2`](https://github.com/cyberbotics/webots_ros2) (Humble, Jazzy, Kilted, Rolling); ROS 1 deprecated | [ODE](https://www.ode.org/) | Required, with OpenGL 3.3; on Linux NVIDIA is recommended ([system requirements](https://cyberbotics.com/doc/guide/system-requirements)) | Apache 2.0 |
| [NVIDIA Isaac Sim](https://github.com/isaac-sim/IsaacSim) 6.2 | ROS 2 Humble and Jazzy; ROS 1 deprecated | [PhysX](https://github.com/NVIDIA-Omniverse/PhysX), or [Newton](https://github.com/newton-physics/newton) | Required: an NVIDIA RTX, at least an RTX 4080 with 16 GB ([requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)) | Apache 2.0 for its code; the Omniverse Kit SDK and the assets under NVIDIA's own license |
| [CoppeliaSim](https://www.coppeliarobotics.com/) 4.10 | ROS 2 with simROS2 (Humble), ROS 1 with simROS | [Bullet](https://github.com/bulletphysics/bullet3), ODE, MuJoCo, Vortex or Newton | Required for vision sensors | Edu for students and teachers only, Pro for everyone else ([licensing](https://manual.coppeliarobotics.com/en/licensing.htm)) |
| [MuJoCo](https://github.com/google-deepmind/mujoco) 3.16 | No ROS interface of its own; ROS 2 with [`mujoco_ros2_control`](https://github.com/ros-controls/mujoco_ros2_control) | MuJoCo | Not required; with one, MJX or MuJoCo Warp run many simulations in parallel | Apache 2.0 |
| [Stage](https://github.com/rtv/Stage) | ROS 1 with [`stage_ros`](http://wiki.ros.org/stage_ros); ROS 2 with [`stage_ros2`](https://github.com/tuw-robotics/stage_ros2) and its [Stage fork](https://github.com/tuw-robotics/Stage) | 2D | Not required | GPL-2.0 (Stage), GPL-3.0 (`stage_ros2`) |
| [Flatland](https://flatland-simulator.readthedocs.io/) | ROS 1, and ROS 2 on its [`ros2` branch](https://github.com/avidbots/flatland/tree/ros2) | 2D | Not required | BSD-3 |

What each is good at:

- **Webots** comes with many robot models, sensors and examples, for research and teaching. It runs on Linux, Windows and macOS. In CI it runs in its Docker image with a virtual display ([installation](https://cyberbotics.com/doc/guide/installation-procedure)).
- **Isaac Sim** renders photorealistic camera images and simulates RTX lidars, for camera-heavy perception and for generating labeled training data. [Isaac Lab](https://github.com/isaac-sim/IsaacLab) builds on it for reinforcement and imitation learning. It needs a powerful NVIDIA GPU, so it doesn't run on GitHub's standard runners.
- **CoppeliaSim** offers several physics engines in one tool, and scripts in Lua or Python. Its free license is for education only.
- **MuJoCo** is a fast, accurate physics engine for contact-rich tasks and for reinforcement learning with many simulations in parallel. It has its own [cameras](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera), which it can render, and [range sensors](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-rangefinder). ROS topics for the sensors, and the connection to ros2_control, come from packages such as `mujoco_ros2_control` and its plugins; a scanning lidar that publishes a laser scan needs such a plugin or your own setup.
- **Stage and Flatland** simulate a robot in 2D: fast, light and enough to test navigation, without 3D physics. [Flatland](https://github.com/avidbots/flatland) is maintained by Avidbots. Stage's original repository hasn't changed since 2020; for ROS 2, `stage_ros2` comes with its own maintained fork of it.

## Which one to use

- **ROS 1 Noetic:** Gazebo Classic 11, as now. The robots' URDFs, worlds and the Simulation Test are made and tested for it.
- **The ROS 2 port:** the new Gazebo, with the release recommended for the chosen ROS 2 distribution: Jazzy with Harmonic (supported until May 2029) or Lyrical with Jetty (until May 2031). Which distribution the port targets is still open in the roadmap. `gz_ros2_control` keeps the setup of the ros_control world: the same controllers on the simulated and the real robot. A test like the Simulation Test, which only needs the odometry and the true pose, runs on Gazebo's server without rendering.
- **Another simulator** is the better choice for some work:
    - camera-heavy perception, or training data for a neural network: Isaac Sim, if an RTX GPU is available;
    - reinforcement learning with many simulations in parallel: MuJoCo, or Isaac Lab;
    - teaching on Windows or macOS with ready-made robots: Webots;
    - quick tests of navigation in 2D: Stage or Flatland.

## Limits of simulation

A simulation is only as good as its model, and the results differ from the real robot:

- **Contacts and friction:** the physics engine approximates wheel contacts with a few points and parameters such as friction, stiffness and damping. A very high friction without contact damping can make the wheels jitter, for example Remo's when it turns on the spot ([diffbot#116](https://github.com/ros-mobile-robots/diffbot/issues/116)). And Gazebo can put the contacts of a cylinder-shaped wheel at its outer edges: the robot then turns less than its wheels say, as if they were farther apart, about 15 % less for DiffBot's wheels. A sphere-shaped collision avoids that ([diffbot#120](https://github.com/ros-mobile-robots/diffbot/pull/120)).
- **The model's geometry and masses:** the controller converts wheel turns to distance with its configured wheel radius. If that doesn't match the URDF, the odometry and the simulated robot disagree: a configured radius 1.8 % smaller than the URDF's, for example, makes the robot drive 1.8 % farther than its odometry reports ([diffbot#118](https://github.com/ros-mobile-robots/diffbot/issues/118)).
- **Control:** in Gazebo, a PID in `gazebo_ros_control` with gains for the simulation turns the velocity command into wheel torque; on the real robot, the Teensy's PID and the motors do that. Their responses differ.
- **Timing:** the simulation runs in steps, and the commands arrive asynchronously, so repeated runs can differ, more so with a model that reacts sensitively.

[Why the results differ from the commands](development/simulation-test.md#why-the-results-differ-from-the-commands) on the Simulation Test page shows these effects with measured numbers. A test that passes in the simulation shows that the model and the configuration work together; it doesn't replace a test on the real robot.
