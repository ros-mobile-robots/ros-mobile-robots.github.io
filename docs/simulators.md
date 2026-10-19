# Simulators

A simulator runs a robot model in a virtual world, with physics and simulated sensors. With it you can develop and test without the robot, try things that would be risky on the real one, and test every change automatically in CI, as the [Simulation Test](development/simulation-test.md) does. This page explains what DiffBot and Remo use, what changes with ROS 2, which other simulators exist and when they're the better choice, and where simulation reaches its limits.

## What DiffBot and Remo use: Gazebo Classic 11

On ROS Noetic, DiffBot and Remo run in [Gazebo Classic 11](https://classic.gazebosim.org/), the Gazebo that comes with ROS Noetic's desktop-full installation and with the [dev container](development/dev-container.md):

- [`diffbot_gazebo`](diffbot_gazebo.md) has the worlds and the launch files. `gazebo_ros` spawns the robot from its URDF and publishes the true pose of every model.
- `gazebo_ros_control` loads [ros_control](diffbot_control.md) into Gazebo, so the same controllers and configuration drive the simulated wheels as the real ones.
- The URDF's `<gazebo>` tags add what only the simulation needs, such as friction and contact settings, the lidar and camera plugins, and the ros_control plugin.
- The Simulation Test runs only the Gazebo server, without a GUI and without a GPU, on GitHub's standard runners.

Gazebo Classic has reached its end of life: [Gazebo 11 is the last major release](https://classic.gazebosim.org/), its support ended on 29 January 2025, and its [repository](https://github.com/gazebosim/gazebo-classic) is archived, with 11.15.1 as the final version. So it gets no more fixes. ROS Noetic, which reached its end of life on 31 May 2025, can't move to the new Gazebo in practice: the ROS 1 bridge `ros_ign` has binary packages only for Gazebo Citadel, which is also past its end of life, and newer Gazebo releases need it built from source ([ros_gz, noetic branch](https://github.com/gazebosim/ros_gz/tree/noetic)). The roadmap keeps ROS 1 Noetic with Gazebo Classic on the `noetic-devel` branch ([Contributing](development/index.md#roadmap)).

## The new Gazebo for ROS 2

The successor of Gazebo Classic is [Gazebo](https://gazebosim.org/), formerly called Ignition (its simulator is `gz-sim`). It's a rewrite, with new names for the tools and plugins. Each release pairs with a ROS 2 distribution ([ROS installation](https://gazebosim.org/docs/latest/ros_installation/), [releases](https://gazebosim.org/docs/latest/releases/)):

| Gazebo | Supported until | Recommended with |
|:--|:--|:--|
| Fortress (LTS) | May 2027 | ROS 2 Humble |
| Harmonic (LTS) | May 2029 | ROS 2 Jazzy |
| Ionic | December 2026 | ROS 2 Kilted |
| Jetty (LTS) | May 2031 | ROS 2 Lyrical (released May 2026) and Rolling |

Other combinations are possible with some effort, for example Humble with Harmonic. For a robot's software:

- [`ros_gz`](https://github.com/gazebosim/ros_gz) connects Gazebo's topics with ROS 2, and [`gz_ros2_control`](https://github.com/ros-controls/gz_ros2_control) replaces `gazebo_ros_control`, so the ros2_control controllers drive the simulated joints.
- The physics engine is [DART](https://gazebosim.org/api/sim/10/physics.html) by default instead of ODE, so friction and contact settings that were tuned for Gazebo Classic have to be checked again.
- Lidars are simulated with the GPU: there is no CPU lidar like Gazebo Classic's ([feature comparison](https://gazebosim.org/docs/latest/comparison/)). A server without a display can still render with `--headless-rendering`, which uses EGL; without a GPU, Gazebo falls back to software rendering ([headless rendering](https://gazebosim.org/api/sim/9/headless_rendering.html)). Whether a test with lidar or camera runs fast enough on GitHub's standard runners, which have no GPU, still has to be tried.
- The [migration guide](https://gazebosim.org/docs/latest/gazebo_classic_migration/) covers URDF and SDF files, plugins and ROS 2 packages.

## Alternatives

| Simulator | ROS | Physics | Hardware | License |
|:--|:--|:--|:--|:--|
| [Webots](https://cyberbotics.com/) R2025a | ROS 2 with [`webots_ros2`](https://github.com/cyberbotics/webots_ros2) (Humble, Jazzy, Kilted, Rolling); ROS 1 deprecated | ODE | A GPU with OpenGL 3.3; on Linux NVIDIA is recommended ([system requirements](https://cyberbotics.com/doc/guide/system-requirements)) | Apache 2.0 |
| [NVIDIA Isaac Sim](https://github.com/isaac-sim/IsaacSim) 6.2 | ROS 2 Humble and Jazzy; ROS 1 deprecated | PhysX, or Newton | An NVIDIA RTX GPU, at least an RTX 4080 with 16 GB ([requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)) | Apache 2.0 for its code; the Omniverse Kit SDK and the assets under NVIDIA's own license |
| [CoppeliaSim](https://www.coppeliarobotics.com/) 4.10 | ROS 2 with simROS2 (Humble), ROS 1 with simROS | Bullet, ODE, MuJoCo, Vortex or Newton | A graphics card for vision sensors | Edu for students and teachers only, Pro for everyone else ([licensing](https://manual.coppeliarobotics.com/en/licensing.htm)) |
| [MuJoCo](https://github.com/google-deepmind/mujoco) 3.16 | No ROS interface of its own; ROS 2 with [`mujoco_ros2_control`](https://github.com/ros-controls/mujoco_ros2_control) | MuJoCo | None needed; a GPU runs many simulations in parallel (MJX, MuJoCo Warp) | Apache 2.0 |
| [Stage](https://github.com/tuw-robotics/stage_ros2), [Flatland](https://github.com/avidbots/flatland) | ROS 2 (Stage), ROS 1 and ROS 2 (Flatland) | 2D | None | GPL-3.0 (stage_ros2), BSD-3 (Flatland) |

What each is good at:

- **Webots** comes with many robot models, sensors and examples, for research and teaching. It runs on Linux, Windows and macOS. In CI it runs in its Docker image with a virtual display ([installation](https://cyberbotics.com/doc/guide/installation-procedure)).
- **Isaac Sim** renders photorealistic camera images and simulates RTX lidars, for camera-heavy perception and for generating labeled training data. [Isaac Lab](https://github.com/isaac-sim/IsaacLab) builds on it for reinforcement and imitation learning. It needs a powerful NVIDIA GPU, so it doesn't run on GitHub's standard runners.
- **CoppeliaSim** offers several physics engines in one tool, and scripts in Lua or Python. Its free license is for education only.
- **MuJoCo** is a fast, accurate physics engine for contact-rich tasks and for reinforcement learning with many simulations in parallel. Sensors like lidars and cameras come from plugins, for example `mujoco_ros2_control`'s, not from MuJoCo itself.
- **Stage and Flatland** simulate a robot in 2D: fast, light and enough to test navigation, without 3D physics.

## Which one to use

- **ROS 1 Noetic:** Gazebo Classic 11, as now. Nothing else is still supported with Noetic, and the robots' URDFs, worlds and the Simulation Test are made for it.
- **The ROS 2 port:** the new Gazebo, with the release recommended for the chosen ROS 2 distribution: Jazzy with Harmonic (supported until May 2029) or Lyrical with Jetty (until May 2031). Which distribution the port targets is still open in the roadmap. `gz_ros2_control` keeps the setup of the ros_control world: the same controllers on the simulated and the real robot. A test like the Simulation Test, which only needs the odometry and the true pose, runs on Gazebo's server without rendering.
- **Another simulator** is the better choice for some work:
    - camera-heavy perception, or training data for a neural network: Isaac Sim, if an RTX GPU is available;
    - reinforcement learning with many simulations in parallel: MuJoCo, or Isaac Lab;
    - teaching on Windows or macOS with ready-made robots: Webots;
    - quick tests of navigation in 2D: Stage or Flatland.

## Limits of simulation

A simulation is only as good as its model, and the results differ from the real robot:

- **Contacts and friction:** the physics engine approximates wheel contacts with a few points and parameters such as friction, stiffness and damping. With a friction of 200 and no contact damping, Remo's simulated wheels jittered when the robot turned on the spot ([diffbot#116](https://github.com/ros-mobile-robots/diffbot/issues/116)). And Gazebo put the contacts of DiffBot's cylinder-shaped wheels at their outer edges, so it turned 16 % less than its wheels said, as if they were farther apart; a sphere-shaped collision fixed that ([diffbot#120](https://github.com/ros-mobile-robots/diffbot/pull/120)).
- **The model's geometry and masses:** the controller converts wheel turns to distance with its configured wheel radius. If that doesn't match the URDF, the odometry and the simulated robot disagree: in the Simulation Test, a configured radius 1.8 % smaller than the URDF's made the robots drive 1 cm farther than their odometry reported ([diffbot#118](https://github.com/ros-mobile-robots/diffbot/issues/118)).
- **Control:** in Gazebo, a PID in `gazebo_ros_control` with gains for the simulation turns the velocity command into wheel torque; on the real robot, the Teensy's PID and the motors do that. Their responses differ.
- **Timing:** the simulation runs in steps, and the commands arrive asynchronously, so repeated runs can differ, more so with a model that reacts sensitively.

[Why the results differ from the commands](development/simulation-test.md#why-the-results-differ-from-the-commands) on the Simulation Test page shows these effects with measured numbers. A test that passes in the simulation shows that the model and the configuration work together; it doesn't replace a test on the real robot.
