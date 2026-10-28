# Image and Tests

How the [robot container's](index.md) image is built, what takes its space, and how CI tests it. You don't need this page to use the robot container. It's for changing the image, or for understanding it.

## How the image is built

The image is defined in the diffbot repository's [`.docker/robot`]({{ robot_container_src_url }}/.docker/robot) folder. Its [`Dockerfile`]({{ robot_container_src_url }}/.docker/robot/Dockerfile) starts from the official ROS image [`ros:noetic-ros-base`](https://hub.docker.com/_/ros), which exists for arm64 and amd64, and adds the robot's packages in three steps, with the same tools as a native workspace:

1. **vcstool** fetches the repositories listed in [`remo_robot.repos`]({{ robot_container_src_url }}/remo_robot.repos): Remo's model `remo_description` and the lidar driver `rplidar_ros`.
2. **rosdep** installs the Ubuntu and ROS packages that the packages' `package.xml` files declare as dependencies.
3. **catkin** builds the packages, in the workspace `/home/ros/catkin_ws`.

A [catkin workspace](https://wiki.ros.org/catkin/workspaces) is a folder whose `src` folder holds the source code of ROS packages, which catkin builds together. The image has one, with the diffbot repository and the two fetched repositories in `src` ([folders in the container](setup.md#6-the-diffbot-repository)). The dev container on your PC has its own.

Which packages are built is a list in the Dockerfile ([lines 42–45]({{ robot_container_src_url }}/.docker/robot/Dockerfile#L42-L45)): the packages that the bringup and the lidar need, for both robots.

```dockerfile title="Dockerfile" linenums="42"
--8<-- "{{ robot_container_src_raw }}/.docker/robot/Dockerfile:42:45"
```

The list is needed because rosdep reads only the `package.xml` files it's given. In its `package.xml`, a package declares each other package it needs when the robot runs with an `<exec_depend>` line, for example:

```xml title="diffbot_bringup/package.xml"
<exec_depend>diffbot_base</exec_depend>
```

With this line, rosdep would recognize `diffbot_base` as a package of the workspace. But it wouldn't read `diffbot_base`'s own `package.xml`, so it wouldn't install what `diffbot_base` needs, such as rosserial (our test with rosdep 0.25.1 in the ROS image). That's why rosdep gets the `package.xml` of each package to build.

catkin does follow these lines: `catkin build diffbot_bringup` would first build the workspace packages that `diffbot_bringup` declares. But today `diffbot_bringup` declares only `teleop_twist_keyboard`, not the packages its launch files start. Adding them is the first step of [diffbot#131](https://github.com/ros-mobile-robots/diffbot/issues/131). Gazebo, navigation and SLAM aren't in the image: they run on the PC.

Docker builds the image on the Raspberry Pi with the first `docker compose up -d --build` ([Start](usage.md#start)). The container runs as the user `ros` with UID 1000, not as root.

## What takes the space

Unpacked, the image takes about 3.2 GB on an amd64 PC, and about 0.7 GB compressed. On that PC, `docker images` shows about 3.9 GB, because Docker's containerd image store keeps both ([disk space usage](https://docs.docker.com/engine/storage/containerd/#disk-space-usage)). That's the default for Docker Engine 29 and later on a fresh installation. Installations with the legacy image store keep only the unpacked files. It hasn't been measured on the Raspberry Pi yet. What takes the space, measured with `du` and `dpkg-query` in the image:

- **The official ROS image, 2.4 GB:** besides ROS, it has the tools to build ROS packages, such as the C++ compiler, CMake and Boost's headers. 1.2 GB of it is a Fortran compiler, flang, with the compiler libraries it's built on (LLVM). It comes in through Boost: ROS's `pluginlib` depends on all of Boost. That includes Boost.MPI, for programs that run on several computers at once, which needs OpenMPI, and OpenMPI's development package needs a Fortran compiler.
- **The robot's part, 0.7 GB:** almost all of it comes with RViz, which the robot models declare ([Limitations](index.md#limitations-and-planned-work)): graphics drivers (Mesa), the window toolkit Qt and the 3D engine OGRE. The built workspace itself is about 70 MB.

## How it's tested

The workflow [`robot_container.yml`]({{ robot_container_src_url }}/.github/workflows/robot_container.yml) runs for every pull request and push that changes the image or its packages. GitHub provides the computers it runs on, called runners. It runs twice: on an arm64 runner (`ubuntu-24.04-arm`, free for public repositories) and on an amd64 runner. So each builds the image natively, for its own architecture, without emulation, in about two minutes.

```yaml title="robot_container.yml" linenums="32"
--8<-- "{{ robot_container_src_raw }}/.github/workflows/robot_container.yml:32:38"
```

Each run checks that `env.example` gives a valid configuration, builds the image, and runs [`smoke_test.sh`]({{ robot_container_src_url }}/.docker/robot/smoke_test.sh) in it twice: directly in the image, and through `compose.yaml` with its mounts and groups. The test checks, without hardware, that:

- the packages and the programs that talk to the hardware are there,
- both robot models expand with xacro,
- the launch files parse, and the lidar driver runs in the robot's namespace, so its scans are on `/diffbot/scan`,
- Remo's bringup starts: the robot model loads and both controllers run.

The result is the workflow's check on the pull request, passed or failed. The image itself is deleted with the runner: it isn't published. Without a Teensy, rosserial can't open its port and the hardware interface gets no wheel feedback, so the test can't show that the robot drives. That needs the real robot: teleop and lidar scans on a Raspberry Pi in Remo, the open test in [diffbot#109](https://github.com/ros-mobile-robots/diffbot/issues/109).

[Testing and CI](../../development/ci.md) describes all workflows of the diffbot repository, what each one checks, and how to run the checks on your PC.
