# Image and Tests

How the [robot container's](index.md) Docker image is built, what takes its space, and how CI tests and publishes the image. You don't need this page to use the robot container. The page is for changing the image, or for understanding the image. The [Code Walkthrough](walkthrough.md) goes through the files behind the image, block by block.

## How the image is built

The robot's Docker image is defined in the diffbot repository's [`.docker/robot`]({{ robot_container_src_url }}/.docker/robot) folder. Its [`Dockerfile`]({{ robot_container_src_url }}/.docker/robot/Dockerfile) starts from the official ROS image [`ros:noetic-ros-base`](https://hub.docker.com/_/ros), which exists for arm64 and amd64. The Dockerfile then adds the robot's packages in three steps, with the same tools as a native workspace:

1. **vcstool** fetches the repositories listed in [`remo_robot.repos`]({{ robot_container_src_url }}/remo_robot.repos): Remo's model `remo_description` and the lidar driver `rplidar_ros`.
2. **rosdep** installs the Ubuntu and ROS packages that the packages' `package.xml` files declare as dependencies.
3. **catkin** builds the packages, in the workspace `/home/ros/catkin_ws`.

[The Dockerfile](walkthrough.md#the-dockerfile) in the Code Walkthrough goes through the file block by block: the two stages, the user `ros`, the list of packages, why rosdep gets each package's folder, and what a container runs.

CI builds the image and publishes the image ([How it's tested](#how-its-tested)). Docker builds the image on the Raspberry Pi only for [your own build](versions.md#your-own-build).

## What takes the space

Unpacked, the image takes about 3.2 GB on an amd64 PC, and about 0.7 GB compressed. On that PC, `docker images` shows about 3.9 GB, because Docker's containerd image store keeps both ([disk space usage](https://docs.docker.com/engine/storage/containerd/#disk-space-usage)). That's the default for Docker Engine 29 and later on a fresh installation. Installations with the legacy image store keep only the unpacked files. It hasn't been measured on the Raspberry Pi yet. What takes the space, measured with `du` and `dpkg-query` in the image:

- **The official ROS image, 2.4 GB:** besides ROS, it has the tools to build ROS packages, such as the C++ compiler, CMake and Boost's headers. 1.2 GB of it is a Fortran compiler, flang, with the compiler libraries it's built on (LLVM). It comes in through Boost: ROS's `pluginlib` depends on all of Boost. That includes Boost.MPI, for programs that run on several computers at once, which needs OpenMPI, and OpenMPI's development package needs a Fortran compiler.
- **The robot's part, about 0.7 GB of the unpacked 3.2 GB:** almost all of it comes with RViz, which the robot models declare ([Limitations](index.md#limitations-and-planned-work)): graphics drivers (Mesa), the window toolkit Qt and the 3D engine OGRE. The built workspace itself is about 70 MB.

## How it's tested

The workflow [`robot_container.yml`]({{ robot_container_src_url }}/.github/workflows/robot_container.yml) runs for every pull request and push that changes the image or its packages. GitHub provides the computers it runs on, called runners. It runs twice: on an arm64 runner (`ubuntu-24.04-arm`, free for public repositories) and on an amd64 runner. So each builds the image natively, for its own architecture, without emulation, in about two minutes. The build step's times in seven runs:

| Runner | Shortest build | Longest build |
|:--|:--|:--|
| arm64, `ubuntu-24.04-arm` | 1 min 42 s | 1 min 55 s |
| amd64, `ubuntu-24.04` | 1 min 42 s | 2 min 21 s |

[The workflow](walkthrough.md#the-workflow) in the Code Walkthrough goes through the workflow's file, job by job.

Each run checks that `env.example` gives a valid configuration, builds the image, and runs [`smoke_test.sh`]({{ robot_container_src_url }}/.docker/robot/smoke_test.sh) in it twice: directly in the image, and through `compose.yaml` with its mounts and groups. The test checks, without hardware, that:

- the packages and the programs that talk to the hardware are there,
- both robots' URDF models expand with xacro,
- the launch files parse, and the lidar driver runs in the robot's namespace, so its scans are on `/diffbot/scan`,
- Remo's bringup starts: its URDF model loads and both controllers run.

For a pull request, the result is the workflow's check, passed or failed, and the image is deleted with the runner. After a change is merged into `noetic-devel`, a third job publishes the two tested images under one name. The version `noetic-<commit>` is published once and never replaced, and `noetic` only moves to a later commit. A Raspberry Pi then downloads the arm64 image and a PC the amd64 one. [The script that publishes](walkthrough.md#the-script-that-publishes) in the Code Walkthrough explains the job's script: what the script checks before the script writes, and how the first publication differs.

Without a Teensy, rosserial can't open its port and the hardware interface gets no wheel feedback, so the test can't show that the robot drives. That needs the real robot: teleop and lidar scans on a Raspberry Pi in Remo, the open test in [diffbot#109](https://github.com/ros-mobile-robots/diffbot/issues/109).

[Testing and CI](../../development/ci.md) describes all workflows of the diffbot repository, what each one checks, and how to run the checks on your PC.
