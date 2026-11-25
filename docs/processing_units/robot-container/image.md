# Image and Tests

The [robot container](index.md) runs ROS Noetic and the robot's packages on the Raspberry Pi from one Docker image, which CI builds and the Raspberry Pi downloads. This page explains how the image is built, what takes its space, and how CI tests and publishes the image. You don't need the page to use the robot container. The page is for changing the image, or for understanding the image. The [Code Walkthrough](walkthrough.md) goes through the files behind the image, block by block.

The page follows the image's way: [how it's built](#how-the-image-is-built), [what takes the space](#what-takes-the-space), [how it's tested](#how-its-tested) and [how it's published](#how-its-published).

## How the image is built

The robot's Docker image is defined in the diffbot repository's [`.docker/robot`]({{ robot_container_src_url }}/.docker/robot) folder. Its [`Dockerfile`]({{ robot_container_src_url }}/.docker/robot/Dockerfile) starts from the official ROS image [`ros:noetic-ros-base`](https://hub.docker.com/_/ros), which exists for arm64 and amd64. The Dockerfile then adds the robot's packages in three steps, with the same tools as a native workspace:

1. **[vcstool](../../packages/packages-setup.md#source-dependencies)** fetches the repositories listed in [`remo_robot.repos`]({{ robot_container_src_url }}/remo_robot.repos): Remo's model [`remo_description`](../../packages/remo_description.md) and the lidar driver [`rplidar_ros`](https://github.com/ros-mobile-robots/rplidar_ros/tree/0c474e311f925f81ffba97313745f80a2de02e68).
2. **[rosdep]({{ robot_container_src_url }}/.docker/robot/Dockerfile#L72-L81)** installs the Ubuntu and ROS packages that the packages' [`package.xml`](https://www.ros.org/reps/rep-0140.html) files declare as dependencies.
3. **[catkin](../ros-setup.md#build-tool-catkin_tools)** builds the packages, in the workspace [`/home/ros/catkin_ws`]({{ robot_container_src_url }}/.docker/robot/Dockerfile#L68).

[The Dockerfile](walkthrough.md#the-dockerfile) in the Code Walkthrough goes through the file block by block: the two stages, the user `ros`, the list of packages, why rosdep gets each package's folder, and what a container runs.

CI builds the image and publishes the image ([How it's published](#how-its-published)). Docker builds the image on the Raspberry Pi only for [your own build](versions.md#your-own-build).

### The same inputs for both builds

CI builds the image twice, once per processor architecture, on two computers ([How it's tested](#how-its-tested)). Three inputs of the build come from outside the diffbot repository. Two of them are named by their content, so both builds get the same files, also when one of the two builds runs again later. The third is named by a release tag:

| Input | How the build names it | Where |
|:--|:--|:--|
| The two base images, [`ubuntu:20.04`]({{ robot_container_src_url }}/.docker/robot/Dockerfile#L24) and [`ros:noetic-ros-base`]({{ robot_container_src_url }}/.docker/robot/Dockerfile#L31) | By tag and digest. A digest is a checksum of an image's content, so it names exactly one content, whatever the tag points to by now | The [Dockerfile](#how-the-image-is-built)'s two [`FROM`](https://github.com/moby/buildkit/blob/v0.26.1/frontend/dockerfile/docs/reference.md#from) lines |
| The lidar driver [`rplidar_ros`]({{ robot_container_src_url }}/remo_robot.repos#L2-L9) | By one commit of its branch `noetic-devel`. A commit names exactly one state of the code | [`remo_robot.repos`]({{ robot_container_src_url }}/remo_robot.repos) |
| Remo's model [`remo_description`]({{ robot_container_src_url }}/remo_robot.repos#L14-L21) | By the release tag `2.5.0`. A tag is a name for a commit. It selects one version, but it isn't a checksum of the content | `remo_robot.repos` |

These lines of the Dockerfile explain the digests and name the first base image ([lines 12–24]({{ robot_container_src_url }}/.docker/robot/Dockerfile#L12-L24)). The second `FROM` line, in line 31, names the ROS image the same way:

```dockerfile title="Dockerfile" linenums="12"
--8<-- "{{ robot_container_src_raw }}/.docker/robot/Dockerfile:12:24"
```

Each digest is the one of the image's index. An index is the list that holds one image per processor architecture. Docker reads the index and picks the image for the computer that builds, so the same line works on the arm64 and on the amd64 computer.

`remo_robot.repos` lists the two repositories that [vcstool](../../packages/packages-setup.md#source-dependencies) fetches, with the lidar driver's commit and the model's release tag in the [`version`]({{ robot_container_src_url }}/remo_robot.repos#L9-L21) fields:

```yaml title="remo_robot.repos" linenums="1"
--8<-- "{{ robot_container_src_raw }}/remo_robot.repos:1:21"
```

One more input isn't fixed at all: the Ubuntu and ROS packages that apt and [rosdep](#how-the-image-is-built) install during the build. They come from the package servers at the time of the build, so two builds at different times can get different package versions.

## What takes the space

Unpacked, the image takes about 3.2 GB on an amd64 PC, and about 0.7 GB compressed. On that PC, `docker images` shows about 3.9 GB, because Docker's containerd image store keeps both ([disk space usage](https://docs.docker.com/engine/storage/containerd/#disk-space-usage)). That's the default for Docker Engine 29 and later on a fresh installation. Installations with the legacy image store keep only the unpacked files. It hasn't been measured on the Raspberry Pi yet. What takes the space, measured with `du` and `dpkg-query` in the image:

- **The official ROS image, 2.4 GB:** besides ROS, it has the tools to build ROS packages, such as the C++ compiler, CMake and Boost's headers. 1.2 GB of it is a Fortran compiler, flang, with the compiler libraries it's built on (LLVM). It comes in through Boost: ROS's [`pluginlib`](https://github.com/ros/pluginlib/blob/1.13.2/pluginlib/package.xml#L35) depends on all of Boost. That includes Boost.MPI, for programs that run on several computers at once, which needs OpenMPI, and OpenMPI's development package needs a Fortran compiler.
- **The robot's part, about 0.7 GB of the unpacked 3.2 GB:** almost all of it comes with RViz, which the robot models declare ([Limitations](index.md#limitations-and-planned-work)): graphics drivers (Mesa), the window toolkit Qt and the 3D engine OGRE. The built workspace itself is about 70 MB.

## How it's tested

The workflow [`robot_container.yml`]({{ robot_container_src_url }}/.github/workflows/robot_container.yml) runs in three cases:

- **For a pull request:** when the pull request changes the image or its packages.
- **For a push to the branch `noetic-devel`:** when the push changes them. A merged pull request is such a push.
- **By hand:** from the repository's Actions tab.

The figure shows what a run does, and where a pull request's run ends:

```mermaid
graph TB
  subgraph BUILD [Job build, once per architecture]
    PRE[Checks before the build:<br/>settings, local build name,<br/>publish script]
    IMG[Build the image]
    SMOKE[Smoke test, directly<br/>and through Compose]
    PRE --> IMG
    IMG --> SMOKE
  end
  PR[Pull request:<br/>passed or failed,<br/>nothing is kept]
  SAVE[noetic-devel:<br/>save the tested<br/>image as an archive]
  PUB[Job publish:<br/>copy both archives<br/>to the registry]
  SMOKE --> PR
  SMOKE --> SAVE
  SAVE --> PUB
```

GitHub provides the computers that the workflow runs on, called runners. The job [`build`](walkthrough.md#the-job-build) runs twice: on an arm64 runner (`ubuntu-24.04-arm`, free for public repositories) and on an amd64 runner. So each runner builds the image natively, for its own architecture, without emulation, in about two minutes. The build step's times in seven runs, and one build on a Raspberry Pi for comparison:

| Where | Shortest build | Longest build |
|:--|:--|:--|
| arm64 runner, `ubuntu-24.04-arm` | 1 min 42 s | 1 min 55 s |
| amd64 runner, `ubuntu-24.04` | 1 min 42 s | 2 min 21 s |
| Raspberry Pi 4 B in Remo, one build | 14 min 20 s | 14 min 20 s |

The Raspberry Pi's build was measured on 31 October 2026, at the diffbot repository's commit [`3541965`](https://github.com/ros-mobile-robots/diffbot/commit/3541965496d50ce629e00054e8d3acd12e4beed5) ([the bench record](https://github.com/ros-mobile-robots/diffbot/issues/160#issuecomment-6550353204)).

Each run of the job `build` does five things, in this order:

1. **It checks the settings template:** [`env.example`]({{ robot_container_src_url }}/.docker/robot/env.example) has to give a valid configuration with [`compose.yaml`](setup.md#how-composeyaml-uses-the-settings).
2. **It checks the name of a local build:** with [`ROBOT_IMAGE`](setup.md#all-settings) set to the published image's name and every service switched on, `compose.yaml` alone must build nothing, and [`compose.build.yaml`](versions.md#on-the-raspberry-pi) must build only `diffbot-robot:local`, alone and combined with `compose.yaml` in both orders.
3. **It tests the publish script** without a registry ([The script's tests](#the-scripts-tests)).
4. **It builds the image:** with `docker build`, for the runner's own architecture.
5. **It runs [`smoke_test.sh`](#one-check-step-by-step) in the image, twice:** directly in a container of the image, and through `compose.yaml` with its mounts and groups.

[The workflow](walkthrough.md#the-workflow) in the Code Walkthrough goes through the workflow's file, job by job.

The smoke test checks four things, without hardware:

- **The packages and programs:** ROS finds the packages, and the programs that talk to the hardware exist.
- **The robot models:** both robots' URDF models expand with [xacro](../../diffbot_gazebo.md#troubleshooting).
- **The launch files:** the bringup's and the lidar's launch files parse. The lidar's launch file names the driver's node [`/diffbot/rplidarNode`]({{ robot_container_src_url }}/.docker/robot/smoke_test.sh#L40-L43), in the robot's namespace, so a connected lidar's scans are on [`/diffbot/scan`](usage.md#3-the-topics-carry-data).
- **Remo's bringup:** the bringup starts, its URDF model loads, and both controllers run.

The test doesn't start the lidar driver, and no check talks to a device. Without a Teensy, [rosserial](../teensy-mcu.md#other-boards-such-as-an-arduino) can't open its port and the hardware interface gets no wheel feedback, so the test can't show that the robot drives. That needs the real robot: [the robot container's status](index.md) says what ran on Remo and what is still open in [diffbot#109](https://github.com/ros-mobile-robots/diffbot/issues/109).

### One check, step by step

The last check of the smoke test starts Remo's bringup as the robot would, only without the Teensy. These lines of [`smoke_test.sh`]({{ robot_container_src_url }}/.docker/robot/smoke_test.sh) are the check ([lines 45–58]({{ robot_container_src_url }}/.docker/robot/smoke_test.sh#L45-L58)):

```bash title="smoke_test.sh" linenums="45"
--8<-- "{{ robot_container_src_raw }}/.docker/robot/smoke_test.sh:45:58"
```

1. **What the check sets up** (line 48): [`roslaunch`](https://github.com/ros/ros_comm/tree/25d371664e34ec9d26ee331434de9a38c412c890/tools/roslaunch) starts [`bringup.launch`](../../packages/imu/robot_localization.md#on-the-real-robot) for Remo in the background, with its output in the file `/tmp/bringup.log`. A ROS master runs already, from an earlier step of the script.
2. **What the check waits for** (lines 49–54): once per second, for at most 60 s, the script asks the controller manager for its controllers with the service [`/diffbot/controller_manager/list_controllers`]({{ robot_container_src_url }}/.docker/robot/smoke_test.sh#L51). It counts the controllers in the state `running` in the variable [`running`]({{ robot_container_src_url }}/.docker/robot/smoke_test.sh#L49-L52).
3. **What has to hold** (lines 55–58): `running` is 2, for [`joint_state_controller`]({{ robot_container_src_url }}/diffbot_control/config/diffbot_control.yaml#L9-L20) and [`mobile_base_controller`]({{ robot_container_src_url }}/diffbot_control/config/diffbot_control.yaml#L23-L54). The odometry topic [`/diffbot/mobile_base_controller/odom`]({{ robot_container_src_url }}/.docker/robot/smoke_test.sh#L57) is listed. The parameter [`/diffbot/robot_description`]({{ robot_container_src_url }}/.docker/robot/smoke_test.sh#L58) holds the URDF model. If `running` isn't 2 after 60 s, the script prints the bringup's output and fails.

To run the whole smoke test on your PC, build the image and start the script in a container of the image. Run this command in the root folder of your clone of the diffbot repository:

```console
docker build -f .docker/robot/Dockerfile -t diffbot-robot:noetic . &&
  docker run --rm -v "$PWD/.docker/robot/smoke_test.sh:/smoke_test.sh:ro" diffbot-robot:noetic bash /smoke_test.sh
```

`&&` starts the test only if the build succeeded. Without it, a failed build would leave an image from an earlier build under the same name, and the test would run in that older image.

The output has one heading per check, starting with `==`. Recorded on an amd64 PC, it ends like this:

```text
== Remo's bringup starts without hardware: the model loads and both controllers run
running controllers: 2
/diffbot/mobile_base_controller/odom
Smoke test passed
```

## How it's published

For a pull request, the run ends after the smoke tests. The result is the workflow's check, passed or failed, and the image is deleted with the runner. Nothing is published, because a pull request's code isn't reviewed yet.

After a change is merged into `noetic-devel`, each run of the job `build` also saves the image that it tested, as an archive. A third job, `publish`, then copies both archives to the registry and combines them under one name. The job doesn't build the image again. So the published image is the very image that passed the smoke test, not a second build that could differ from it. A Raspberry Pi then downloads the arm64 image and a PC the amd64 one. The version `noetic-<commit>` is published once and never replaced, and `noetic` only moves to a later commit ([Versions](versions.md#versions)).

The Code Walkthrough goes through the code of the publication:

- **[The job `build`](walkthrough.md#the-job-build):** the steps that save the tested image and hand the image over.
- **[The job `publish`](walkthrough.md#the-job-publish):** the conditions, the rights and the steps of the job that publishes.
- **[The script that publishes](walkthrough.md#the-script-that-publishes):** what the script asks the registry first, what the script checks before the script writes, and how the first publication differs.

### The script's tests

[`test_publish_image.sh`]({{ robot_container_src_url }}/.docker/robot/test_publish_image.sh) runs [`publish_image.sh`](walkthrough.md#the-script-that-publishes) without a registry and without images. Stand-ins take the place of [`skopeo`]({{ robot_container_src_url }}/.docker/robot/test_publish_image.sh#L52-L127) and [`docker`]({{ robot_container_src_url }}/.docker/robot/test_publish_image.sh#L130-L161). They answer the script's reads with texts recorded from skopeo, and they note each write that the script attempts instead of doing it. Each test chooses the registry's answer, runs the script, and compares the script's exit code and its writes with the expected ones:

| The registry's answer in the test | Expected writes | Expected result |
|:--|:--|:--|
| *manifest unknown* for both names | Both images, the version and `noetic` | The script succeeds |
| A login helper that isn't installed | None | The script fails |
| The token service answers 404 | None | The script fails |
| A 404 page without the registry's error code | None | The script fails |
| The registry can't be reached | None | The script fails |
| The read is denied | None | The script fails |
| The repository is unknown | None | The script fails |
| The read is denied, in a first publication | Only `first-publication` | The script fails, because the read stays denied |
| The repository is unknown, in a first publication | Only `first-publication` | The script fails, because the repository stays unknown |
| The login file can't be read, in a first publication | None | The script fails |
| One name is free and the read of the other is denied, in a first publication | None | The script fails |
| The version and `noetic` exist | None | The script succeeds |
| They exist, but one image is from another commit | None | The script fails |
| They exist, but `noetic` has another digest | None | The script fails |

Run the script from the root folder of your clone, with the tool [`jq`](https://jqlang.org/manual/v1.7/) installed:

```console
.docker/robot/test_publish_image.sh
```

The script prints one line per test, starting with `ok` or `FAILED`, and ends with the count:

```text
14 tests, 0 failed
```

[Testing and CI](../../development/ci.md) describes all workflows of the diffbot repository, what each one checks, and how to run the checks on your PC.
