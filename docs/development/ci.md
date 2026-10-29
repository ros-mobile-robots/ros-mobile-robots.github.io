# Testing and CI

Every pull request and every push to the default branch runs automated checks with [GitHub Actions](https://docs.github.com/actions). A PR is only merged when they pass.

## diffbot

| Workflow | File | What it checks |
|:---------|:-----|:---------------|
| CI | [`.github/workflows/diffbot_ci_action.yml`]({{ diffbot_repo_url }}/.github/workflows/diffbot_ci_action.yml) | Builds and tests all catkin packages with [industrial_ci](https://github.com/ros-industrial/industrial_ci), against ROS Noetic packages from the ROS `testing` and `main` repositories (one job each) |
| Firmware | [`.github/workflows/firmware.yml`]({{ diffbot_repo_url }}/.github/workflows/firmware.yml) | Builds the Teensy firmware in [`diffbot_firmware`](https://github.com/ros-mobile-robots/diffbot/tree/noetic-devel/diffbot_firmware) with [PlatformIO](https://platformio.org/), in the firmware's container ([`.docker/firmware`]({{ diffbot_repo_url }}/.docker/firmware)), once for each board in its [`platformio.ini`]({{ diffbot_repo_url }}/diffbot_firmware/platformio.ini): `teensy40` (Teensy 4.0) and `teensy31` (Teensy 3.1/3.2) |
| Dev container | [`.github/workflows/devcontainer.yml`]({{ diffbot_repo_url }}/.github/workflows/devcontainer.yml) | Builds the [dev container](dev-container.md) image, creates the container (which builds the workspace), then builds and runs the tests in it |
| Robot container | [`.github/workflows/robot_container.yml`]({{ diffbot_repo_url }}/.github/workflows/robot_container.yml) | Builds the [robot image](../processing_units/robot-container/index.md) when it or its packages change, natively on GitHub's arm64 and amd64 runners. It checks the settings template and runs the image's smoke test without hardware, in the image and through Docker Compose |

### industrial_ci

industrial_ci starts a ROS Docker image, installs the packages' dependencies with rosdep, builds the workspace with catkin and runs the tests. The workflow runs it twice: with ROS packages from the `main` repository, which users install, and from `testing`, where new package versions appear first ([ROS testing repository](http://wiki.ros.org/ShadowRepository)).

The [simulation test](simulation-test.md) also drives Remo, whose model `remo_description` isn't a released ROS package, so rosdep can't install it. `UPSTREAM_WORKSPACE: diffbot_dev.repos` in the workflow makes industrial_ci clone the repositories in this [vcstool](https://github.com/dirk-thomas/vcstool) file, the same [source dependencies](../packages/packages-setup.md#source-dependencies) that the [dev container](dev-container.md) imports. industrial_ci builds them first, in an upstream workspace, then builds the diffbot packages on top of it and runs only their tests ([workspace management](https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#workspace-management)).

[ccache](https://ccache.dev/) speeds up the C++ builds. The workflow stores its cache with `actions/cache`. The cache key contains the run ID, so each successful run saves a new cache, and `restore-keys` loads the newest one at the start of the next run.

### Dev container workflow

[`devcontainers/ci`](https://github.com/devcontainers/ci) creates the container exactly as VS Code or the Dev Container CLI would, including [`setup.sh`]({{ diffbot_repo_url }}/.devcontainer/noetic/setup.sh), which builds the workspace. Then it runs:

```console
catkin build --catkin-make-args run_tests
catkin_test_results build
```

So a broken Dockerfile, a missing dependency or a failing build shows up in the PR, not on a developer's machine weeks later.

### Tests

**Simulation test:** [`diffbot_gazebo`]({{ diffbot_repo_url }}/diffbot_gazebo) drives DiffBot and Remo in Gazebo and checks where they went, with the robot's odometry and its true pose. Both workflows run it on every pull request. [Simulation Test](simulation-test.md) explains what it checks, how Gazebo and ros_control work together in it, and how to run it and add checks.

The other packages have no tests yet. So for most changes the checks prove that everything builds, and a pull request describes how it was tested, for example in the simulation or on the robot. Tests for `diffbot_base` and the firmware without the robot are planned in [diffbot#114](https://github.com/ros-mobile-robots/diffbot/issues/114).

**Robot container smoke test:** the [robot image](../processing_units/robot-container/image.md#how-its-tested) is checked without hardware: its packages and programs, the robot models, the launch files, and Remo's bringup with both controllers running. Whether the robot drives with it needs a test on the robot ([diffbot#109](https://github.com/ros-mobile-robots/diffbot/issues/109)).

### GitHub Actions versions

The workflows use the latest major versions of the GitHub actions, which run on Node 24. GitHub retires old versions: in 2026 the CI workflow failed before building anything because `actions/cache@v2` had been switched off ([GitHub notice](https://github.blog/changelog/2024-12-05-notice-of-upcoming-releases-and-breaking-changes-for-github-actions/)), and Node 20 actions were retired in September 2026 ([GitHub changelog](https://github.blog/changelog/2026-09-23-node-20-is-no-longer-available-in-github-actions/)). When a check fails during "Set up job", an outdated action is the likely cause.

### Writing tests and debugging

- **Tests:** ROS 1 packages use [gtest](https://github.com/google/googletest) for C++ unit tests and [rostest](http://wiki.ros.org/rostest) for tests that start ROS nodes. [Ros-Test-Example](https://github.com/steup/Ros-Test-Example) shows both in a catkin workspace ([slides](https://github.com/steup/Ros-Test-Example/blob/master/src/cars/doc/slides/slides.pdf)). catkin-tools explains [building and running tests](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html#building-and-running-tests).
- **Debugging:** a debugger can only stop at breakpoints in a workspace built with debug symbols:

    ```console
    catkin build --save-config --cmake-args -DCMAKE_BUILD_TYPE=Debug
    ```

    See the [catkin-tools cheat sheet](https://catkin-tools.readthedocs.io/en/latest/cheat_sheet.html) for more.

### Running the checks locally

- **Workspace build and tests:** in the [dev container](dev-container.md), in `~/catkin_ws`, run the two commands from the [dev container workflow](#dev-container-workflow) above.
- **Firmware:** build it in its own container, [`.docker/firmware`]({{ diffbot_repo_url }}/.docker/firmware), as CI does. It needs only Docker, not PlatformIO on your PC. The script [`.docker/firmware/pio`]({{ diffbot_repo_url }}/.docker/firmware/pio) runs PlatformIO's `pio` command in that container. From the diffbot folder:

    ```console
    .docker/firmware/pio run -e teensy40
    .docker/firmware/pio run -e teensy31
    ```

    Run these commands in a terminal on your PC, in your diffbot clone: on Windows, in the Ubuntu of WSL 2, not inside the dev container. The first build needs the internet and took about a minute on our test PC, and later builds took seconds. The firmware file is then `diffbot_firmware/.pio/build/<board>/firmware.hex`, which PJRC's [Teensy Loader](https://www.pjrc.com/teensy/loader.html) can flash.

    ??? info "How the firmware container works"
        - **Why a container of its own:** the ROS [dev container](dev-container.md) runs Ubuntu 20.04, whose C library is too old for the current PlatformIO Teensy tools: they need glibc 2.34 or later. In our test, the Teensy tool `teensy_size` stopped with a missing `GLIBC_2.34` there. The firmware container runs Debian 12, with glibc 2.36, and doesn't need ROS.
        - **The image** ([`Dockerfile`]({{ diffbot_repo_url }}/.docker/firmware/Dockerfile)): Python 3.12, PlatformIO 6.2.0, pinned in `requirements.txt`, and a compiler for programs that run on the PC, such as unit tests. The Teensy toolchain isn't in the image: PlatformIO downloads it on the first build.
        - **The script** first builds the image with [Docker Compose](https://docs.docker.com/compose/) and [`compose.yaml`]({{ diffbot_repo_url }}/.docker/firmware/compose.yaml). The first build shows Docker's progress. Later builds took about 2 seconds on our test PC, because Docker reuses the cached image layers whose inputs haven't changed. They only show their output when they fail. Then it runs `pio` with your arguments in the container, in `diffbot_firmware`. In a script or in CI, without a terminal, it runs the container without one.
        - **Your user:** the script passes your user and group ID (`id -u` and `id -g`) to Compose as `LOCAL_UID` and `LOCAL_GID`. The build creates a user with these IDs in the image, and the container runs as that user. So the files that the build writes into `diffbot_firmware/.pio` belong to you. The image and its volume carry both IDs in their names, so another user on the same PC gets their own. To use other IDs, set both variables, for example `LOCAL_UID=1001 LOCAL_GID=1001 .docker/firmware/pio run -e teensy40`.
        - **The downloads:** PlatformIO keeps them in the container's folder `/pio-core`. On your PC, that's a Docker [volume](https://docs.docker.com/engine/storage/volumes/), a storage area that Docker keeps between runs, so only the first build downloads. Don't build in the container and on the PC at the same time, because both write to `diffbot_firmware/.pio`.
        - **In CI:** the Firmware workflow runs the same script. Each CI job starts on a fresh machine, where a Docker volume would be empty. So the workflow sets `PIO_CORE` to a folder on the machine, which `compose.yaml` then mounts as `/pio-core` instead of the volume, and [`actions/cache`](https://github.com/actions/cache) saves that folder between runs.
        - **Planned** in [diffbot#149](https://github.com/ros-mobile-robots/diffbot/issues/149): flashing from the container, and a VS Code dev container for the firmware with the same image.

    Or install PlatformIO on your PC. These steps were tested on Ubuntu 24.04 (WSL 2) with Python 3.12 and PlatformIO 6.2. From the diffbot folder:

    ```console
    sudo apt install python3-venv
    python3 -m venv ~/.venvs/platformio
    ~/.venvs/platformio/bin/pip install platformio
    cd diffbot_firmware
    ~/.venvs/platformio/bin/pio run
    ```

    `pio run` builds the Teensy 4.0, the default environment, and `pio run -e teensy31` the Teensy 3.1/3.2.

## This documentation site

| Workflow | File | What it does |
|:---------|:-----|:-------------|
| Documentation CI | [`.github/workflows/ci.yml`]({{ docs_repo_url }}/.github/workflows/ci.yml) | Builds the site with `mkdocs build --strict`, which fails on any warning ([strict mode](https://www.mkdocs.org/user-guide/configuration/#strict)), such as a broken link or anchor ([link validation](https://www.mkdocs.org/user-guide/configuration/#validation)). On a push to `main` it also publishes the site to the `gh-pages` branch, which GitHub Pages serves at ros-mobile-robots.com. |
| Lint | [`.github/workflows/lint.yml`]({{ docs_repo_url }}/.github/workflows/lint.yml) | Checks the spelling with [codespell](https://github.com/codespell-project/codespell); its settings are in [`.codespellrc`]({{ docs_repo_url }}/.codespellrc). |
| PR preview | [`.github/workflows/preview.yml`]({{ docs_repo_url }}/.github/workflows/preview.yml) | Builds each pull request from a branch in this repository and publishes it at `https://ros-mobile-robots.com/pr-preview/pr-<number>/`, so changes can be checked on the real site before merging. The preview is removed when the PR is closed. Pull requests from forks get no preview, only the build and spelling checks. |

To build the site locally:

```console
pip install -r requirements.txt
mkdocs serve
```
