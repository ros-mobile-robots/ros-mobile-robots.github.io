# Testing and CI

Every pull request and every push to the default branch runs automated checks with [GitHub Actions](https://docs.github.com/actions). A PR is only merged when they pass.

## diffbot

| Workflow | File | What it checks |
|:---------|:-----|:---------------|
| CI | [`.github/workflows/diffbot_ci_action.yml`]({{ diffbot_repo_url }}/.github/workflows/diffbot_ci_action.yml) | Builds and tests all catkin packages with [industrial_ci](https://github.com/ros-industrial/industrial_ci), against ROS Noetic packages from the ROS `testing` and `main` repositories (one job each) |
| Build base controller | [`.github/workflows/build_base_controller.yml`]({{ diffbot_repo_url }}/.github/workflows/build_base_controller.yml) | Builds the Teensy firmware in [`diffbot_base/scripts/base_controller`](https://github.com/ros-mobile-robots/diffbot/tree/noetic-devel/diffbot_base/scripts/base_controller) with [PlatformIO](https://platformio.org/), once for each board in its [`platformio.ini`]({{ diffbot_repo_url }}/diffbot_base/scripts/base_controller/platformio.ini): `teensy40` (Teensy 4.0) and `teensy31` (Teensy 3.1/3.2) |
| Dev container | [`.github/workflows/devcontainer.yml`]({{ diffbot_repo_url }}/.github/workflows/devcontainer.yml) | Builds the [dev container](dev-container.md) image, creates the container (which builds the workspace), then builds and runs the tests in it |

### industrial_ci

industrial_ci starts a ROS Docker image, installs the packages' dependencies with rosdep, builds the workspace with catkin and runs the tests. The workflow runs it twice: with ROS packages from the `main` repository, which users install, and from `testing`, where new package versions appear first.

[ccache](https://ccache.dev/) speeds up the C++ builds. The workflow stores its cache with `actions/cache`. The cache key contains the run ID, so each successful run saves a new cache, and `restore-keys` loads the newest one at the start of the next run.

### Dev container workflow

[`devcontainers/ci`](https://github.com/devcontainers/ci) creates the container exactly as VS Code or the Dev Container CLI would, including [`setup.sh`]({{ diffbot_repo_url }}/.devcontainer/noetic/setup.sh), which builds the workspace. Then it runs:

```console
catkin build --catkin-make-args run_tests
catkin_test_results build
```

So a broken Dockerfile, a missing dependency or a failing build shows up in the PR, not on a developer's machine weeks later.

### Tests

The packages have no test cases yet, so the checks prove that everything builds, not that it behaves correctly. Tests are planned as part of the [roadmap](index.md#roadmap).

### GitHub Actions versions

The workflows use the latest major versions of the GitHub actions, which run on Node 24. GitHub retires old versions: in 2026 the CI workflow failed before building anything because `actions/cache@v2` had been switched off, and Node 20 actions were retired in September 2026. When a check fails during "Set up job", an outdated action is the likely cause.

### Writing tests and debugging

- **Tests:** ROS 1 packages use [gtest](https://github.com/google/googletest) for C++ unit tests and [rostest](http://wiki.ros.org/rostest) for tests that start ROS nodes. [Ros-Test-Example](https://github.com/steup/Ros-Test-Example) shows both in a catkin workspace ([slides](https://github.com/steup/Ros-Test-Example/blob/master/src/cars/doc/slides/slides.pdf)). catkin-tools explains [building and running tests](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html#building-and-running-tests).
- **Debugging:** a debugger can only stop at breakpoints in a workspace built with debug symbols:

    ```console
    catkin build --save-config --cmake-args -DCMAKE_BUILD_TYPE=Debug
    ```

    See the [catkin-tools cheat sheet](https://catkin-tools.readthedocs.io/en/latest/cheat_sheet.html) for more.

### Running the checks locally

- **Workspace build and tests:** in the [dev container](dev-container.md), in `~/catkin_ws`, run the two commands from the [dev container workflow](#dev-container-workflow) above.
- **Firmware:** build it on the host, not in the [ROS container](dev-container.md): the current PlatformIO Teensy tools need a newer C library (glibc 2.34 or later) than Ubuntu 20.04 has. CI builds on Ubuntu 24.04, and the steps below were tested on Ubuntu 24.04 (WSL 2) with Python 3.12 and PlatformIO 6.2. From the diffbot folder:

    ```console
    sudo apt install python3-venv
    python3 -m venv ~/.venvs/platformio
    ~/.venvs/platformio/bin/pip install platformio
    cd diffbot_base/scripts/base_controller
    ~/.venvs/platformio/bin/pio run
    ```

    `pio run` builds the Teensy 4.0, the default environment; `pio run -e teensy31` builds the Teensy 3.1/3.2.

## This documentation site

| Workflow | File | What it does |
|:---------|:-----|:-------------|
| Documentation CI | [`.github/workflows/ci.yml`]({{ docs_repo_url }}/.github/workflows/ci.yml) | Builds the site with `mkdocs build --strict`, which fails on any warning, such as a broken link. On a push to `main` it also publishes the site to the `gh-pages` branch, which GitHub Pages serves at ros-mobile-robots.com. |
| Lint | [`.github/workflows/lint.yml`]({{ docs_repo_url }}/.github/workflows/lint.yml) | Checks the spelling with [codespell](https://github.com/codespell-project/codespell); its settings are in [`.codespellrc`]({{ docs_repo_url }}/.codespellrc). |
| PR preview | [`.github/workflows/preview.yml`]({{ docs_repo_url }}/.github/workflows/preview.yml) | Builds each pull request from a branch in this repository and publishes it at `https://ros-mobile-robots.com/pr-preview/pr-<number>/`, so changes can be checked on the real site before merging. The preview is removed when the PR is closed. Pull requests from forks get no preview, only the build and spelling checks. |

To build the site locally:

```console
pip install -r requirements.txt
mkdocs serve
```
