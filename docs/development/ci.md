# Testing and CI

Every pull request and every push to the default branch runs automated checks with [GitHub Actions](https://docs.github.com/actions). A PR is only merged when they pass.

## diffbot

| Workflow | File | What it checks |
|:---------|:-----|:---------------|
| CI | [`.github/workflows/diffbot_ci_action.yml`]({{ diffbot_repo_url }}/.github/workflows/diffbot_ci_action.yml) | Builds and tests all catkin packages with [industrial_ci](https://github.com/ros-industrial/industrial_ci), against ROS Noetic packages from the ROS `testing` and `main` repositories (one job each) |
| Firmware | [`.github/workflows/firmware.yml`]({{ diffbot_repo_url }}/.github/workflows/firmware.yml) | Builds the Teensy firmware in [`diffbot_firmware`](https://github.com/ros-mobile-robots/diffbot/tree/noetic-devel/diffbot_firmware) with [PlatformIO](https://platformio.org/), once for each board in its [`platformio.ini`]({{ diffbot_repo_url }}/diffbot_firmware/platformio.ini): `teensy40` (Teensy 4.0) and `teensy31` (Teensy 3.1/3.2) |
| Dev container | [`.github/workflows/devcontainer.yml`]({{ diffbot_repo_url }}/.github/workflows/devcontainer.yml) | Builds the [dev container](dev-container.md) image, creates the container (which builds the workspace), then builds and runs the tests in it |
| PR description | [`.github/workflows/pr_description.yml`]({{ diffbot_repo_url }}/.github/workflows/pr_description.yml) | Checks that the pull request's description links its documentation: the docs pull request, a page on this site, or `None` ([how](#pr-description)) |
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

### PR description

diffbot's [pull request template]({{ diffbot_repo_url }}/.github/pull_request_template.md) has a placeholder under `## Documentation` for the docs pull request or the page on this site that explains the change. A template only fills in the text when a pull request is opened, so [`pr_description.yml`]({{ ci_src_url }}/.github/workflows/pr_description.yml) checks that the section was filled in.

#### What it accepts

- a docs pull request, as `ros-mobile-robots/ros-mobile-robots.github.io#N` or its URL on GitHub
- a link to a page on ros-mobile-robots.com that already explains the change
- `None` or `None needed`, alone or with a reason after a dash or colon, when nothing user-visible changes

The template's comments don't count. The check looks at the format only. Whether the linked page really explains the change is for the review.

#### When it runs

The excerpts show the workflow at the diffbot commit that this page pins, with its line numbers. [lines 7–20]({{ ci_src_url }}/.github/workflows/pr_description.yml#L7-L20):

```yaml title="pr_description.yml: when it runs" linenums="7"
--8<-- "{{ ci_src_raw }}/.github/workflows/pr_description.yml:7:20"
```

- **`types`:** the workflow runs when a pull request is opened, reopened or edited, and when it gets new commits. So after fixing the description, the new run shows the result, without a new commit. "Re-run jobs" on the old, failed run doesn't help: a re-run repeats the old event, with the old description.
- **`concurrency`:** runs for the same pull request share a group. When a new run starts, for example after another edit, GitHub cancels the one still running.
- **`permissions: {}`:** the job gets a token without any permission, because it only reads the event. Even a mistake in the script couldn't change the repository.

#### How the script works

The job has one step, a Python script, [lines 30–34]({{ ci_src_url }}/.github/workflows/pr_description.yml#L30-L34):

```yaml title="pr_description.yml: the step" linenums="30" hl_lines="2 3"
--8<-- "{{ ci_src_raw }}/.github/workflows/pr_description.yml:30:34"
```

`env` puts the description into the environment variable `BODY`, and `python3 -` runs the script that follows until the line `EOF`. The script removes the template's comments and finds the section from `## Documentation` to the next `##` heading, [lines 40–44]({{ ci_src_url }}/.github/workflows/pr_description.yml#L40-L44):

```yaml title="pr_description.yml: the section" linenums="40"
--8<-- "{{ ci_src_raw }}/.github/workflows/pr_description.yml:40:44"
```

Then it looks for the three accepted forms, [lines 46–67]({{ ci_src_url }}/.github/workflows/pr_description.yml#L46-L67):

```yaml title="pr_description.yml: the accepted forms" linenums="46"
--8<-- "{{ ci_src_raw }}/.github/workflows/pr_description.yml:46:67"
```

- **`docs_pr`:** the docs repository's full reference with a number. The `(?<![\w./-])` before it makes sure that it isn't the end of a longer name or path.
- **`linked()`:** a link counts only with the right host. `urlsplit()` takes the link apart, so `https://ros-mobile-robots.com.example.org` (host `ros-mobile-robots.com.example.org`) and `https://ros-mobile-robots.com@example.org` (host `example.org`) don't count. On github.com, only the docs repository's pull requests count. Punctuation after a bare link, such as the period at the end of a sentence, isn't part of it.
- **`none`:** `None` at the start of a line, alone or with a reason after a dash or colon. "None yet, a docs PR will follow" doesn't count.

Last, it reports the result, [lines 69–81]({{ ci_src_url }}/.github/workflows/pr_description.yml#L69-L81):

```yaml title="pr_description.yml: the result" linenums="69"
--8<-- "{{ ci_src_raw }}/.github/workflows/pr_description.yml:69:81"
```

A line that starts with `::error` is a [workflow command](https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-commands#setting-an-error-message): the runner, the machine that runs the job, turns it into an error annotation on the pull request. The exit code 1 fails the check.

#### Why the description is an environment variable

GitHub replaces each {% raw %}`${{ … }}`{% endraw %} expression in a `run:` script with its value [before the shell runs the script](https://docs.github.com/en/actions/concepts/security/script-injections#understanding-the-risk-of-script-injections). Anyone who opens a pull request writes its description. If the script contained the description as an expression, for example as a Python string, its text would become part of the script: a description with `"""` in it would end the string, and the text after it would run as Python code on the runner. That's a script injection.

With `env`, the description stays data. GitHub puts it into the variable `BODY`, and the script reads it with `os.environ.get("BODY")` as a string, which is never run. GitHub recommends this for untrusted input such as a pull request's title or description ([good practices](https://docs.github.com/en/actions/reference/security/secure-use#good-practices-for-mitigating-script-injection-attacks)).

#### Why the script prints no description

The runner reads every line that the script prints, and runs the workflow commands among them, such as the `::error` line above. If the script printed the description, a description with a line like `::error::…` would create an annotation of its own. So the script prints only fixed text. The runner lists the step's environment, the description included, at the top of the step's log, but it doesn't run commands from that list: on [diffbot#163](https://github.com/ros-mobile-robots/diffbot/pull/163), a description with `::error::` lines created no annotation.

### Tests

Only `diffbot_gazebo` registers catkin tests so far: the simulation test and the geometry test below. The robot image has a smoke test of its own. So for most changes the checks prove that everything builds, and a pull request describes how it was tested, for example in the simulation or on the robot. Unit tests for the firmware on the PC ([diffbot#158](https://github.com/ros-mobile-robots/diffbot/issues/158)) and tests for `diffbot_base` without the robot ([diffbot#114](https://github.com/ros-mobile-robots/diffbot/issues/114)) are planned.

#### Simulation test

[`diffbot_gazebo`]({{ diffbot_repo_url }}/diffbot_gazebo) drives DiffBot and Remo in Gazebo and checks where they went, with the robot's odometry and its true pose. Both workflows run it on every pull request. [Simulation Test](simulation-test.md) explains what it checks, how Gazebo and ros_control work together in it, and how to run it and add checks.

#### Geometry test

[`test_geometry_consistency.py`]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py) checks that each robot's URDF and its controller configuration agree on the wheel radius and the wheel separation, as the simulation's and the real robot's launch files load them. It also checks the wheel joints that the controller drives by name. It runs without Gazebo, for both robots, and catches differences that the simulation test would miss. [Geometry Test](geometry-test.md) explains what it compares, how it works and how to run it.

#### Robot container smoke test

The [robot image](../processing_units/robot-container/image.md#how-its-tested) is checked without hardware: its packages and programs, the robot models, the launch files, and Remo's bringup with both controllers running. Whether the robot drives with it needs a test on the robot ([diffbot#109](https://github.com/ros-mobile-robots/diffbot/issues/109)).

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
- **Firmware:** build it on the host, not in the [ROS container](dev-container.md): the current PlatformIO Teensy tools need a newer C library (glibc 2.34 or later) than Ubuntu 20.04 has. In our test, the Teensy tool `teensy_size` stopped with a missing `GLIBC_2.34` in the ROS container. CI builds on Ubuntu 24.04, and the steps below were tested on Ubuntu 24.04 (WSL 2) with Python 3.12 and PlatformIO 6.2. From the diffbot folder:

    ```console
    sudo apt install python3-venv
    python3 -m venv ~/.venvs/platformio
    ~/.venvs/platformio/bin/pip install platformio
    cd diffbot_firmware
    ~/.venvs/platformio/bin/pio run
    ```

    `pio run` builds the Teensy 4.0, the default environment; `pio run -e teensy31` builds the Teensy 3.1/3.2.

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
