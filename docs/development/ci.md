# Testing and CI

Pull requests and pushes to the default branch run automated checks with [GitHub Actions](https://docs.github.com/actions). Which checks a pull request gets depends on the repository. On this site, the checks also depend on the branch that the pull request targets, as [This documentation site](#this-documentation-site) below explains. A PR is only merged when its checks pass.

## diffbot

| Workflow | File | What it checks |
|:---------|:-----|:---------------|
| CI | [`.github/workflows/diffbot_ci_action.yml`]({{ diffbot_repo_url }}/.github/workflows/diffbot_ci_action.yml) | Builds and tests all [catkin](https://github.com/ros/catkin/blob/0.8.12/doc/howto/format2/catkin_overview.rst) packages with [industrial_ci](https://github.com/ros-industrial/industrial_ci), in one job, against the ROS Noetic packages as they were at Noetic's end of life ([industrial_ci](#industrial_ci) below) |
| Firmware | [`.github/workflows/firmware.yml`]({{ firmware_tests_src_url }}/.github/workflows/firmware.yml) | Builds the Teensy firmware in [`diffbot_firmware`](https://github.com/ros-mobile-robots/diffbot/tree/noetic-devel/diffbot_firmware) with [PlatformIO](https://platformio.org/), in the firmware's container ([`.docker/firmware`]({{ diffbot_repo_url }}/.docker/firmware)), once for each board in its [`platformio.ini`]({{ firmware_tests_src_url }}/diffbot_firmware/platformio.ini): [`teensy40`](firmware-container.md#use-it) (Teensy 4.0) and `teensy31` (Teensy 3.1/3.2). A second job runs the firmware's unit tests on the PC, in the same container: the environment [`native`](firmware-unit-tests.md#how-platformio-keeps-the-suites-apart) with the [Unity](firmware-unit-tests.md#which-framework-for-a-new-test) suites and `native_googletest` with the GoogleTest suites. That job also fails when no test case of one of the two environments passed. Each job also runs the include check, [`tools/check_includes.sh`]({{ firmware_tests_src_url }}/diffbot_firmware/tools/check_includes.sh), on fixed files: clang-tidy reports a C++ name whose header a file doesn't include itself. The second job also runs the tests of the script's flag helper, and checks the script's flags for the PC tests with samples |
| Dev container | [`.github/workflows/devcontainer.yml`]({{ diffbot_repo_url }}/.github/workflows/devcontainer.yml) | Builds the [dev container](dev-container.md) image, creates the container (which builds the workspace), then builds and runs the tests in it |
| PR description | [`.github/workflows/pr_description.yml`]({{ diffbot_repo_url }}/.github/workflows/pr_description.yml) | Checks that the pull request's description links its documentation: the docs pull request, a page on this site, or `None` ([PR description](#pr-description) below) |
| Robot container | [`.github/workflows/robot_container.yml`]({{ diffbot_repo_url }}/.github/workflows/robot_container.yml) | Builds the [robot image](../processing_units/robot-container/index.md) when it or its packages change, natively on GitHub's arm64 and amd64 runners. It checks the settings template and runs the image's smoke test without hardware, in the image and through Docker Compose |

### industrial_ci

industrial_ci starts an Ubuntu 20.04 container and sets up ROS Noetic in it. Then it installs the workspace's dependencies with rosdep, builds the workspace with catkin and runs the tests. The workflow runs industrial_ci in one job, `industrial_ci (noetic, main)`.

For Noetic, both settings of `ROS_REPO`, `main` and `testing`, select the same frozen package repository. `ROS_REPO` names the apt repository that the ROS packages come from. For a ROS distribution that still gets updates, `main` has the packages that users install, and `testing` gets new package versions first ([ROS testing repository](http://wiki.ros.org/ShadowRepository)). Noetic reached its end of life on May 31, 2025, after its final package release ([announcement](https://discourse.openrobotics.org/t/new-packages-for-noetic-2025-05-29-final/44003)). A snapshot is a frozen copy of a package repository on snapshots.ros.org. For an end-of-life distribution, industrial_ci installs the packages from the distribution's final snapshot, with `main` as with `testing` ([`_use_repo_or_final_snapshot`](https://github.com/ros-industrial/industrial_ci/blob/125164b9f1883cdf1858897a7146d1bebf2be5c6/industrial_ci/src/ros.sh#L152-L166) in industrial_ci's `ros.sh`). A job with `testing` would install the same packages, so the workflow has only the job with `main`.

With `main`, the job's log shows industrial_ci's warning `'noetic' is in end-of-life state, ROS_REPO='main' is superfluous`. The `matrix` entry still sets `ROS_REPO: main`, because the entry's values give the job its name, `industrial_ci (noetic, main)`. diffbot's default branch requires a check with that name before a pull request can merge.

The [simulation test](simulation-test.md) also drives Remo, whose model `remo_description` isn't a released ROS package, so rosdep can't install it. `UPSTREAM_WORKSPACE: diffbot_dev.repos` in the workflow makes industrial_ci clone the repositories in this [vcstool](https://github.com/dirk-thomas/vcstool) file, the same [source dependencies](../packages/packages-setup.md#source-dependencies) that the [dev container](dev-container.md) imports. industrial_ci builds them first, in an upstream workspace, then builds the diffbot packages on top of it and runs only their tests ([workspace management](https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#workspace-management)).

[ccache](https://ccache.dev/) keeps compiled C++ files, so that a later build can reuse them. The workflow saves ccache's folder with `actions/cache` at the end of each successful run. The cache key contains the run ID, so each successful run saves a new cache, and `restore-keys` looks for the newest one at the start of the next run. In the current runs, that step finds the newest cache but logs `Failed to restore`, because `tar` can't create the folder `/github/home` on the runner. The build then starts without a cache.

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
- **`permissions: {}`:** the check only reads the event, so it turns off the `GITHUB_TOKEN` permissions that a workflow can set ([permissions](https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-syntax#permissions)). The token can't be used to write to the repository.

#### How the script works

The job has one step, a Python script, [lines 30–34]({{ ci_src_url }}/.github/workflows/pr_description.yml#L30-L34):

```yaml title="pr_description.yml: the step" linenums="30" hl_lines="2 3"
--8<-- "{{ ci_src_raw }}/.github/workflows/pr_description.yml:30:34"
```

`env` puts the description into the environment variable `BODY`. [`python3 -`](https://docs.python.org/3/using/cmdline.html#interface-options) reads Python code from its standard input: the shell's `<<'EOF'` passes it the lines that follow, up to a line with only `EOF`. The quotes around `EOF` keep the shell from changing anything in those lines. The script removes the template's comments and finds the section from `## Documentation` to the next `##` heading or the end of the description, [lines 40–44]({{ ci_src_url }}/.github/workflows/pr_description.yml#L40-L44):

```yaml title="pr_description.yml: the section" linenums="40"
--8<-- "{{ ci_src_raw }}/.github/workflows/pr_description.yml:40:44"
```

Then it looks for the three accepted forms, [lines 46–67]({{ ci_src_url }}/.github/workflows/pr_description.yml#L46-L67):

```yaml title="pr_description.yml: the accepted forms" linenums="46"
--8<-- "{{ ci_src_raw }}/.github/workflows/pr_description.yml:46:67"
```

- **`docs_pr`:** the docs repository's full reference with a number. The `(?<![\w./-])` before it makes sure that it isn't the end of a longer name or path.
- **`linked()`:** a link counts only with `https` and the right host. `urlsplit()` takes the link apart, so `https://ros-mobile-robots.com.example.org` (host `ros-mobile-robots.com.example.org`) and `https://ros-mobile-robots.com@example.org` (host `example.org`) don't count. On github.com, only the docs repository's pull requests count. Punctuation after a bare link, such as the period at the end of a sentence, isn't part of it.
- **`none`:** `None` at the start of a line, alone or with a reason after a dash or colon. "None yet, a docs PR will follow" doesn't count.

Last, it reports the result, [lines 69–82]({{ ci_src_url }}/.github/workflows/pr_description.yml#L69-L82):

```yaml title="pr_description.yml: the result" linenums="69"
--8<-- "{{ ci_src_raw }}/.github/workflows/pr_description.yml:69:82"
```

The printed `::error title=…::…` line is a [workflow command](https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-commands#setting-an-error-message): the runner, the machine that runs the job, turns it into an error annotation on the pull request. The exit code 1 fails the check.

#### Why the description is an environment variable

GitHub replaces each {% raw %}`${{ … }}`{% endraw %} expression in a `run:` script with its value [before the shell runs the script](https://docs.github.com/en/actions/concepts/security/script-injections#understanding-the-risk-of-script-injections). Anyone who opens a pull request writes its description. If the description were inserted between triple double quotes in a Python string, a crafted description could close the string with `"""` and add Python statements of its own, which would run on the runner. That's a script injection.

With `env`, the description stays data. GitHub puts it into the variable `BODY`, and the script reads it with `os.environ.get("BODY")` as a string, which is never run. GitHub recommends this for untrusted input such as a pull request's title or description ([good practices](https://docs.github.com/en/actions/reference/security/secure-use#good-practices-for-mitigating-script-injection-attacks)).

#### Why the script prints no description

The runner reads every line that the script prints, and runs the workflow commands among them, such as the `::error` line above. If the script printed the description, a description with a line like `::error::…` would create an annotation of its own. So the script prints only fixed text. The runner lists the step's environment, the description included, at the top of the step's log, but it doesn't run commands from that list. In a [test run](https://github.com/ros-mobile-robots/diffbot/actions/runs/40921591951/job/123615973689) on [diffbot#163](https://github.com/ros-mobile-robots/diffbot/pull/163), a description with `##[error]` and `::error::` lines showed them in that list and created no annotation.

### Tests

`diffbot_gazebo` and `diffbot_control` register catkin tests: the simulation test, the geometry test and the EKF tests below. The firmware has unit tests on the PC, and the robot image has a smoke test of its own. So for most changes the checks prove that everything builds, and a pull request describes how it was tested, for example in the simulation or on the robot. Tests for `diffbot_base` without the robot are planned ([diffbot#114](https://github.com/ros-mobile-robots/diffbot/issues/114)). So are tests for the rest of the firmware's wheel control, which first moves into code that builds on the PC ([diffbot#158](https://github.com/ros-mobile-robots/diffbot/issues/158)).

#### Simulation test

[`diffbot_gazebo`]({{ diffbot_repo_url }}/diffbot_gazebo) drives DiffBot and Remo in Gazebo and checks where they went, with the robot's odometry and its true pose. Both workflows run it on every pull request. [Simulation Test](simulation-test.md) explains what it checks, how Gazebo and ros_control work together in it, and how to run it and add checks. In the same simulation files, [`test_imu.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_imu.py) checks the simulated IMU, [`test_imu_reset.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_imu_reset.py) that the IMU publishes again after a reset of the simulation, and [`test_odom_tf.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_odom_tf.py) that only the drive controller publishes `odom → base_footprint`.

#### Geometry test

[`test_geometry_consistency.py`]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py) checks that each robot's URDF and its controller configuration agree on the wheel radius and the wheel separation, as the simulation's and the real robot's launch files load them. It also checks the wheel joints that the controller drives by name. It runs without Gazebo, for both robots, and catches differences that the simulation test would miss. [Geometry Test](geometry-test.md) explains what it compares, how it works and how to run it.

#### EKF tests

The [EKF](../packages/imu/robot_localization.md#how-its-tested) that fuses the wheel odometry with the IMU is checked without the robot. In `diffbot_control`, [`test_imu_validator.py`]({{ diffbot_repo_url }}/diffbot_control/test/test_imu_validator.py) checks the IMU sample validator, and [`ekf_synthetic.test`]({{ diffbot_repo_url }}/diffbot_control/test/ekf_synthetic.test) and [`ekf_synthetic_robot_rates.test`]({{ diffbot_repo_url }}/diffbot_control/test/ekf_synthetic_robot_rates.test) run the EKF on a stepped simulated clock, the second at the robot's rates. In `diffbot_gazebo`, both robots drive in Gazebo with a wrong wheel separation, and [`test_ekf_switch.py`]({{ diffbot_repo_url }}/diffbot_gazebo/test/test_ekf_switch.py) checks the `ekf` switch in the launch files.

#### Firmware unit tests

Parts of the Teensy firmware run on the PC, with fake hardware and a fake ROS connection: the IMU code ([IMU (BNO055)](../packages/imu/driver-bno055.md#how-the-firmware-reads-it)), the PID controller, the subscriber for the wheel commands, and the wheel control's command timeout. Each has its test folder in [`diffbot_firmware/test`]({{ firmware_tests_src_url }}/diffbot_firmware/test). PlatformIO builds them for the PC, in the firmware's container like the boards. The tests use two test frameworks, and PlatformIO has an environment for each: [`native`](firmware-unit-tests.md#how-platformio-keeps-the-suites-apart) runs the suites written with [Unity](firmware-unit-tests.md#which-framework-for-a-new-test), and `native_googletest` those written with GoogleTest. The Firmware workflow runs both environments in a job of their own, [`unit tests (native)`]({{ firmware_tests_src_url }}/.github/workflows/firmware.yml#L60-L142). `pio test` also succeeds when no test of an environment passed, for example when a suite's folder lost the prefix `test_` that PlatformIO looks for, or when every test is skipped. So the job has PlatformIO write a JSON report of the run, counts each environment's passed test cases in it, and fails when one of the two environments has none. [Firmware Unit Tests](firmware-unit-tests.md) explains the two frameworks, how to run the suites on your PC and how to write a new one.

#### Robot container smoke test

The [robot image](../processing_units/robot-container/image.md#how-its-tested) is checked without hardware: its packages and programs, the robot models, the launch files, and Remo's bringup with both controllers running. Whether the robot drives with it needs a test on the robot ([diffbot#109](https://github.com/ros-mobile-robots/diffbot/issues/109)).

### GitHub Actions versions

The workflows use the latest major versions of the GitHub actions, which run on Node 24. GitHub retires old versions: in 2026 the CI workflow failed before building anything because `actions/cache@v2` had been switched off ([GitHub notice](https://github.blog/changelog/2024-12-05-notice-of-upcoming-releases-and-breaking-changes-for-github-actions/)), and Node 20 actions were retired in September 2026 ([GitHub changelog](https://github.blog/changelog/2026-09-23-node-20-is-no-longer-available-in-github-actions/)). When a check fails during "Set up job", an outdated action is the likely cause.

### Writing tests and debugging

A new test for a ROS package or for the firmware is registered in one of three ways. Choose by what the test needs:

- **Python test that needs no ROS master:** [nose](https://nose.readthedocs.io/) runs the test file. [`catkin_add_nosetests`](https://docs.ros.org/en/noetic/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-add-nosetests) registers the file in the package's `CMakeLists.txt`, as catkin's how-to [Configuring Python nose tests](https://github.com/ros/catkin/blob/0.8.12/doc/howto/format2/python_nose_configuration.rst) shows. Example: [`test_geometry_consistency.py`]({{ geometry_src_url }}/diffbot_gazebo/test/test_geometry_consistency.py), registered in [`diffbot_gazebo/CMakeLists.txt`]({{ geometry_src_url }}/diffbot_gazebo/CMakeLists.txt#L221-L224).
- **Test that needs running ROS nodes:** [rostest](http://wiki.ros.org/rostest) runs a [`.test`](simulation-test.md#1-the-test-file) file, a launch file that starts the nodes and then the test node. [`add_rostest`](https://github.com/ros/ros_comm/blob/25d371664e34ec9d26ee331434de9a38c412c890/tools/rostest/cmake/rostest-extras.cmake.em#L7-L58) registers the file in the same `CMakeLists.txt`. Example: [`sim_drive_diffbot.test`]({{ diffbot_src_url }}/diffbot_gazebo/test/sim_drive_diffbot.test) of the [simulation test](simulation-test.md).
- **Firmware test on the PC:** PlatformIO's [`native`](#firmware-unit-tests) environment builds and runs a [Unity](https://docs.platformio.org/en/latest/advanced/unit-testing/frameworks/unity.html) test. The test is a folder in [`diffbot_firmware/test`]({{ ekf_src_url }}/diffbot_firmware/test), and its name in [`test_filter` of `platformio.ini`]({{ ekf_src_url }}/diffbot_firmware/platformio.ini#L45-L53) registers the folder. Example: [`test_command_timeout`]({{ ekf_src_url }}/diffbot_firmware/test/test_command_timeout).

A test's name says what the test checks, as a sentence, and a test checks one behavior. The section [Tests in diffbot's AGENTS.md](https://github.com/ros-mobile-robots/diffbot/blob/noetic-devel/AGENTS.md#tests) has the rules. [Running the checks locally](#running-the-checks-locally) has the commands that run all three kinds on your PC. For the first two kinds, these commands build [catkin's `run_tests` target](https://github.com/ros/catkin/blob/0.8.12/doc/howto/format2/run_tests.rst), which catkin-tools describes on its page [catkin test](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_test.html).

A debugger can only stop at breakpoints in a workspace that is built with debug symbols:

```console
catkin build --save-config --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

See the [catkin-tools cheat sheet](https://catkin-tools.readthedocs.io/en/latest/cheat_sheet.html) for more.

### Running the checks locally

- **Workspace build and tests:** in the [dev container](dev-container.md), in `~/catkin_ws`, run the two commands from the [dev container workflow](#dev-container-workflow) above.
- **Firmware:** build it in its own container, as CI does. It needs only Docker, not PlatformIO on your PC. When you change the firmware, build both boards, because the change has to work on both. Run the commands from your diffbot clone's root folder on the PC, not inside the dev container:

    ```console
    .docker/firmware/run.sh pio run -e teensy40
    .docker/firmware/run.sh pio run -e teensy31
    ```

    [Firmware Container](firmware-container.md) explains how to use it, how it works and how your user ID gets into it, and how to install PlatformIO on your PC instead.

- **Firmware unit tests:** run them in the same container, also from the root folder of your diffbot clone:

    ```console
    .docker/firmware/run.sh pio test -e native -e native_googletest
    ```

    PlatformIO builds the selected PC suites in [`diffbot_firmware/test`]({{ firmware_tests_src_url }}/diffbot_firmware/test) and runs them: the [Unity](firmware-unit-tests.md#which-framework-for-a-new-test) suites in the environment [`native`](firmware-unit-tests.md#how-platformio-keeps-the-suites-apart), the GoogleTest suites in `native_googletest`. These commands leave out the sketches for the Teensy that you run by hand, such as `test/encoders`. PlatformIO lists every test with its result, and a summary at the end. [Run the tests](firmware-unit-tests.md#run-the-tests) on the Firmware Unit Tests page shows how to run one test or one suite.

## This documentation site

| Workflow | File | What it does |
|:---------|:-----|:-------------|
| Documentation CI | [`.github/workflows/ci.yml`]({{ docs_repo_url }}/.github/workflows/ci.yml) | Runs the unit tests in [`tests/`](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io/tree/main/tests) and builds the site with `mkdocs build --strict`, which fails on any warning ([strict mode](https://www.mkdocs.org/user-guide/configuration/#strict)), such as a broken link or anchor ([link validation](https://www.mkdocs.org/user-guide/configuration/#validation)). It runs for pull requests into `main` and for pushes to `main`. On a push to `main` it also publishes the site to the `gh-pages` branch, which GitHub Pages serves at ros-mobile-robots.com. |
| Lint | [`.github/workflows/lint.yml`]({{ docs_repo_url }}/.github/workflows/lint.yml) | Checks the spelling with [codespell](https://github.com/codespell-project/codespell). Its settings are in [`.codespellrc`]({{ docs_repo_url }}/.codespellrc). It runs for pull requests into `main`. |
| PR preview | [`.github/workflows/preview.yml`]({{ docs_repo_url }}/.github/workflows/preview.yml) | Builds each pull request from a branch in this repository, whichever branch it targets, and publishes it at `https://ros-mobile-robots.com/pr-preview/pr-<number>/`, so changes can be checked on the real site before merging. The preview is removed when the PR is closed. Pull requests from forks get no preview. |

A pull request into another branch than `main`, a stacked pull request, therefore gets the preview, but no unit tests and no spelling check. A pull request from a fork into such a branch gets none of the three. A maintainer can start Documentation CI and Lint by hand on a branch or a tag of this repository. [Checks on a pull request]({{ docs_repo_url }}/AGENTS.md#checks-on-a-pull-request) in this site's AGENTS.md lists the four cases and says what to do in each.

Run the same checks on your PC, in the repository's root, with Python 3.12 as in CI:

```console
python3.12 -m venv venv && venv/bin/pip install -r requirements.txt codespell
venv/bin/python -m unittest discover -s tests -v
venv/bin/mkdocs build --strict
venv/bin/codespell
```

While you write, `venv/bin/mkdocs serve` shows the site at `http://127.0.0.1:8000` and rebuilds it when a file changes. The command runs until you stop it.
