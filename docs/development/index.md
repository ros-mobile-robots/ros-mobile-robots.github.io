# Contributing

This section explains how DiffBot and Remo are developed: where the code lives, how changes get in, and how the development environment and the automated checks work.

## Repositories

| Repository | Contents | Default branch |
|:-----------|:---------|:---------------|
| [diffbot](https://github.com/ros-mobile-robots/diffbot) | ROS packages, Teensy firmware, dev container, CI | `noetic-devel` |
| [ros-mobile-robots.github.io](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io) | This documentation site | `main` |
| [remo_description](https://github.com/ros-mobile-robots/remo_description) | Remo's URDF description. The STL meshes come with the [Remo files on Gumroad](https://fjp.gumroad.com/l/GnMpU) | `main` |

## Roadmap

The plan is public in the [DiffBot & Remo roadmap](https://github.com/orgs/ros-mobile-robots/projects/3) project. Every open issue there has a phase:

1. **Noetic upkeep:** ROS 1 Noetic stays supported on the `noetic-devel` branch, including Gazebo Classic.
2. **Docker and CI:** one container setup for development and CI.
3. **ROS 2:** the port, in phases for the model and simulation, ros2_control, and the Teensy firmware. It will live in the same repository, on a new branch.
4. **IMU, filter and navigation:** IMU, Kalman filter (robot_localization) and navigation, for ROS 1 and ROS 2.

Issues labelled `good first issue` or `help wanted` are good places to start.

## How changes get in

1. **Issue:** describe the bug or feature, or pick an existing issue and comment that you're on it.
2. **Branch:** name it after the kind of change, for example `feat/imu-driver`, `fix/encoder-ticks`, `docs/dev-container` or `chore/ci-actions`.
3. **Pull request:** against `noetic-devel` in diffbot, or `main` in this repository. Explain what changes and how you tested it.
4. **Checks and review:** the [CI checks](ci.md) must pass, and every PR gets a review.
5. **Merge:** PRs are squash-merged. The PR title and description become the commit message, so they should read like one. The branch is deleted after the merge.

A feature isn't finished until it's documented: code changes in diffbot come with a matching page or update on this site.

## Best practices

The code follows the [ROS best practices](http://wiki.ros.org/Tutorials/Best%20Practices) and the [ROS Quality](http://wiki.ros.org/Quality) guidelines where possible, including the [ROS Enhancement Proposals (REPs)](https://www.ros.org/reps/rep-0000.html). Pages on this site link to the REPs that apply.

Good starting points:

- [ROS developer's guide](http://wiki.ros.org/DevelopersGuide): common practices for components shared with the community, with the [naming conventions](http://wiki.ros.org/ROS/Patterns/Conventions#Naming_ROS_Resources) and the [C++](http://wiki.ros.org/CppStyleGuide) and [Python](http://wiki.ros.org/PyStyleGuide) style guides.
- [ROS best practices](https://github.com/ethz-asl/ros_best_practices/wiki) of the Autonomous Systems Lab at ETH Zurich.

How the code is built, tested and debugged is described in [Testing and CI](ci.md).

## Writing docs

This site is built with [MkDocs](https://www.mkdocs.org/) and the [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) theme. Its source is the [ros-mobile-robots.github.io](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io) repository: one Markdown file per page in `docs/`, and the navigation in `mkdocs.yml`. Pull requests from branches in this repository get a preview on the real site, see [Testing and CI](ci.md#this-documentation-site).

The packages have no generated API documentation (such as Doxygen) yet.

## Development environment

- **Development PC:** use the [dev container](dev-container.md) from Getting Started. It has ROS Noetic, Gazebo, RViz and all dependencies, on Linux and on Windows with WSL 2.
- **Robot:** the Raspberry Pi or Jetson Nano is set up natively, see [Packages Setup](../packages/packages-setup.md).
