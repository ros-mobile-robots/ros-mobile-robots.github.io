# Dev Container

The diffbot repository has a dev container for ROS 1 Noetic: a Docker image with Ubuntu 20.04, ROS Noetic, Gazebo 11, RViz and every dependency of the DiffBot packages. Every developer, and [CI](ci.md), gets the same environment, on Linux and on Windows with WSL 2.

It's the recommended setup for the development PC. The robot's single board computer is still set up natively, see [Packages Setup](../packages/packages-setup.md).

## Requirements

- **Docker:** Docker Engine on Linux or in WSL 2, or Docker Desktop. On Ubuntu, including WSL 2:

    ```console
    sudo apt install docker.io docker-compose-v2 docker-buildx
    sudo usermod -aG docker $USER
    ```

    Log out and in again, or on WSL 2 run `wsl --shutdown` in Windows PowerShell, so the new `docker` group applies.

- **A way to start the container:** [VS Code](https://code.visualstudio.com/) with the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers), the [Dev Container CLI](https://github.com/devcontainers/cli) (`npm install -g @devcontainers/cli`), or plain Docker.
- **GUI apps on a native Linux desktop:** `xauth` on the host (`sudo apt install xauth`). On Windows, WSLg needs nothing extra.

## Usage

Clone the repository on the host first:

```console
git clone https://github.com/ros-mobile-robots/diffbot.git
cd diffbot
```

=== "VS Code"

    Open the `diffbot` folder and choose **Reopen in Container** (or run **Dev Containers: Reopen in Container** from the command palette). The first start builds the image and the workspace, which takes a few minutes.

=== "Dev Container CLI"

    ```console
    devcontainer up --workspace-folder . --config .devcontainer/noetic/devcontainer.json
    devcontainer exec --workspace-folder . --config .devcontainer/noetic/devcontainer.json bash
    ```

=== "Plain Docker"

    The image's user has UID 1000, which is the usual UID of the first user on Linux. With another UID, use VS Code or the CLI, which adapt it.

    ```console
    docker build -f .devcontainer/noetic/Dockerfile -t diffbot:noetic .
    bash .devcontainer/noetic/host-x11.sh
    docker run -it --rm --net=host -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY \
      -e XAUTHORITY=/home/ros/catkin_ws/src/diffbot/.devcontainer/noetic/.x11/xauth \
      -v "$PWD":/home/ros/catkin_ws/src/diffbot diffbot:noetic \
      bash -c "bash src/diffbot/.devcontainer/noetic/setup.sh && bash"
    ```

Inside the container, the workspace is `~/catkin_ws`, already built and sourced. Your clone is mounted at `~/catkin_ws/src/diffbot`, so edits on the host show up in the container and the other way round. For example, start the simulation with Gazebo and RViz:

```console
roslaunch diffbot_control diffbot.launch
```

After changing code, rebuild with `catkin build` in `~/catkin_ws`.

## How it works

| File in diffbot | Purpose |
|:----------------|:--------|
| `.devcontainer/noetic/Dockerfile` | The image: ROS, Gazebo, tools and the packages' dependencies |
| `.devcontainer/noetic/devcontainer.json` | How the container runs: mounts, network, display, what runs when |
| `.devcontainer/noetic/setup.sh` | Runs once in a new container: fetches repositories, installs dependencies, builds the workspace |
| `.devcontainer/noetic/host-x11.sh` | Runs on the host before the container starts: prepares the display access |
| `.dockerignore` | Keeps `.git` and the display cookie out of the image build |
| `.github/workflows/devcontainer.yml` | CI builds the same container on every pull request |

### The image

The [Dockerfile]({{ diffbot_repo_url }}/.devcontainer/noetic/Dockerfile) starts from `osrf/ros:noetic-desktop-full`, the official ROS image with ROS Noetic, Gazebo 11 and RViz on Ubuntu 20.04. On top it installs catkin tools, vcstool and a few shell tools, and creates the user `ros` (UID 1000, sudo without password).

The packages' dependencies are installed with [rosdep](http://wiki.ros.org/rosdep) from their `package.xml` files. The Dockerfile has two stages for this:

1. The first stage copies the repository and keeps only the `package.xml` files.
2. The second stage copies just those files and runs `rosdep install` on them.

Docker reuses a build step as long as its inputs don't change. Because the rosdep step only sees the `package.xml` files, a change to the source code doesn't reinstall the dependencies: the image rebuilds in about 2 seconds. A change to a `package.xml` reinstalls them, which takes about 40 seconds.

!!! note "Why the old Dockerfile was replaced"
    The repository used to have a `Dockerfile` in its root. It stopped building because it added the ROS package source a second time, with ROS's old signing key, which expired in 2025 (`EXPKEYSIG F42ED6FBAB17C654`). The official ROS images already have the ROS package source set up with the current key, so the new Dockerfile doesn't add it again.

### Creating the container

When the container is created, the [devcontainer.json]({{ diffbot_repo_url }}/.devcontainer/noetic/devcontainer.json) settings apply in this order:

1. **On the host:** `host-x11.sh` prepares the display access (see below).
2. **Build and start:** the image is built, and the container starts with your clone mounted at `~/catkin_ws/src/diffbot`. VS Code and the CLI change the UID and GID of the user `ros` to yours, so files created in the container belong to you on the host.
3. **In the container:** [`setup.sh`]({{ diffbot_repo_url }}/.devcontainer/noetic/setup.sh) runs once. It imports `rplidar_ros` and `remo_description` with `vcs import` from `diffbot_dev.repos`, runs `rosdep install` again for anything added since the image was built, builds the workspace with `catkin build` and adds the workspace to `~/.bashrc`.

`remo_description` contains empty placeholder STL files. To see Remo's meshes in RViz and Gazebo, get the real files as described in its [README](https://github.com/ros-mobile-robots/remo_description#stl-mesh-files). DiffBot's own meshes are part of `diffbot_description`.

### Network

The container uses the host network (`--network=host`), so ROS nodes in the container are reachable at the host's IP address. Set up `ROS_MASTER_URI` and `ROS_IP` as described in [ROS Network Setup](../processing_units/ros-network-setup.md) to talk to the robot.

### GUI apps: X11 and WSLg

RViz, Gazebo and rqt open their windows on the host's display. The container gets the X11 socket directory `/tmp/.X11-unix` and the `DISPLAY` variable from the host.

On Windows, [WSLg](https://github.com/microsoft/wslg) provides the X server and accepts local connections without authentication.

A native Linux desktop (X11, or Wayland with Xwayland) only accepts clients that present the display's cookie (`MIT-MAGIC-COOKIE-1`). [`host-x11.sh`]({{ diffbot_repo_url }}/.devcontainer/noetic/host-x11.sh) copies that cookie for the container:

```bash
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$tmp_file" nmerge -
```

- **Wildcard address:** `xauth nlist` prints the cookie entries for the display. Their first four characters are the address family; `ffff` changes it to "wild", so the cookie matches any hostname, including the container's.
- **Where the cookie goes:** the script writes the cookie to `.devcontainer/noetic/.x11/xauth`, through a temporary file and a rename. Git and Docker ignore that folder.
- **How the container finds it:** the container reads the cookie through the workspace mount (`XAUTHORITY` points there). A refreshed cookie is therefore visible in a running container too.

This avoids `xhost +`, which would allow every local user and process to connect to your display.

## Updating

- **New ROS or system dependency:** add it to the package's `package.xml`. Then rebuild the container: **Dev Containers: Rebuild Container** in VS Code, or `devcontainer up --remove-existing-container` with the CLI.
- **New source dependency:** add the repository to `diffbot_dev.repos`, and to the robot's `.repos` file if the robot needs it too.
- **Tools in the image:** add them to the `apt-get install` list in the Dockerfile.

## Troubleshooting

| Problem | Fix |
|:--------|:----|
| `permission denied` on `/var/run/docker.sock` | Your user isn't in the `docker` group yet. Add it, then log out and in (WSL 2: `wsl --shutdown`). |
| `No protocol specified` or `cannot open display` on native Linux | Install `xauth` on the host and recreate the container. Check that `echo $DISPLAY` shows a display on the host. |
| Files in the clone belong to another user (plain Docker) | Your UID isn't 1000. Use VS Code or the CLI, which adapt the UID. |
| Gazebo is slow | The container renders without GPU acceleration, which isn't set up yet. |
