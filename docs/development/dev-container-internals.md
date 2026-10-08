# How the Dev Container Works

What the files in diffbot's `.devcontainer/noetic/` folder do, and how the image, the container, the network and the display access are set up. To install and use the dev container, see [Development Environment](dev-container.md).

| File in diffbot | Purpose |
|:----------------|:--------|
| [`.devcontainer/noetic/Dockerfile`]({{ diffbot_repo_url }}/.devcontainer/noetic/Dockerfile) | The image: ROS, Gazebo, tools and the packages' dependencies |
| [`.devcontainer/noetic/devcontainer.json`]({{ diffbot_repo_url }}/.devcontainer/noetic/devcontainer.json) | How the container runs: mounts, network, display, what runs when |
| [`.devcontainer/noetic/setup.sh`]({{ diffbot_repo_url }}/.devcontainer/noetic/setup.sh) | Runs once in a new container: fetches repositories, installs dependencies, builds the workspace |
| [`.devcontainer/noetic/host-x11.sh`]({{ diffbot_repo_url }}/.devcontainer/noetic/host-x11.sh) | Runs on the host before the container starts: prepares the display access |
| [`.dockerignore`]({{ diffbot_repo_url }}/.dockerignore) | Keeps `.git` and the X11 cookie out of the image build |
| [`.github/workflows/devcontainer.yml`]({{ diffbot_repo_url }}/.github/workflows/devcontainer.yml) | CI builds the same container on every pull request |

## The image

The [Dockerfile]({{ diffbot_repo_url }}/.devcontainer/noetic/Dockerfile) starts from [`osrf/ros:noetic-desktop-full`](https://hub.docker.com/r/osrf/ros), the official ROS image with ROS Noetic, Gazebo 11 and RViz on Ubuntu 20.04. On top it installs [catkin tools](https://catkin-tools.readthedocs.io/), [vcstool](https://github.com/dirk-thomas/vcstool) and a few shell tools, and creates the user `ros` (UID 1000, sudo without password).

The packages' dependencies are installed with [rosdep](http://wiki.ros.org/rosdep) from their `package.xml` files. The Dockerfile has two [stages](https://docs.docker.com/build/building/multi-stage/) for this:

1. The first stage copies the repository and keeps only the `package.xml` files.
2. The second stage copies just those files and runs `rosdep install` on them.

Docker reuses a build step as long as its inputs don't change ([build cache](https://docs.docker.com/build/cache/)). Because the rosdep step only sees the `package.xml` files, a change to the source code doesn't reinstall the dependencies: the image rebuilds in about 2 seconds. A change to a `package.xml` reinstalls them, which takes about 40 seconds.

!!! note "Why the old Dockerfile was replaced"
    The repository used to have a `Dockerfile` in its root. It stopped building because it added the ROS package source a second time, with ROS's old signing key, which expired in 2025 (`EXPKEYSIG F42ED6FBAB17C654`; see the [ROS signing key migration guide](https://discourse.ros.org/t/ros-signing-key-migration-guide/43937)). The official ROS images already have the ROS package source set up with the current key, so the new Dockerfile doesn't add it again.

## Creating the container

When the container is created, the [devcontainer.json]({{ diffbot_repo_url }}/.devcontainer/noetic/devcontainer.json) settings apply in this order (see the dev container [lifecycle scripts](https://containers.dev/implementors/json_reference/#lifecycle-scripts)):

1. **On the host:** [`host-x11.sh`]({{ diffbot_repo_url }}/.devcontainer/noetic/host-x11.sh) prepares the display access (see [GUI apps](#gui-apps-x11-and-wslg) below).
2. **Build and start:** the image is built, and the container starts with your clone mounted at `~/catkin_ws/src/diffbot`. VS Code and the CLI change the UID and GID of the user `ros` to yours, so files created in the container belong to you on the host ([`updateRemoteUserUID`](https://containers.dev/implementors/json_reference/)).
3. **In the container:** [`setup.sh`]({{ diffbot_repo_url }}/.devcontainer/noetic/setup.sh) runs once. It imports `rplidar_ros` and `remo_description` with [`vcs import`](https://github.com/dirk-thomas/vcstool) from [`diffbot_dev.repos`]({{ diffbot_repo_url }}/diffbot_dev.repos), runs `rosdep install` again for anything added since the image was built, builds the workspace with `catkin build` and adds the workspace to `~/.bashrc`.

`remo_description` contains empty placeholder STL files. To see Remo's meshes in RViz and Gazebo, get the real files as described in its [README](https://github.com/ros-mobile-robots/remo_description#stl-mesh-files). DiffBot's own meshes are part of `diffbot_description`.

## Network

The container uses the host's network ([`--network=host`](https://docs.docker.com/engine/network/drivers/host/)): it has no network of its own, and ROS nodes in the container are reachable at the host's IP address. For the real robot, set [`ROS_MASTER_URI` and `ROS_IP`](http://wiki.ros.org/ROS/EnvironmentVariables) as described in [ROS Network Setup](../processing_units/ros-network-setup.md).

ROS 1 nodes connect to each other directly, in both directions: the machines need "full bi-directional connectivity, on all ports" ([ROS NetworkSetup](http://wiki.ros.org/ROS/NetworkSetup)). So the robot must be able to reach your PC too:

- **Linux PC:** the host's IP address is the PC's address on your network, so this works as usual.
- **Windows with WSL 2:** by default, WSL 2 sits behind its own network translation (NAT) with a private IP address, and devices on your network can't connect to it. Switch on [mirrored networking](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking) (Windows 11 22H2 or later): add `networkingMode=mirrored` under `[wsl2]` in `%UserProfile%\.wslconfig` ([WSL settings](https://learn.microsoft.com/en-us/windows/wsl/wsl-config)) and run `wsl --shutdown`. WSL then shares Windows' network addresses, and the robot can reach it. Windows' Hyper-V firewall may also need to allow incoming connections, as described on that page. This setup isn't tested with the robot yet.

## GUI apps: X11 and WSLg

The container has no screen of its own; it's "headless". Linux GUI apps don't need one: a program like RViz is an X client: it connects to an X server and sends it what to draw, and the X server shows the window ([X Window System](https://www.x.org/releases/current/doc/man/man7/X.7.xhtml)). The container borrows the host's X server. It gets the socket folder `/tmp/.X11-unix`, through which clients reach the X server, and the [`DISPLAY`](https://www.x.org/releases/current/doc/man/man7/X.7.xhtml#heading5) variable, which says which display to use. So RViz runs in the container, but its window opens on your desktop like any other.

On Windows, [WSLg](https://github.com/microsoft/wslg) provides the X server and accepts local clients without a cookie, so nothing else is needed.

??? info "How WSLg shows Linux windows on Windows"
    An X11 program like RViz connects through the X socket to XWayland, WSLg's X server. Weston, a Wayland compositor, collects the windows and sends them over a remote desktop (RDP) connection to Windows, which shows each one as a normal window. All of this runs in a small "system distro" next to your Ubuntu.

    <figure>
      <img src="../images/wslg-architecture.png" alt="WSLg architecture: X11 and Wayland apps in the user distro connect to XWayland and Weston in the WSLg system distro, which sends windows to the Windows host over RDP">
      <figcaption>Diagram: <a href="https://github.com/microsoft/wslg">WSLg</a>, Microsoft, <a href="https://github.com/microsoft/wslg/blob/main/LICENSE">MIT License</a></figcaption>
    </figure>

A native Linux desktop (X11, or Wayland with Xwayland) only accepts clients that present the display's cookie (`MIT-MAGIC-COOKIE-1`, see [Requirements](dev-container.md#requirements)). [`host-x11.sh`]({{ diffbot_repo_url }}/.devcontainer/noetic/host-x11.sh) copies that cookie for the container, the method from the [ROS Docker GUI tutorial](http://wiki.ros.org/docker/Tutorials/GUI):

```bash
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$tmp_file" nmerge -
```

- **Wildcard address:** `xauth nlist` prints the cookie entries for the display. Their first four characters are the address family; `ffff` changes it to "wild", so the cookie matches any hostname, including the container's.
- **Where the cookie goes:** the script writes the cookie to `.devcontainer/noetic/.x11/xauth`, through a temporary file and a rename. Git and Docker ignore that folder.
- **How the container finds it:** the container reads the cookie through the workspace mount (`XAUTHORITY` points there). A refreshed cookie is therefore visible in a running container too.

This avoids [`xhost +`](https://www.x.org/releases/current/doc/man/man1/xhost.1.xhtml), which switches off the access control, so every local user and process could connect to your display. The ROS tutorial also calls that "not the safest way".
