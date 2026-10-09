# Set Up Your PC

Your PC, the development PC, runs the simulation and the tools for the real robot in a [dev container](../development/dev-container.md). This page prepares it once: the operating system, Docker, a tool to start the container, and the display access for windows like RViz and Gazebo. After that, install Git as described in [Git and GitHub](git-and-github.md), and start the container as described in [Use the Dev Container](../development/dev-container.md).

The commands on this page run on the *host*: your Linux, or on Windows the Ubuntu in WSL 2. [How the pieces fit together](../development/dev-container.md#how-the-pieces-fit-together) explains the terms host, container, X server and X clients.

## Operating system

=== "Linux"

    A Linux PC with a desktop, for example [Ubuntu](https://ubuntu.com/download/desktop) 24.04 LTS. Docker Engine supports Ubuntu 22.04, 24.04 and 26.04 ([OS requirements](https://docs.docker.com/engine/install/ubuntu/#os-requirements)). Other distributions with Docker Engine should work, but aren't tested.

    The PC doesn't need ROS: ROS Noetic runs in the container, on Ubuntu 20.04.

=== "Windows (WSL 2)"

    [WSL 2](https://learn.microsoft.com/en-us/windows/wsl/) (Windows Subsystem for Linux) runs an Ubuntu inside Windows. To install it, open PowerShell as administrator, run the following, and restart Windows ([Install WSL](https://learn.microsoft.com/en-us/windows/wsl/install)):

    ```powershell
    wsl --install
    ```

    This installs WSL and Ubuntu. If WSL is already installed, update it with `wsl --update`. For windows like RViz and Gazebo, WSL needs Windows 11 or Windows 10 build 19044 or later (see below). The setup was tested with Ubuntu 24.04 in WSL 2.

    From here on, run the commands in a terminal of that Ubuntu, not in PowerShell.

## Docker

The dev container runs on [Docker Engine](https://docs.docker.com/engine/), the part of Docker that builds images and runs containers. There are two ways to get it: install Docker Engine directly in your Linux (on Windows, in the Ubuntu in WSL 2), or install Docker Desktop, an app that brings its own Docker Engine. Use one of them, not both. Docker Engine is the tested setup.

=== "Docker Engine (tested)"

    Install Docker Engine as described in Docker's guide [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/), with Docker's apt repository. This works the same in the Ubuntu in WSL 2.

    Ubuntu's own packages work too, and this setup was tested with them (Docker 29 on Ubuntu 24.04). Docker calls them unofficial, and they can be older than Docker's releases:

    ```console
    sudo apt install docker.io docker-buildx
    ```

    Either way, then follow Docker's [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/), so you can use Docker without `sudo`:

    ```console
    sudo usermod -aG docker $USER
    ```

    Log out and in again so the new `docker` group applies. On WSL 2, run `wsl --shutdown` in Windows PowerShell and open Ubuntu again. Docker runs as a systemd service, which WSL turns on by default for current Ubuntu versions ([systemd in WSL](https://learn.microsoft.com/en-us/windows/wsl/systemd)).

=== "Docker Desktop (untested)"

    [Docker Desktop](https://docs.docker.com/desktop/) is an app for Windows, macOS and Linux that runs Docker Engine in its own virtual machine. On Windows, that's a separate WSL distribution called `docker-desktop`, and its [WSL integration](https://docs.docker.com/desktop/features/wsl/) makes the `docker` command available in your Ubuntu. Uninstall Docker Engine from your Ubuntu first, because the two conflict.

    Two things are different from Docker Engine:

    - The container's "host" is Docker Desktop's virtual machine, not your Ubuntu. Its host network, which this setup uses, is an opt-in feature from version 4.34 (Settings → Resources → Network → **Enable host networking**) and only carries TCP and UDP ([Docker docs](https://docs.docker.com/engine/network/drivers/host/#docker-desktop)).
    - Docker Desktop is free for personal use, education, non-commercial open source projects and small businesses. Larger companies need a paid subscription ([Docker Desktop license](https://docs.docker.com/subscription-billing/desktop-license/)).

    Docker Desktop isn't tested with this setup, and especially not with the robot.

Check that Docker works, without `sudo`:

```console
docker run hello-world
```

It downloads a small test image and prints a message from the container.

## A way to start the container

[VS Code](https://code.visualstudio.com/) with the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) is the easiest. On Windows, VS Code also needs the [WSL extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl), so it can reach Docker in the Ubuntu. The [Dev Container CLI](https://github.com/devcontainers/cli) works from a terminal (`npm install -g @devcontainers/cli`, which needs Node.js). Plain Docker works too. All three are shown under [Usage](../development/dev-container.md#usage).

## For windows like RViz and Gazebo

RViz, Gazebo and other GUI programs run inside the container, which has no screen of its own. They are X clients: they show their windows on your desktop through your host's X server (see [How the pieces fit together](../development/dev-container.md#how-the-pieces-fit-together)), and the X server has to let them in. This is set up once, depending on your host. It's only needed for the container: with ROS installed directly on your PC, its windows open like those of any other program.

=== "Linux"

    Install `xauth` on the host:

    ```console
    sudo apt install xauth
    ```

    With the usual cookie-based setup, the X server only accepts clients that present the display's *cookie*. In X11, a cookie is a secret: a random 128-bit value that your desktop creates when you log in. Clients without it, for example other users' programs, can't draw on your screen or read it. Any client that has the cookie can connect, which is how the container gets access ([X security](https://www.x.org/releases/current/doc/man/man7/Xsecurity.7.xhtml)). It has nothing to do with web cookies. [`xauth`](https://www.x.org/releases/current/doc/man/man1/xauth.1.xhtml) is the standard tool for these cookies, and the setup uses it to hand yours to the container (see [GUI apps](../development/dev-container-internals.md#gui-apps-x11-and-wslg)).

=== "Windows (WSL 2)"

    Nothing to install: WSLg shows the windows on the Windows desktop. It's part of WSL 2 on Windows 11 and on Windows 10 build 19044 or later (see the [prerequisites](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)).

## Real robot from Windows (optional)

Only needed if your PC runs Windows and you want to connect to the real robot. By default, WSL 2 sits behind its own network translation (NAT), so the robot can't connect to your PC, and ROS 1 needs connections in both directions. WSL's *mirrored networking* removes that barrier; it needs Windows 11 22H2 or later. Why, how to set it up and how to test it: [Work machine on Windows (WSL 2)](../processing_units/ros-network-setup.md#work-machine-on-windows-wsl-2).

You don't need it on Linux, where the container uses the PC's own network address. You also don't need it for simulation, because then all ROS nodes run in the container on your PC, and no other device has to connect to it.

!!! warning "Not tested with the robot yet"
    Mirrored networking is Microsoft's fix for exactly this problem, but nobody has tested this setup with DiffBot or Remo yet.
