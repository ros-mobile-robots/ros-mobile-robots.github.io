# Firmware Container

The Teensy firmware in [`diffbot_firmware`]({{ firmware_src_url }}/diffbot_firmware) builds with [PlatformIO](https://platformio.org/), which isn't in the [dev container](dev-container.md). So the firmware has a container of its own, [`.docker/firmware`]({{ firmware_src_url }}/.docker/firmware): Debian 12 with Python 3.12, PlatformIO 6.2.0 and a compiler, without ROS. A script runs PlatformIO in it as your user, and CI builds the firmware the same way. You need only Docker.

??? info "Why not in the dev container"
    The dev container can't run the current Teensy tools: its Ubuntu 20.04 has glibc 2.31, the C library, and the PlatformIO Teensy tools need 2.34 or later. In our test, the tool `teensy_size` stopped there with a missing `GLIBC_2.34`. The firmware container's Debian 12 has glibc 2.36.

    A VS Code dev container for the firmware, with this container's image, is planned in [diffbot#149](https://github.com/ros-mobile-robots/diffbot/issues/149). Until then, build from a terminal on your PC, as below.

## Use it

In a terminal on your PC, in your diffbot clone's root folder. On Windows, that's the Ubuntu of WSL 2, not the dev container:

```console
.docker/firmware/pio run -e teensy40
.docker/firmware/pio run -e teensy31
```

`pio` is PlatformIO's command, and `-e` selects one of the boards in [`platformio.ini`]({{ firmware_src_url }}/diffbot_firmware/platformio.ini): `teensy40` builds for the Teensy 4.0, `teensy31` for the Teensy 3.1/3.2. Without `-e`, `pio run` builds `teensy40`. The first build downloads the base image, PlatformIO and the Teensy toolchain, which needs the internet and took about a minute on our test PC. Later builds take seconds. The end of a build looks like this:

```console
Processing teensy40 (platform: teensy; board: teensy40; framework: arduino)
--------------------------------------------------------------------------------
Verbose mode can be enabled via `-v, --verbose` option
CONFIGURATION: https://docs.platformio.org/page/boards/teensy/teensy40.html
PLATFORM: Teensy (6.0.0) > Teensy 4.0
…
Checking size .pio/build/teensy40/firmware.elf
Advanced Memory Usage is available via "PlatformIO Home > Project Inspect"
teensy_size: Memory Usage on Teensy 4.0:
teensy_size:   FLASH: code:54832, data:10424, headers:8468   free for files:1957892
teensy_size:    RAM1: variables:13536, code:52712, padding:12824   free for local variables:445216
teensy_size:    RAM2: variables:12416  free for malloc/new:511872
Building .pio/build/teensy40/firmware.hex
========================= [SUCCESS] Took 1.50 seconds =========================
Environment    Status    Duration
-------------  --------  ------------
teensy40       SUCCESS   00:00:01.501
========================= 1 succeeded in 00:00:01.501 =========================
```

The firmware file is then `diffbot_firmware/.pio/build/<board>/firmware.hex`, and it belongs to you. PJRC's [Teensy Loader](https://www.pjrc.com/teensy/loader.html) flashes it onto the Teensy. Don't build in the container and on the PC at the same time, because both write to `diffbot_firmware/.pio`.

**Another user and group ID:** the script uses yours (`id -u` and `id -g`) unless you set `LOCAL_UID` and `LOCAL_GID`, for example `LOCAL_UID=1234 LOCAL_GID=1234 .docker/firmware/pio run -e teensy40`. That ID needs write access to your clone. In the container, `id` then shows:

```console
uid=1234(pio) gid=1234(pio) groups=1234(pio)
```

??? info "Without the script, or without the container"
    **With Docker Compose directly,** export both IDs first, because [`compose.yaml`]({{ firmware_src_url }}/.docker/firmware/compose.yaml) requires them:

    ```console
    export LOCAL_UID=$(id -u) LOCAL_GID=$(id -g)
    docker compose -f .docker/firmware/compose.yaml build firmware
    docker compose -f .docker/firmware/compose.yaml run --rm firmware pio run -e teensy40
    ```

    **With PlatformIO on your PC,** outside the dev container, in a virtual environment. These steps were tested on Ubuntu 24.04 (WSL 2) with Python 3.12 and PlatformIO 6.2:

    ```console
    sudo apt install python3-venv
    python3 -m venv ~/.venvs/platformio
    ~/.venvs/platformio/bin/pip install platformio
    cd diffbot_firmware
    ~/.venvs/platformio/bin/pio run -e teensy40
    ```

## How it works

The excerpts show the files at the diffbot commit that this page pins, with their line numbers. The link above each block opens the same lines on GitHub.

### The script

[`.docker/firmware/pio`]({{ firmware_src_url }}/.docker/firmware/pio) runs from anywhere, because it finds `compose.yaml` next to itself. It first sets the IDs, [lines 13–16]({{ firmware_src_url }}/.docker/firmware/pio#L13-L16):

```bash title="pio: the IDs" linenums="13"
--8<-- "{{ firmware_src_raw }}/.docker/firmware/pio:13:16"
```

`${LOCAL_UID:-$(id -u)}` keeps a `LOCAL_UID` that you set, and takes your user ID otherwise. `export` passes both to Docker Compose. `compose` holds the start of every Compose command, with the path to `compose.yaml`.

Then it builds the image, [lines 18–31]({{ firmware_src_url }}/.docker/firmware/pio#L18-L31):

```bash title="pio: the image" linenums="18"
--8<-- "{{ firmware_src_raw }}/.docker/firmware/pio:18:31"
```

`config --images` asks Compose for the image's name, which contains your IDs. When that image doesn't exist yet, the build shows Docker's progress, because it downloads for a while. Otherwise the build takes seconds when the Dockerfile and `requirements.txt` haven't changed, because Docker reuses its cached layers. The script keeps the build's output and prints it only when the build fails.

Last, it runs PlatformIO in the container, [lines 33–38]({{ firmware_src_url }}/.docker/firmware/pio#L33-L38):

```bash title="pio: the run" linenums="33"
--8<-- "{{ firmware_src_raw }}/.docker/firmware/pio:33:38"
```

`run --rm` starts a container for this one command and removes it afterwards. `pio "$@"` passes your arguments on, for example `run -e teensy40`. Without a terminal, in CI for example, `-T` starts the container without one. `exec` replaces the script with Docker, so the build's exit code is the script's.

### The container

[`compose.yaml`]({{ firmware_src_url }}/.docker/firmware/compose.yaml) describes the image and the container, [lines 17–38]({{ firmware_src_url }}/.docker/firmware/compose.yaml#L17-L38):

```yaml title="compose.yaml" linenums="17" hl_lines="6 7 8 9"
--8<-- "{{ firmware_src_raw }}/.docker/firmware/compose.yaml:17:38"
```

- **`build`:** the image is built from the Dockerfile in this folder, with your IDs as build arguments. `${LOCAL_UID:?…}` stops with that message when `LOCAL_UID` isn't set.
- **`image`:** the image's name carries your IDs, so another user on the same PC gets an image of their own.
- **`user`:** the container runs with your IDs, so the files it writes into the repository belong to you.
- **`volumes`:** the whole repository is mounted at `/workspace`, and the container starts in `diffbot_firmware`. PlatformIO keeps its downloads in `/pio-core`: on your PC that's a Docker [volume](https://docs.docker.com/engine/storage/volumes/), a storage area that Docker keeps between runs, again with your IDs in its name. With `PIO_CORE` set to a folder, that folder is mounted there instead, which [CI](#in-ci) uses.

### The image

The [Dockerfile]({{ firmware_src_url }}/.docker/firmware/Dockerfile) starts from Python on Debian 12 and installs a compiler, for programs that run on the PC such as unit tests, and PlatformIO in the version that [`requirements.txt`]({{ firmware_src_url }}/.docker/firmware/requirements.txt) pins, [lines 5–16]({{ firmware_src_url }}/.docker/firmware/Dockerfile#L5-L16):

```dockerfile title="Dockerfile: the tools" linenums="5"
--8<-- "{{ firmware_src_raw }}/.docker/firmware/Dockerfile:5:16"
```

The Teensy toolchain isn't in the image: PlatformIO downloads it into `/pio-core` on the first build. Then the image gets a user with your IDs, [lines 18–35]({{ firmware_src_url }}/.docker/firmware/Dockerfile#L18-L35):

```dockerfile title="Dockerfile: your user" linenums="18"
--8<-- "{{ firmware_src_raw }}/.docker/firmware/Dockerfile:18:35"
```

`USER_UID` and `USER_GID` have no default. A build without both stops instead of silently using 1000.

### Your user ID, and the other containers

The ID travels in four steps: the script takes `id -u`, Compose passes it to the build as `USER_UID`, the Dockerfile creates the user `pio` with it, and the container runs as that ID. Files that the build writes into your clone, such as `firmware.hex`, then belong to you, not to root or another ID.

The other containers solve the same problem differently:

- **The [dev container](dev-container-internals.md)** creates its user `ros` with 1000, the usual ID of the first user on Linux. VS Code changes that user to your IDs when it creates the container ([`updateRemoteUserUID`](https://containers.dev/implementors/json_reference/#-general-devcontainerjson-properties-), on by default on Linux). The firmware container runs without VS Code, so it gets your IDs at build time instead.
- **The [robot container](../processing_units/robot-container/index.md)** runs as `ros` with 1000 on the Raspberry Pi, the robot's first user. The planned published image ([diffbot#151](https://github.com/ros-mobile-robots/diffbot/pull/151)) is built once per architecture with the IDs 1000, not for each robot's user. [Setup](../processing_units/robot-container/setup.md#8-the-folder-for-logs) covers a robot whose first user has another ID.

### In CI

The [Firmware workflow]({{ firmware_src_url }}/.github/workflows/firmware.yml) builds each board through the same script, [lines 20–44]({{ firmware_src_url }}/.github/workflows/firmware.yml#L20-L44):

```yaml title="firmware.yml: the steps" linenums="20"
--8<-- "{{ firmware_src_raw }}/.github/workflows/firmware.yml:20:44"
```

Each CI job starts on a fresh machine, where a Docker volume would be empty. So the first step sets `PIO_CORE` to a folder in `$RUNNER_TEMP`, the runner's temporary folder, and a line `NAME=value` in the file that `$GITHUB_ENV` names makes it an environment variable for the later steps ([environment files](https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-commands#setting-an-environment-variable)). [`actions/cache`](https://github.com/actions/cache) restores a matching cache from an earlier run, if there is one, and saves a new one after a successful job when no cache had the exact key. The key holds the board, the runner's OS and architecture, and a hash of `requirements.txt` and `platformio.ini`. The script then sets the IDs to the runner's user, as on your PC.

## Planned

[diffbot#149](https://github.com/ros-mobile-robots/diffbot/issues/149) tracks the next steps: the firmware's unit tests in this container and in CI ([diffbot#157](https://github.com/ros-mobile-robots/diffbot/issues/157)), flashing from the container, a VS Code dev container for the firmware with the same image, and pinning the Teensy platform and libraries.
