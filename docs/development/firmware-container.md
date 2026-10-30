# Firmware Container

The Teensy firmware in [`diffbot_firmware`]({{ firmware_src_url }}/diffbot_firmware) builds with [PlatformIO](https://platformio.org/), which isn't in the [dev container](dev-container.md). So the firmware has a container of its own, [`.docker/firmware`]({{ firmware_src_url }}/.docker/firmware): Debian 12 with Python 3.12, PlatformIO 6.2.0 and a compiler, without ROS. A script runs PlatformIO in it as your user, and CI builds the firmware the same way. You need only Docker.

??? info "Why not in the dev container"
    The dev container can't run the current Teensy tools: its Ubuntu 20.04 has glibc 2.31, the C library, and the PlatformIO Teensy tools need 2.34 or later. In our test, the tool `teensy_size` stopped there with a missing `GLIBC_2.34`. The firmware container's Debian 12 has glibc 2.36.

    A VS Code dev container for the firmware is planned in [diffbot#149](https://github.com/ros-mobile-robots/diffbot/issues/149). Until then, build from a terminal on your PC, as below.

## Use it

Open a terminal on your PC and go to the root folder of your diffbot clone. On Windows, use the Ubuntu of WSL 2, not the dev container. Then build the firmware for your robot's Teensy:

=== "Remo: Teensy 4.0"

    ```console
    .docker/firmware/run.sh pio run -e teensy40
    ```

=== "DiffBot: Teensy 3.2"

    ```console
    .docker/firmware/run.sh pio run -e teensy31
    ```

[`.docker/firmware/run.sh`]({{ firmware_src_url }}/.docker/firmware/run.sh) is our script. It runs a command in the firmware container, here PlatformIO's command `pio`, and [the script](#the-script) below shows how. `pio run` builds the firmware. `-e` selects an environment in [`platformio.ini`]({{ firmware_src_url }}/diffbot_firmware/platformio.ini), and each environment builds for one board: `teensy40` for the Teensy 4.0, `teensy31` for the Teensy 3.1 and 3.2. Without `-e`, `pio run` builds `teensy40`. [Which Microcontroller?](../processing_units/teensy-mcu.md#which-microcontroller) lists the Teensy of each robot.

For your robot, one build is enough. If you change the firmware, build the other board too, because [CI](#in-ci) checks that the firmware builds for both.

The first build downloads the base image, PlatformIO and the Teensy toolchain, which needs the internet and took about a minute on our test PC. Later builds take seconds. The end of a Teensy 4.0 build looks like this:

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

PlatformIO writes each build into its standard build folder, `diffbot_firmware/.pio/build/<environment>/`. So the firmware is `diffbot_firmware/.pio/build/teensy40/firmware.hex` for Remo and `diffbot_firmware/.pio/build/teensy31/firmware.hex` for DiffBot. Git ignores `.pio`. The file belongs to your user, not to root, so you can copy or delete it without `sudo`.

**Flash it** with PJRC's [Teensy Loader](https://www.pjrc.com/teensy/loader.html), an app that you install on your PC. It isn't part of the container. It loads `firmware.hex` onto the Teensy over USB. On Linux, it needs PJRC's [udev rules](https://www.pjrc.com/teensy/loader_linux.html), a file that you copy to `/etc/udev/rules.d/`, so that it may write to the Teensy. Flashing from the container is planned in [diffbot#149](https://github.com/ros-mobile-robots/diffbot/issues/149). For that, the container needs access to the Teensy's USB port.

**Other commands** run the same way, for example `.docker/firmware/run.sh bash` for a shell in the container. Shell syntax such as `&&` needs a shell in the container, for example `.docker/firmware/run.sh bash -c 'pwd && id'`. Otherwise the shell on your PC reads it. Commands start in `diffbot_firmware`. Paths in them are paths in the container, where your clone is `/workspace`, so a folder on your PC outside your clone doesn't exist there.

**Another user and group ID:** the script uses yours (`id -u` and `id -g`) unless you set `LOCAL_UID` and `LOCAL_GID`, for example `LOCAL_UID=1234 LOCAL_GID=1234 .docker/firmware/run.sh pio run -e teensy40`. That ID needs write access to your clone. In the container, `id` then shows:

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

!!! warning "PlatformIO on your PC as well"
    Don't build with it and in the container at the same time. Both write to `diffbot_firmware/.pio`.

## How it works

The excerpts show the files at the diffbot commit that this page pins, with their line numbers. The link above each block opens the same lines on GitHub.

### The script

[`.docker/firmware/run.sh`]({{ firmware_src_url }}/.docker/firmware/run.sh) runs from anywhere, because it finds `compose.yaml` next to itself. Without a command, it says how to use it and stops before Docker starts, [lines 15–28]({{ firmware_src_url }}/.docker/firmware/run.sh#L15-L28):

```bash title="run.sh: without a command" linenums="15"
--8<-- "{{ firmware_src_raw }}/.docker/firmware/run.sh:15:28"
```

It writes the usage to stderr, the error output, and exits with 2, so that a CI job or another script that forgot the command fails. `--help` writes the same to the normal output and exits with 0.

Then it sets the user and group IDs, [lines 30–33]({{ firmware_src_url }}/.docker/firmware/run.sh#L30-L33):

```bash title="run.sh: the IDs" linenums="30"
--8<-- "{{ firmware_src_raw }}/.docker/firmware/run.sh:30:33"
```

`here` is the script's folder: `dirname` takes the folder from the script's path, and `cd … && pwd` turns it into a full path. `${LOCAL_UID:-$(id -u)}` keeps a `LOCAL_UID` that you set, and takes your user ID otherwise. The same goes for the group ID. `export` makes both environment variables, which the programs that the script starts can read. Docker Compose fills `${LOCAL_UID}` and `${LOCAL_GID}` in `compose.yaml` with them. `compose`, a variable like `here`, is a list with the start of every Compose command: `docker compose -f <folder>/compose.yaml`. The script adds the rest, for example `build firmware`.

Next, it builds the image, [lines 35–49]({{ firmware_src_url }}/.docker/firmware/run.sh#L35-L49):

```bash title="run.sh: the image" linenums="35"
--8<-- "{{ firmware_src_raw }}/.docker/firmware/run.sh:35:49"
```

`config --images` asks Compose for the image's name, which contains your user and group IDs. `docker image inspect` checks whether that image exists:

- **Not yet** (`!`): the build shows Docker's progress, because it downloads the base image and the Python packages for a while.
- **It exists** (`else`): the build usually takes seconds, because Docker reuses the cached result of each step when the Dockerfile and `requirements.txt` haven't changed. Its output goes into the variable `build_log`, not to the terminal. When the build fails, `status` gets the build's exit code, and the script prints the log to stderr and stops with that code.

Last, it runs your command in the container, [lines 51–57]({{ firmware_src_url }}/.docker/firmware/run.sh#L51-L57):

```bash title="run.sh: the run" linenums="51"
--8<-- "{{ firmware_src_raw }}/.docker/firmware/run.sh:51:57"
```

`tty_flag` starts as an empty list. `[ ! -t 0 ]` is true when the input isn't a terminal, and `[ ! -t 1 ]` when the output isn't, for example in CI or when the output goes into a file. Then the list gets `-T`, and the container starts without a terminal. Docker Compose before version 2.39.4 checks only the output, so the script checks both itself.

The last line runs your command:

- **`"${compose[@]}" run --rm`** starts a container for this one command and removes it afterwards.
- **`${tty_flag[@]+"${tty_flag[@]}"}`** inserts `-T` when the list has it, and nothing when it's empty. The shorter `"${tty_flag[@]}"` would stop the script for an empty list in Bash before 4.4, such as macOS's Bash 3.2, because `set -u` treats it as unset.
- **`firmware "$@"`:** `firmware` is the service in `compose.yaml`, and `"$@"` is your command with its arguments, for example `pio run -e teensy40`. Each argument reaches the container unchanged, because no shell reads it in between.
- **`exec`** replaces the script's Bash process with the `docker` command, so no Bash waits in between. A signal sent to the script, such as a stop, then reaches Docker. When your command in the container finishes, Docker Compose returns its exit code, 0 for success and another number for an error, so a failed build fails the CI step.

### The container

[`compose.yaml`]({{ firmware_src_url }}/.docker/firmware/compose.yaml) describes the image and the container, [lines 17–38]({{ firmware_src_url }}/.docker/firmware/compose.yaml#L17-L38):

```yaml title="compose.yaml" linenums="17" hl_lines="6 7 8 9"
--8<-- "{{ firmware_src_raw }}/.docker/firmware/compose.yaml:17:38"
```

- **`build`:** the image is built from the Dockerfile in this folder, with your user and group IDs as build arguments. `${LOCAL_UID:?…}` stops with that message when `LOCAL_UID` isn't set.
- **`image`:** the image's name carries your user and group IDs, so another user on the same PC gets an image of their own.
- **`user`:** the container runs with your user and group IDs, so the files it writes into the repository belong to your user.
- **`volumes`:** the whole repository is mounted at `/workspace`, and the container starts in `diffbot_firmware`. PlatformIO keeps its downloads in `/pio-core`: on your PC that's a Docker [volume](https://docs.docker.com/engine/storage/volumes/), a storage area that Docker keeps between runs, again with your user and group IDs in its name. With `PIO_CORE` set to a folder, that folder is mounted there instead, which [CI](#in-ci) uses.

### The image

The [Dockerfile]({{ firmware_src_url }}/.docker/firmware/Dockerfile) starts from Python on Debian 12 and installs a compiler, for programs that run on the PC such as unit tests, and PlatformIO in the version that [`requirements.txt`]({{ firmware_src_url }}/.docker/firmware/requirements.txt) pins, [lines 5–16]({{ firmware_src_url }}/.docker/firmware/Dockerfile#L5-L16):

```dockerfile title="Dockerfile: the tools" linenums="5"
--8<-- "{{ firmware_src_raw }}/.docker/firmware/Dockerfile:5:16"
```

[`requirements.txt`]({{ firmware_src_url }}/.docker/firmware/requirements.txt) holds PlatformIO's version, [line 1]({{ firmware_src_url }}/.docker/firmware/requirements.txt#L1-L1):

```text title="requirements.txt" linenums="1"
--8<-- "{{ firmware_src_raw }}/.docker/firmware/requirements.txt:1:1"
```

`==` installs exactly this version, so the builds in this container, on your PC and in CI, use the same PlatformIO. A newer version is a change of this line, which also starts a new cache in [CI](#in-ci).

The Teensy toolchain isn't in the image: PlatformIO downloads it into `/pio-core` on the first build. Then the image gets a user with your user and group IDs, [lines 18–35]({{ firmware_src_url }}/.docker/firmware/Dockerfile#L18-L35):

```dockerfile title="Dockerfile: your user" linenums="18"
--8<-- "{{ firmware_src_raw }}/.docker/firmware/Dockerfile:18:35"
```

!!! note "No default IDs"
    The build arguments `USER_UID` and `USER_GID` have no default value. A build without both stops with an error, instead of quietly creating the user with the ID 1000, which may not be yours.

### Your user ID, and the other containers

The ID travels in four steps: the script takes `id -u`, Compose passes it to the build as `USER_UID`, the Dockerfile creates the user `pio` with it, and the container runs as that ID. Files that the build writes into your clone, such as `firmware.hex`, then belong to your user, not to root or another ID.

The other containers solve the same problem differently:

- **The [dev container](dev-container-internals.md)** creates its user `ros` with 1000, the usual ID of the first user on Linux. VS Code changes that user to your user and group IDs when it creates the container ([`updateRemoteUserUID`](https://containers.dev/implementors/json_reference/#-general-devcontainerjson-properties-), on by default on Linux). The firmware container runs without VS Code, so it gets your user and group IDs at build time instead.
- **The [robot container](../processing_units/robot-container/index.md)** runs as `ros` with 1000 on the Raspberry Pi, the robot's first user. The planned published image ([diffbot#151](https://github.com/ros-mobile-robots/diffbot/pull/151)) is built once per architecture with the IDs 1000, not for each robot's user. [Setup](../processing_units/robot-container/setup.md#8-the-folder-for-logs) covers a robot whose first user has another ID.

### In CI

The [Firmware workflow]({{ firmware_src_url }}/.github/workflows/firmware.yml) builds each board through the same script, [lines 20–46]({{ firmware_src_url }}/.github/workflows/firmware.yml#L20-L46):

```yaml title="firmware.yml: the steps" linenums="20"
--8<-- "{{ firmware_src_raw }}/.github/workflows/firmware.yml:20:46"
```

Each CI job starts on a fresh machine, where a Docker volume would be empty, so PlatformIO would download the Teensy toolchain in every job. The steps avoid that:

- **Settings for the firmware container:** `PIO_CORE` gets a folder in `$RUNNER_TEMP`, a temporary folder that GitHub empties at the start and end of each job. Each step runs in a new shell, so a variable set with `export` would be gone in the next step. Instead, the step appends the line `PIO_CORE=<folder>` to a file whose path GitHub puts in `$GITHUB_ENV`. After the step, GitHub sets each `NAME=value` line of that file as an environment variable in the later steps ([environment files](https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-commands#setting-an-environment-variable)).
- **[`actions/cache`](https://github.com/actions/cache):** restores the `PIO_CORE` folder from an earlier run at the start of the job, and saves it when needed after a successful job. [The cache](#the-cache) below explains how.
- **Build the firmware:** runs the script as on your PC. It sets the user and group IDs to the runner's, and Compose mounts the `PIO_CORE` folder as the container's `/pio-core`.

#### The cache

`PIO_CORE` holds what PlatformIO downloads for the Teensy, about 510 MiB in our test: mostly the compiler for the Teensy's ARM processor (275 MiB) and the Arduino framework for the Teensy (100 MiB), plus PJRC's Teensy tools, PlatformIO's Teensy platform and the downloaded archives of the libraries in `platformio.ini`. Both boards need the same packages. The installed libraries aren't part of it, because PlatformIO installs them into the project, in `diffbot_firmware/.pio/libdeps`.

The cache is reused between workflow runs, not between the two boards. `matrix` runs the job twice at the same time, once with `matrix.env` set to `teensy31` and once with `teensy40`. The board is part of the cache key, so each board has a cache of its own, and the two jobs never try to save the same one. For Remo's board, the key looks like `pio-core-Linux-X64-teensy40-1a2b…`. The hash at the end comes from `requirements.txt` and `platformio.ini`, so it changes when PlatformIO's version or the configuration of the boards changes. For that board, the runs go like this, with `1a2b` and `5e6f` standing for whole keys with these hashes:

<div class="readable-table">
<table>
<thead><tr><th scope="col">Run</th><th scope="col">Restored at the start</th><th scope="col">Saved at the end</th></tr></thead>
<tbody>
<tr><td>The very first run</td><td>Nothing. PlatformIO downloads everything.</td><td><code>1a2b</code></td></tr>
<tr><td>A later run with the same key</td><td><code>1a2b</code>, the exact key</td><td>Nothing, because it exists</td></tr>
<tr><td>After a change of <code>requirements.txt</code> or <code>platformio.ini</code>, which makes the key <code>5e6f</code></td><td><code>1a2b</code>, through <code>restore-keys</code></td><td><code>5e6f</code></td></tr>
</tbody>
</table>
</div>

In the third run, no cache has the exact key. `restore-keys` then restores the newest cache whose key starts with `pio-core-Linux-X64-teensy40-`, the same board with an older hash. PlatformIO uses the packages that are already in the folder and downloads only the missing ones. A saved cache can't change, so the job saves the folder under the new key. A run on a pull request also finds the caches of its base branch, `noetic-devel`, so the first run on a new branch usually gets the exact key. GitHub's documentation explains [the matching](https://docs.github.com/en/actions/reference/workflows-and-actions/dependency-caching#cache-key-matching) and [which branches' caches a run can use](https://docs.github.com/en/actions/reference/workflows-and-actions/dependency-caching#restrictions-for-accessing-a-cache).

## Planned

[diffbot#149](https://github.com/ros-mobile-robots/diffbot/issues/149) tracks the next steps: the firmware's unit tests in this container and in CI ([diffbot#157](https://github.com/ros-mobile-robots/diffbot/issues/157)), flashing from the container, a VS Code dev container for the firmware, and pinning the Teensy platform and libraries.
