# Versions and Your Own Build

This page is about the robot's Docker image: which version runs, how to update the image or stay on one version, what a test report notes, and how to build the image yourself. You don't need this page to start the robot. [Start, Check and Stop](usage.md) covers that.

The commands run on the Raspberry Pi, in the folder `~/diffbot/.docker/robot`, unless a step names your PC.

## Versions

A version is one build of the robot's Docker image. The setting `ROBOT_IMAGE` in `.env` chooses the version that runs ([All settings](setup.md#all-settings)).

### The newest version

Without `ROBOT_IMAGE`, the robot runs `noetic`. That's the newest Docker image that CI published from the `noetic-devel` branch. To update, on the Raspberry Pi:

```console
git pull
docker compose pull --policy always
docker compose up -d --no-build
```

1. `git pull` updates your clone of the repository, with `compose.yaml` and the udev rules.
2. `docker compose pull --policy always` asks the registry for `noetic` again, and downloads the image if there's a newer one.
3. `docker compose up -d --no-build` recreates the containers with the downloaded image.

### One fixed version

Each published Docker image also has a name with the full commit it was built from: `noetic-<commit>`. This name never moves to another image. The [package's page](https://github.com/orgs/ros-mobile-robots/packages/container/package/diffbot-robot) lists the names. To stay on one version, or to go back to an earlier one, set that version in `.env`:

```bash title=".docker/robot/.env"
ROBOT_IMAGE=ghcr.io/ros-mobile-robots/diffbot-robot:noetic-<commit>
```

Then download that version and restart the containers. On the Raspberry Pi:

```console
docker compose pull --policy always
docker compose up -d --no-build
```

To go back to the newest version, remove the line from `.env` and run both commands again. Removing the line alone changes nothing that runs.

??? info "Even more exact: the digest"
    A digest is a checksum of a Docker image's content: `sha256:` followed by 64 characters. If one byte of the image changes, the digest changes. So a digest names exactly one content, whatever names point to it ([Docker's reference on pulling by digest](https://docs.docker.com/reference/cli/docker/image/pull/#pull-an-image-by-digest-immutable-identifier)).

    `noetic-<commit>` is enough to stay on one version, because a published version is never replaced ([How it's tested](image.md#how-its-tested)). The digest is for a record that has to be exact. The package's page shows the digest of each version. In `.env`, `@` and the digest replace `:` and the name:

    ```bash title=".docker/robot/.env"
    ROBOT_IMAGE=ghcr.io/ros-mobile-robots/diffbot-robot@sha256:<digest>
    ```

    Use the digest that the package's page shows for `noetic-<commit>` itself, not the digest of one architecture's image. On the Raspberry Pi, `docker images --digests ghcr.io/ros-mobile-robots/diffbot-robot` lists the downloaded versions with their digests, in the column `DIGEST`.

### What a version covers

A version covers the Docker image, and nothing else. Three more parts decide what the robot does, and each has a state of its own:

| Part | Where the part comes from | How to see the part's state |
|:--|:--|:--|
| **The Docker image:** ROS Noetic, the robot's compiled packages and the entrypoint script | The registry, in the version that `ROBOT_IMAGE` names | The first command of [the test report](#what-a-test-report-needs) prints the image's commit |
| **Your clone, `~/diffbot`:** `compose.yaml` and the udev rules | `git clone`, updated by `git pull` | `git -C ~/diffbot log -1` |
| **`.env`:** your settings | You. Nothing else changes `.env` | `cat ~/diffbot/.docker/robot/.env` |
| **The Teensy's firmware:** the program that drives the motors and reads the encoders | Your last flash, from the PC | The Teensy doesn't report the firmware's commit. Note the commit when you flash |

udev uses the copy of the rules that [step 7](setup.md#7-stable-device-names) of the setup installed, not the file in your clone. After a `git pull` that changed the rules, install them again.

### What a test report needs

A test report, for example for the robot test in [diffbot#109](https://github.com/ros-mobile-robots/diffbot/issues/109), says what ran, so that someone else can repeat the test. That takes four notes, one for each part in the table above. The first three commands run on the Raspberry Pi, in `~/diffbot/.docker/robot`, while the robot runs.

1. **The Docker image's commit:** this commit says which code the containers run. CI writes the commit into each published image, as a label. This command prints the label of the running bringup container's image:

    ```console
    docker inspect --format '{{ '{{' }}index .Config.Labels "org.opencontainers.image.revision"{{ '}}' }}' $(docker compose ps -q bringup)
    ```

    For your own build, the command prints an empty line, because only CI writes this label. [What to note about your own build](#what-to-note-about-your-own-build) says what identifies your own build instead.

2. **The clone's commit:** this commit says which `compose.yaml` was used. The same holds for the udev rules, if the installed rules are the clone's ([What a version covers](#what-a-version-covers)). This command prints the commit:

    ```console
    git -C ~/diffbot rev-parse HEAD
    ```

3. **The settings:** what Compose made of them. `docker compose config` prints `compose.yaml` with every setting filled in, from `.env`, from variables in the shell and from the defaults. Run `docker compose config` right after the start, in the same shell. The output then shows what the containers were started with. This command saves the output for the report:

    ```console
    docker compose config > ~/robot-settings.yaml
    ```

4. **The firmware's commit:** this commit says which firmware the Teensy runs. Note the commit when you flash. On your PC, in the clone that you flash from:

    ```console
    git rev-parse HEAD
    ```

## Your own build

You need a build of your own only to run packages that you changed. For everything else, the published Docker image is enough.

The image has to be built for arm64, the Raspberry Pi's processor architecture. There are two places to build the image:

| Where | How the image is built for arm64 | What is known |
|:--|:--|:--|
| [On the Raspberry Pi](#on-the-raspberry-pi) | Natively. The Raspberry Pi is an arm64 computer | Not run on a Raspberry Pi yet, so the build time on a Raspberry Pi isn't measured. CI runs the same Dockerfile on an arm64 computer of GitHub |
| [On your PC](#on-your-pc-for-the-raspberry-pi) | Under emulation, on a PC with an amd64 processor. The emulator QEMU runs the arm64 programs of the build on that processor | Not tested yet. The build time isn't measured |

For comparison: CI builds the published image natively, on an arm64 computer of GitHub, in about two minutes ([How it's tested](image.md#how-its-tested)).

### On the Raspberry Pi

Give your build a name of its own in `.env`:

```bash title=".docker/robot/.env"
ROBOT_IMAGE=diffbot-robot:local
```

Then build the Docker image from your clone and start the containers. On the Raspberry Pi:

```console
docker compose up -d --build --pull never
```

- **`--build`:** builds the image before the start, with the [Dockerfile](image.md#how-the-image-is-built) and the packages in your clone.
- **`--pull never`:** keeps Compose from looking for `diffbot-robot:local` in a registry.

The first build downloads the official ROS Docker image and compiles the robot's packages. Later builds reuse the steps whose input didn't change, so they're usually much faster. The name `diffbot-robot:local` keeps your build apart from the published versions. To go back to them, remove the line from `.env` and run `docker compose pull --policy always` and `docker compose up -d --no-build`.

### On your PC, for the Raspberry Pi

!!! warning "Not tested yet"
    These steps follow Docker's documentation for a build under emulation. They haven't been run with the robot's Docker image yet, and their build time isn't measured. CI doesn't check them either: it builds and tests the arm64 image natively, on GitHub's arm64 runner. Setting up the builder may need a privileged container, or an emulator registered on the PC. So use a computer where that is allowed.

A PC with an amd64 processor can build the arm64 Docker image too, as an emulated build. That's one of the three ways to a multi-platform build that Docker's guide names ([the guide's section on the strategies](https://docs.docker.com/build/building/multi-platform/#strategies)). An emulated build isn't cross compilation, where a compiler on the PC would produce arm64 programs. Here the compiler is an arm64 program itself, and the emulator QEMU runs the compiler on the PC's processor. So the Dockerfile needs no change, but the build steps can run much slower than on an arm64 computer. A computer with an arm64 processor needs no emulation and builds the image natively.

This page offers the emulated build because that build uses the Dockerfile as the file is ([the guide's section on QEMU](https://docs.docker.com/build/building/multi-platform/#qemu)). The Dockerfile installs the packages with apt and rosdep and compiles the workspace with catkin, all inside the image that is being built. So all of these programs are arm64 programs. Cross compilation would need another Dockerfile: one with a build stage that runs on the PC's own architecture, and with a compiler that is told to produce arm64 programs ([the guide's section on cross-compilation](https://docs.docker.com/build/building/multi-platform/#cross-compilation)). The robot's Dockerfile has no such stage.

1. **Check for emulation:** the builder that Docker uses on the PC has to support arm64. This command lists the builders with their platforms, and marks the selected one with `*`. On your PC:

    ```console
    docker buildx ls
    ```

    The selected builder has to list `linux/arm64` among its platforms. Docker Desktop's builder does. With Docker Engine on Linux, Docker's guide names two ways to get there:

    - **A builder with the `docker-container` driver:** its BuildKit image brings the emulator along, so nothing is registered on the PC ([the guide's section on a custom builder](https://docs.docker.com/build/building/multi-platform/#custom-builder)). Buildx starts this builder as a privileged container ([the driver's source](https://github.com/docker/buildx/blob/v0.30.1/driver/docker-container/driver.go)). The builder doesn't end with the build: Docker starts the builder again after a restart, until you remove the builder with `docker buildx rm` and the builder's name ([Docker's page on this driver](https://docs.docker.com/build/builders/drivers/docker-container/)).
    - **QEMU registered on the PC:** the guide registers the emulator with the PC's kernel, through a privileged container ([the guide's section on QEMU](https://docs.docker.com/build/building/multi-platform/#qemu)).

2. **Build the image for arm64:** this command builds the image from the files of your clone. Run the command on your PC, in the root folder of your clone of the diffbot repository:

    ```console
    docker buildx build --platform linux/arm64 -f .docker/robot/Dockerfile -t diffbot-robot:local --load .
    ```

    `--platform linux/arm64` names the processor architecture to build for. `--load` puts the result into the PC's Docker, next to the PC's other images.

3. **Copy the image to the Raspberry Pi:** this command sends the image over SSH. Run the command on your PC, with the Raspberry Pi's user name and address:

    ```console
    docker save diffbot-robot:local | gzip | ssh <user>@<robot-ip> docker load
    ```

    `docker save` writes the image as an archive, `gzip` compresses the archive, `ssh` sends the archive to the Raspberry Pi, and `docker load` reads the archive into the Raspberry Pi's Docker.

4. **Start the copied image:** set `ROBOT_IMAGE=diffbot-robot:local` in `.env`. Then start the containers with the copied image. On the Raspberry Pi, in `~/diffbot/.docker/robot`:

    ```console
    docker compose up -d --no-build --pull never
    ```

### What to note about your own build

Your own build has no commit written into the image. So note where the build comes from, at the time of the build and on the computer that builds. Run these two commands in the clone that you build from:

```console
git rev-parse HEAD
git status --short
```

- **`git rev-parse HEAD`:** prints the commit that the clone is at.
- **`git status --short`:** lists the files that differ from that commit. If the command lists any files, the commit alone doesn't describe the build.

The simple way is to commit your changes before you build. Otherwise keep the changes with your notes. This command writes them to a file:

```console
git diff --binary HEAD > own-build.patch
```

The file doesn't hold new files that Git doesn't track yet. `git status --short` marks them with `??`. Keep a copy of each of them too.
