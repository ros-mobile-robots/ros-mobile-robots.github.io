# Versions and Your Own Build

This page is about the robot's Docker image: which version runs, how to update the image or stay on one version, what a test report notes, and how to build the image yourself. You don't need this page to start the robot. [Start, Check and Stop](usage.md) covers that.

The commands run on the Raspberry Pi, in the folder `~/diffbot/.docker/robot`, unless a step names your PC.

## Versions

A version is one build of the robot's Docker image, from one commit of the diffbot repository. The image's full name says where the image is stored and which version is meant:

```text
ghcr.io/ros-mobile-robots/diffbot-robot:noetic
```

| Part of the name | What the part is |
|:--|:--|
| `ghcr.io` | The registry: the server that stores the image. This one is GitHub's |
| `ros-mobile-robots/diffbot-robot` | The image, in the organization ros-mobile-robots. The [package's page](https://github.com/orgs/ros-mobile-robots/packages/container/package/diffbot-robot) lists its versions |
| `noetic`, after the colon | The tag: a label that chooses one version. The table below has the two kinds of tags |

| Tag | Which version the tag chooses |
|:--|:--|
| `noetic` | The newest version that CI published from the branch `noetic-devel`. This tag moves to each new version |
| `noetic-<commit>` | The version that was built from that commit, with the commit's full 40 characters. This tag never moves |

The registry only stores the versions. A version runs on the robot after a pull: Docker downloads the version from the registry and keeps it on the Raspberry Pi. The setting `ROBOT_IMAGE` in `.env` holds the name that Compose pulls and starts ([All settings](setup.md#all-settings)). Without this setting, the name is the one above, with the tag `noetic`.

That gives three choices, from the most frequent to the rarest:

| What you want | What to do |
|:--|:--|
| Start the robot with the version that is on the Raspberry Pi | `docker compose up -d`, as in [Start](usage.md#start). The command doesn't ask the registry again once a version is downloaded |
| Get the newest version | [Update to the newest version](#update-to-the-newest-version) |
| Run one exact version, for example to repeat a test | [Run one fixed version](#run-one-fixed-version) |

### Update to the newest version

Without `ROBOT_IMAGE`, the robot runs the tag `noetic`. To update, on the Raspberry Pi:

```console
git pull && docker compose pull --policy always && docker compose up -d
```

1. `git pull` updates your clone of the repository, with `compose.yaml` and the udev rules.
2. `docker compose pull --policy always` asks the registry for `noetic` again, and downloads the image if there's a newer one.
3. `docker compose up -d` recreates the containers with the downloaded image.

`&&` runs the next command only if the command before it succeeded. So a failed download stops the update. Without `&&`, the last command would run anyway and start the version that was on the Raspberry Pi before, and the update would look finished. [Which version runs](#which-version-runs) shows how to check the result.

### Run one fixed version

Each published version also has a tag with the full commit it was built from: `noetic-<commit>`. The [package's page](https://github.com/orgs/ros-mobile-robots/packages/container/package/diffbot-robot) lists these tags. The page also lists two tags per version that end in `-amd64` and `-arm64`. They are the version's two builds, one per processor architecture. Use the tag without such an ending: Docker then picks the build for the Raspberry Pi by itself.

To stay on one version, or to go back to an earlier one, copy that version's whole tag from the package's page and set the name in `.env`:

```bash title=".docker/robot/.env"
ROBOT_IMAGE=ghcr.io/ros-mobile-robots/diffbot-robot:noetic-<commit>
```

Then download that version and restart the containers. On the Raspberry Pi:

```console
docker compose pull --policy always && docker compose up -d
```

To go back to the newest version, remove the line from `.env` and run the same command again. Removing the line alone changes nothing that runs.

??? info "Even more exact: the digest"
    A digest is a checksum of a Docker image's content: `sha256:` followed by 64 characters. If one byte of the image changes, the digest changes. So a digest names exactly one content, whatever names point to it ([Docker's reference on pulling by digest](https://docs.docker.com/reference/cli/docker/image/pull/#pull-an-image-by-digest-immutable-identifier)).

    `noetic-<commit>` is enough to stay on one version, because a published version is never replaced ([How it's published](image.md#how-its-published)). The digest is for a record that has to be exact. The package's page shows the digest of each version. In `.env`, `@` and the digest replace `:` and the tag:

    ```bash title=".docker/robot/.env"
    ROBOT_IMAGE=ghcr.io/ros-mobile-robots/diffbot-robot@sha256:<digest>
    ```

    Use the digest that the package's page shows for `noetic-<commit>` itself, not the digest of one architecture's image. On the Raspberry Pi, `docker images --digests ghcr.io/ros-mobile-robots/diffbot-robot` lists the downloaded versions with their digests, in the column `DIGEST`.

### Which version runs

CI writes the commit that a version was built from into the image, as a label. This command prints the label of the image that the running bringup container uses. On the Raspberry Pi, while the robot runs:

```console
docker inspect --format '{{ '{{' }}index .Config.Labels "org.opencontainers.image.revision"{{ '}}' }}' $(docker compose ps -q bringup)
```

The command prints the commit's 40 characters. After an update, that's the commit of the newest published version, which the package's page shows in the tag `noetic-<commit>`. With a fixed version, it's the commit in the tag that you set. For your own build, the command prints an empty line, because only CI writes this label. [What to note about your own build](#what-to-note-about-your-own-build) says what identifies your own build instead.

### What a version covers

A version covers the Docker image, and nothing else. Three more parts decide what the robot does, and each has a state of its own:

| Part | Where the part comes from | How to see the part's state |
|:--|:--|:--|
| **The Docker image:** ROS Noetic, the robot's compiled packages and the entrypoint script | The registry, in the version that `ROBOT_IMAGE` names | [Which version runs](#which-version-runs) prints the image's commit |
| **Your clone, `~/diffbot`:** `compose.yaml` and the udev rules | `git clone`, updated by `git pull` | [A clone's commit and its local changes](#a-clones-commit-and-its-local-changes) |
| **`.env`:** your settings | You. Nothing else changes `.env` | `cat ~/diffbot/.docker/robot/.env` |
| **The Teensy's firmware:** the program that drives the motors and reads the encoders | Your last flash, from the PC | The Teensy doesn't report the firmware's commit. Note the commit when you flash |

udev uses the copy of the rules that [step 7](setup.md#7-stable-device-names) of the setup installed, not the file in your clone. After a `git pull` that changed the rules, install them again.

### A clone's commit and its local changes

Two of the parts above come from a Git clone: `compose.yaml` from the clone on the Raspberry Pi, and the firmware from the clone on your PC. A commit describes a clone only if no file in the clone differs from that commit. So note two things. Run these two commands in the clone:

```console
git rev-parse HEAD
git status --short
```

- **`git rev-parse HEAD`:** prints the commit that the clone is at. The command prints the same commit after you changed a file without committing the change.
- **`git status --short`:** lists the files that differ from that commit. No output means that no file that Git tracks differs from the commit. Git ignores `.env`, your settings, on purpose. The test report records them on their own.

If `git status --short` lists files, keep the changes with your notes. This command writes them to a file:

```console
git diff --binary HEAD > local-changes.patch
```

The file doesn't hold new files that Git doesn't track yet. `git status --short` marks them with `??`. Keep a copy of each of them too. The simple way around all this is to commit your changes first.

### What a test report needs

A test report, for example for the robot test in [diffbot#109](https://github.com/ros-mobile-robots/diffbot/issues/109), says what ran, so that someone else can repeat the test. That takes four notes, one for each part in [the table above](#what-a-version-covers). The first three are taken on the Raspberry Pi, in `~/diffbot/.docker/robot`, while the robot runs.

1. **The Docker image's commit:** this commit says which code the containers run. [Which version runs](#which-version-runs) has the command that prints it.

2. **The clone's commit and local changes:** they say which `compose.yaml` was used. The same holds for the udev rules, if the installed rules are the clone's ([What a version covers](#what-a-version-covers)). These commands print the commit and list the changed files of the clone in `~/diffbot`:

    ```console
    git -C ~/diffbot rev-parse HEAD
    git -C ~/diffbot status --short
    ```

    If the second command lists files, save the changes with the report, as [A clone's commit and its local changes](#a-clones-commit-and-its-local-changes) shows.

3. **The settings:** what Compose made of them. `docker compose config` prints `compose.yaml` with every setting filled in, from `.env`, from variables in the shell and from the defaults. Run `docker compose config` right after the start, in the same shell. The output then shows what the containers were started with. This command saves the output for the report:

    ```console
    docker compose config > ~/robot-settings.yaml
    ```

4. **The firmware's commit and local changes:** they say which firmware the Teensy runs. The Teensy can't report them later, so note them when you flash. Run the two commands of [A clone's commit and its local changes](#a-clones-commit-and-its-local-changes) on your PC, in the clone that you flash from, and save the changes if there are any.

## Your own build

You need a build of your own only to run packages that you changed. For everything else, the published Docker image is enough.

The image has to be built for arm64, the Raspberry Pi's processor architecture. There are two places to build the image:

| Where | How the image is built for arm64 | What is known |
|:--|:--|:--|
| [On the Raspberry Pi](#on-the-raspberry-pi) | Natively. The Raspberry Pi is an arm64 computer | Remo's Raspberry Pi 4 B built the image in 14 min 20 s, at the diffbot repository's commit `3541965` ([the bench record](https://github.com/ros-mobile-robots/diffbot/issues/160#issuecomment-6550353204)). The build command on this page, with the file `compose.build.yaml`, hasn't run on a Raspberry Pi yet |
| [On your PC](#on-your-pc-for-the-raspberry-pi) | Under emulation, on a PC with an amd64 processor. The emulator QEMU runs the arm64 programs of the build on that processor | Not tested yet. The build time isn't measured |

For comparison: CI builds the published image natively, on an arm64 computer of GitHub, in about two minutes ([How it's tested](image.md#how-its-tested)).

### On the Raspberry Pi

`compose.yaml`, the file that runs the robot, never builds an image. A second file next to it, [`compose.build.yaml`]({{ robot_container_src_url }}/.docker/robot/compose.build.yaml), holds the build. These lines of the file define the build's one service ([lines 11–17]({{ robot_container_src_url }}/.docker/robot/compose.build.yaml#L11-L17)):

```yaml title="compose.build.yaml" linenums="11"
--8<-- "{{ robot_container_src_raw }}/.docker/robot/compose.build.yaml:11:17"
```

- **`robot-image`** (line 12): the service's name. The build has a service of its own, apart from `bringup` and `lidar` in `compose.yaml`.
- **`image`** (line 13): the build's name, `diffbot-robot:local`. The name is fixed in this file, and `ROBOT_IMAGE` doesn't change it. So your build never gets the name of the published image, and a later start can't take your build for a published version.
- **`build`** (lines 14–17): tells Compose how to build the image, from the [Dockerfile](image.md#how-the-image-is-built) in `.docker/robot`. Its `context` is the whole repository, the folder whose files the Dockerfile can copy into the image.

Three steps build the image and run it:

1. **Build the image from your clone:** `-f` names the Compose file to read, here the file with the build alone. On the Raspberry Pi:

    ```console
    docker compose -f compose.build.yaml build
    ```

    The first build downloads the official ROS Docker image and compiles the robot's packages. Later builds reuse the steps whose input didn't change, so they're usually much faster.

2. **Choose your build in `.env`:**

    ```bash title=".docker/robot/.env"
    ROBOT_IMAGE=diffbot-robot:local
    ```

3. **Start the containers with your build:** on the Raspberry Pi, run this command.

    ```console
    docker compose up -d --pull never
    ```

    `--pull never` keeps Compose from looking for `diffbot-robot:local` in a registry.

After another change of the code, repeat steps 1 and 3. To go back to the published versions, remove the line from `.env` and run `docker compose pull --policy always && docker compose up -d`.

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
    docker compose up -d --pull never
    ```

### What to note about your own build

Your own build has no commit written into the image. So note where the build comes from, at the time of the build and on the computer that builds: the commit and the local changes of the clone that you build from. [A clone's commit and its local changes](#a-clones-commit-and-its-local-changes) has the commands.
