# Start, Check and Stop

Once the Raspberry Pi is [set up](setup.md), these commands start, check and stop the robot. The `docker compose` commands run on the Raspberry Pi, in the folder `~/diffbot/.docker/robot`, where Compose finds `compose.yaml` and your settings in `.env`.

## Start

Flash the Teensy's firmware first ([Microcontroller](../teensy-mcu.md)), and connect the Teensy and the lidar to the Raspberry Pi's USB ports. If the ROS master runs on your PC, start `roscore` there, in the dev container, before the robot: without the master, the robot's containers stop at once.

On the Raspberry Pi:

```console
cd ~/diffbot/.docker/robot
docker compose up -d --no-build
```

`up` creates and starts the containers described in `compose.yaml`: the bringup, and the lidar if `COMPOSE_PROFILES=lidar` is set. `-d` runs them in the background and gives you the prompt back. The first start downloads the image that CI published, [`ghcr.io/ros-mobile-robots/diffbot-robot`](https://github.com/orgs/ros-mobile-robots/packages/container/package/diffbot-robot), with the packages already compiled. Compressed, it's about 0.7 GB for amd64. Later starts use the downloaded image. `--no-build` makes a failed download an error. Without it, Compose would build the image on the Raspberry Pi instead, which takes long.

## Check

On the Raspberry Pi, list the containers:

```console
docker compose ps -a
```

On a robot without a lidar, it shows one container, the bringup, with the status `Up`. This output, like the one below, was recorded without hardware:

```text
NAME                      IMAGE                  COMMAND                  SERVICE   CREATED          STATUS          PORTS
diffbot-robot-bringup-1   diffbot-robot:noetic   "/entrypoint.sh rosl…"   bringup   26 seconds ago   Up 26 seconds
```

With the lidar, a second line follows, for the service `lidar`. `-a` lists stopped containers too: a container that stopped shows `Exited` and a number, its exit code, and its output tells why.

Then follow the containers' output:

```console
docker compose logs -f
```

Each line starts with the container it comes from. `Ctrl+C` stops following, and the robot keeps running. The bringup's output starts like this, shortened at the `...`. It was recorded with the robot image without hardware, with this page's example addresses:

```text
bringup-1  | started roslaunch server http://192.168.1.20:43309/
...
bringup-1  | NODES
bringup-1  |   /diffbot/
bringup-1  |     controller_spawner (controller_manager/spawner)
bringup-1  |     diffbot_base (diffbot_base/diffbot_base)
bringup-1  |     robot_state_publisher (robot_state_publisher/robot_state_publisher)
bringup-1  |     rosserial_base_controller (rosserial_python/serial_node.py)
bringup-1  |
bringup-1  | ROS_MASTER_URI=http://192.168.1.10:11311
bringup-1  |
bringup-1  | process[diffbot/rosserial_base_controller-1]: started with pid [52]
bringup-1  | process[diffbot/diffbot_base-2]: started with pid [53]
bringup-1  | process[diffbot/controller_spawner-3]: started with pid [54]
bringup-1  | process[diffbot/robot_state_publisher-4]: started with pid [55]
...
bringup-1  | [INFO] [1793227440.919570252]: ... Done Initializing DiffBot Hardware Interface
...
bringup-1  | [INFO] [1793227441.364928]: Connecting to /dev/ttyACM0 at 115200 baud
```

- **`started roslaunch server`** with the Raspberry Pi's address: the bringup got `ROS_IP`.
- **`ROS_MASTER_URI`** with the PC's address: the bringup uses the master on the PC.
- **`NODES` and the `started` lines:** the four nodes of `bringup.launch` run: rosserial, the hardware interface `diffbot_base`, the controller spawner and the robot state publisher.
- **`Connecting to /dev/ttyACM0`:** rosserial opens the Teensy.

With the Teensy, rosserial then prints *Requesting topics...*, and a line *Setup publisher on …* or *Setup subscriber on …* for each topic of the firmware. The controller spawner ends with *Started controllers: joint_state_controller, mobile_base_controller*.

The Teensy's lines will be added after the test on the robot ([diffbot#109](https://github.com/ros-mobile-robots/diffbot/issues/109)).

Then check on your PC, in the dev container, that the robot's topics have data. A topic that's listed but has no messages isn't enough:

```console
rostopic hz /diffbot/measured_joint_states
rostopic echo -n 1 /diffbot/mobile_base_controller/odom
rostopic hz /diffbot/scan
```

`measured_joint_states` comes from the Teensy: messages at a steady rate mean that rosserial talks with the firmware and the encoders are read. `odom` is the controller's odometry. `scan` comes from the lidar, if it runs.

## Drive

From your PC, first with the wheels off the ground, then slowly on the floor:

```console
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/diffbot/mobile_base_controller/cmd_vel
```

## Stop

On the Raspberry Pi:

```console
docker compose down
```

`down` stops and removes the containers. The image stays for the next start, and so do the ROS logs in `~/robot-data`. The output that `docker compose logs` shows is removed with the containers. Before you switch the Raspberry Pi off, shut it down with `sudo poweroff`, so the SD card isn't written to while the power goes.

## All commands

On the Raspberry Pi, in `~/diffbot/.docker/robot`:

| Task | Command |
|:--|:--|
| Start, or restart after a change of `.env` | `docker compose up -d --no-build` |
| Is it running? | `docker compose ps -a` |
| Follow the output | `docker compose logs -f` |
| Stop | `docker compose down` |
| Update | `git pull`, `docker compose pull --policy always`, then `docker compose up -d --no-build` |
| Switch off | `docker compose down`, then `sudo poweroff` |

!!! warning "After the Teensy was reset, reflashed or unplugged"
    Stop the containers and start them again: `docker compose down`, then `docker compose up -d --no-build`. A container gets its devices when it starts, and a reconnected Teensy is a new device for the kernel, which the running container doesn't see. A new start also starts a new odometry, so a measurement run ends there.

??? info "Running a command in a container"
    Most ROS commands run on your PC. To run one on the robot itself, for example when no PC is connected:

    ```console
    docker compose exec bringup /entrypoint.sh rostopic list
    ```

    | Part | From | What it does |
    |:--|:--|:--|
    | `docker compose` | Docker | Works on the containers that `compose.yaml` in the current folder describes |
    | `exec` | Docker | Runs a command in a running container |
    | `bringup` | `compose.yaml` | The name of the service, the container to run it in |
    | `/entrypoint.sh` | the diffbot repository | [Our script]({{ robot_container_src_url }}/.docker/robot/entrypoint.sh) that loads ROS and the workspace, then runs the rest of the line. Docker runs it when a container starts, but `exec` starts a new process without it, so the command names it |
    | `rostopic list` | ROS | The command itself |

    `docker compose exec bringup /entrypoint.sh bash` opens a shell in the container, with ROS loaded.

## Versions and your own build

`noetic`, the default, is the newest image that CI published from the `noetic-devel` branch. `docker compose pull --policy always` downloads it again when there's a newer one, and `docker compose up -d --no-build` then recreates the containers with it.

Each published image also has a name with the full commit it was built from, `noetic-<commit>`, which never changes. Its digest, `sha256:` followed by 64 characters, names exactly the same image content. The [package's page](https://github.com/orgs/ros-mobile-robots/packages/container/package/diffbot-robot) lists both. To stay on one version, or to go back to an earlier one, set it in `.env`, by its commit:

```bash title=".docker/robot/.env"
ROBOT_IMAGE=ghcr.io/ros-mobile-robots/diffbot-robot:noetic-<commit>
```

Or exactly, by its digest. `@` and the digest replace `:` and the name:

```bash title=".docker/robot/.env"
ROBOT_IMAGE=ghcr.io/ros-mobile-robots/diffbot-robot@sha256:<digest>
```

Use the digest of the version itself, the one the package's page shows for `noetic-<commit>`, not the digest of one architecture's image. Then download and restart:

```console
docker compose pull --policy always
docker compose up -d --no-build
```

To go back to the newest, remove the line from `.env` and run both commands again. Removing the line alone changes nothing that runs. The version only covers the image. `compose.yaml`, the udev rules and `.env` come from your clone of the repository, and the Teensy's firmware from your last flash.

**For a test report**, for example the robot test in [diffbot#109](https://github.com/ros-mobile-robots/diffbot/issues/109), note what ran, on the Raspberry Pi:

- the image's digest, in the `DIGEST` column of `docker images --digests ghcr.io/ros-mobile-robots/diffbot-robot`, for the tag you ran
- the commit the image was built from:

    ```console
    docker inspect --format '{{ '{{' }}index .Config.Labels "org.opencontainers.image.revision"{{ '}}' }}' $(docker compose ps -q bringup)
    ```

- your clone's commit, `git -C ~/diffbot rev-parse HEAD`, and the settings in `.env`
- the commit you flashed the Teensy's firmware from, on your PC

**Your own build**, to run changed packages: set `ROBOT_IMAGE=diffbot-robot:local` in `.env`, and build the image from your clone, on the Raspberry Pi:

```console
docker compose up -d --build --pull never
```

The first build downloads the official ROS image and compiles the robot's packages, which takes a while. Later builds reuse the steps whose input didn't change, so they're usually much faster. The name `diffbot-robot:local` keeps your build apart from the published versions. To go back to them, remove the line from `.env` and run `docker compose pull --policy always` and `docker compose up -d --no-build`.

## Logs

The robot's messages end up in three places.

### What the containers print

Everything that the bringup and the lidar driver print, as in [Check](#check) above. Docker keeps it on the Raspberry Pi, at most 100 MB per container, until `docker compose down` removes the containers. To read it again, for example only the bringup's:

```console
docker compose logs bringup
```

### ROS's log files

roslaunch and each node also write a log file. They go to `~/robot-data/log` on the Raspberry Pi, with a folder for each start, and `latest` always points to the newest one. These files stay after `docker compose down`. To read them on your PC, copy them there, on the PC:

```console
scp -r <user>@<robot-ip>:robot-data/log/latest robot-logs
```

### All nodes' messages, live

Each node also sends its messages to the node `rosout`, which runs with the ROS master and collects them from the robot and the PC. With the master on your PC, [rqt_console](https://wiki.ros.org/rqt_console) shows them there, live, with filters by node and severity. On the PC, in the dev container:

```console
rqt_console
```
