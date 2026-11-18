# Troubleshooting

What you might see while setting up or starting the [robot container](index.md), how to check the cause, and how to fix it. Find the step that failed, then the heading with the message that you see:

1. [The download fails](#the-download-fails): `docker compose up` or `pull` can't get the robot's Docker image.
2. [The containers don't start](#the-containers-dont-start): the image is there, but Compose or the bringup stops.
3. [The robot runs, but data is missing](#the-robot-runs-but-data-is-missing): the containers are `Up`, but a topic stays empty.
4. [Logging in and Docker's permissions](#logging-in-and-dockers-permissions): SSH or the `docker` command itself fails.

The commands run on the Raspberry Pi, in `~/diffbot/.docker/robot`, unless a step says *on the PC*. Each entry ends with what you see once the fix worked.

## The download fails

The first start, and every update, downloads the robot's Docker image from GitHub's registry, `ghcr.io`. Start with the registry itself. Then the message of the failed download says what to check: the image's name, or the access to the package.

### The first check: does the registry answer?

This command sends one request to the registry and prints the number of the registry's answer:

```console
curl -sS -o /dev/null -w '%{http_code}\n' https://ghcr.io/v2/
```

- **`401`:** the registry answers. 401 means "log in first", which is the registry's normal answer to a request without a login. Docker handles that step by itself for a public image. So the network is fine: go on with the entry below that has the message of your failed download.
- **`000` and an error, such as *Could not resolve host* or *Connection timed out*:** the Raspberry Pi doesn't reach the registry. Connect the Raspberry Pi to the internet: check the Wi-Fi or the cable, and the router.
- **Then:** the command prints `401`.

### `docker compose up` or `pull` stops with *not found* or *manifest unknown*

The registry answered, but it has no version under the name that Compose asked for.

- **Check:** `docker compose config --images` prints the image name that Compose uses, once per service. Without `ROBOT_IMAGE` in `.env`, that's `ghcr.io/ros-mobile-robots/diffbot-robot:noetic`. The [package's page](https://github.com/orgs/ros-mobile-robots/packages/container/package/diffbot-robot) lists the versions that exist.
- **Fix:** correct the name or the version in `ROBOT_IMAGE`, or remove the line for the newest version ([Versions](versions.md#versions)).
- **Then:** `docker compose pull` ends without an error, and `docker compose up -d` starts the containers.

### `docker compose up` or `pull` stops with *denied*

The registry answered, but it doesn't let a reader without a login download the image. That happens for a name that the registry doesn't have at all, and for a package that isn't public.

- **Check:** `docker compose config --images` prints the image name that Compose uses. If the name is the expected one, `ghcr.io/ros-mobile-robots/diffbot-robot` with a tag, open the [package's page](https://github.com/orgs/ros-mobile-robots/packages/container/package/diffbot-robot) in a browser that isn't logged in to GitHub. A public package shows its versions there.
- **Fix:** correct a wrong name in `ROBOT_IMAGE`. If the name is right and the page doesn't show the package, the image isn't published for everyone yet: report that in the [diffbot repository's issues](https://github.com/ros-mobile-robots/diffbot/issues). Until then, [your own build](versions.md#your-own-build) runs the robot.
- **Then:** `docker compose pull` ends without an error.

## The containers don't start

### Compose stops with *bind source path does not exist*

The folder for ROS's log files is missing.

- **Check:** `ls -ld ~/robot-data`
- **Fix:** `mkdir -p ~/robot-data`, or set `ROBOT_DATA` to the folder that you use ([step 8](setup.md#8-the-folder-for-logs)).
- **Then:** `docker compose up -d` creates the containers.

### Compose stops with *error gathering device information* for `/dev/teensy` or `/dev/rplidar`

A serial device that the settings name isn't there.

- **Check:** `ls -l /dev/teensy /dev/rplidar` and `lsusb`
- **Fix:** plug the device in, and install the udev rules ([step 7](setup.md#7-stable-device-names)). On a robot without a lidar, remove `COMPOSE_PROFILES=lidar` from `.env`.
- **Then:** `ls -l` shows each name as a link to the kernel's device, and `docker compose up -d` creates the containers.

### A setting in `.env` has no effect

A variable with the same name in the Raspberry Pi's shell wins over `.env` ([When Compose reads `.env`](setup.md#when-compose-reads-env)).

- **Check:** `docker compose config --environment` shows the value that Compose uses.
- **Fix:** remove the variable's `export`, for example from `~/.bashrc`, and log in again.
- **Then:** `docker compose config --environment` shows the value from `.env`. Run `docker compose up -d` to apply it.

### The bringup's output shows *Permission denied* for `/dev/ttyACM0`

The user in the container isn't in the group that owns the Teensy's device ([Why `DIALOUT_GID` is a number](setup.md#why-dialout_gid-is-a-number)).

- **Check:** `stat -Lc %g /dev/teensy` prints the group's number.
- **Fix:** write that number into `DIALOUT_GID` in `.env`, then run `docker compose up -d`.
- **Then:** the bringup's output goes on with *Requesting topics...* after *Connecting to /dev/ttyACM0*.

### The bringup stops with *cannot create log directory* and *Permission denied* for `/home/ros/.ros/log`

The user in the container, with the user ID 1000, can't write into the folder for logs.

- **Check:** `ls -ldn ~/robot-data ~/robot-data/log`. UID 1000 must be able to write, for example as the owner `1000` with `drwx`.
- **Fix:** `sudo chown -R 1000:1000 ~/robot-data`, then `sudo chmod -R u+rwX ~/robot-data`, then `docker compose up -d`.
- **Then:** `docker compose ps -a` shows the bringup as `Up`.

### The bringup or the lidar stops with *unable to contact ROS master*

No ROS master answers at the address in `ROS_MASTER_URI`.

- **Check:** `roscore` runs on the PC. From the Raspberry Pi, `ping` the PC's address.
- **Fix:** start `roscore` first, then run `docker compose up -d`. Correct `ROS_MASTER_URI` if the address is wrong. A PC with WSL 2 needs [mirrored networking](../ros-network-setup.md#work-machine-on-windows-wsl-2).
- **Then:** `docker compose ps -a` shows the containers as `Up`, and the bringup's output shows `ROS_MASTER_URI` with the PC's address.

### The bringup or the lidar stops with *run_id on parameter server does not match declared run_id*

Without `ROS_MASTER_URI` and with the lidar, both containers started a ROS master.

- **Check:** `docker compose ps -a` shows one of the two containers as `Exited`.
- **Fix:** `docker compose --profile lidar down`. Then use the robot alone without the lidar, or the ROS master on the PC ([Without any PC](usage.md#without-any-pc)).
- **Then:** `docker compose ps -a` shows every listed container as `Up`.

### `docker compose exec bringup` with a ROS command stops with *executable file not found in $PATH*

The command has no `/entrypoint.sh` in front of the ROS command, so ROS isn't loaded ([On the robot alone](usage.md#on-the-robot-alone)).

- **Fix:** put the script in front: `docker compose exec bringup /entrypoint.sh rostopic list`
- **Then:** the command prints the list of topics.

### The lidar still runs after removing `COMPOSE_PROFILES=lidar`

Without the profile, Compose no longer stops the lidar container.

- **Check:** `docker compose ps -a` still lists the service `lidar`.
- **Fix:** `docker compose --profile lidar down`, then `docker compose up -d`.
- **Then:** `docker compose ps -a` lists the bringup alone.

After every change of `.env`, run `docker compose up -d` again: Compose recreates the containers whose settings changed. To switch the lidar off, run `docker compose down` first, while `COMPOSE_PROFILES=lidar` is still set.

## The robot runs, but data is missing

### The PC lists the robot's topics, but `rostopic hz` shows no messages

The nodes found each other through the ROS master, but their direct connections fail.

- **Check:** `ROS_IP` on each machine is that machine's own address.
- **Fix:** correct `ROS_IP` on the robot or on the PC. Both directions must work ([ROS Network Setup](../ros-network-setup.md)).
- **Then:** on the PC, `rostopic hz /diffbot/measured_joint_states` prints lines with `average rate:`.

### The bringup's output shows *Unable to sync with device*

rosserial opened the serial port, but the Teensy doesn't answer.

- **Check:** the Teensy has the firmware, and the USB cable carries data, not only power.
- **Fix:** flash the firmware ([Microcontroller](../teensy-mcu.md)), then run `docker compose down` and `docker compose up -d`.
- **Then:** the bringup's output shows a line *Setup publisher on …* for each topic of the firmware.

### No `/diffbot/scan`

The lidar container doesn't run, or the lidar driver stopped. On a robot without a lidar, and with the ROS master on the Raspberry Pi, no lidar container runs: skip the scan check then.

- **Check:** `docker compose ps -a` lists the service `lidar` with the status `Up`.
- **Fix:** if the list has no `lidar`, first make sure that the ROS master runs on the PC: `roscore` is started there, and `ROS_MASTER_URI` in `.env` has the PC's address ([The ROS master](usage.md#the-ros-master)). Then set `COMPOSE_PROFILES=lidar` in `.env` and run `docker compose up -d`. If the list shows `lidar` as `Exited`, `docker compose logs lidar` prints why.
- **Then:** on the PC, `rostopic hz /diffbot/scan` prints lines with `average rate:`.

## Logging in and Docker's permissions

### SSH can't connect

- **Check:** the router lists the Raspberry Pi. The first boot takes a few minutes.
- **Fix:** wait. If the Raspberry Pi never appears, check the Wi-Fi name and password in Imager and write the card again.
- **Then:** `ssh <user>@<robot-ip>` asks for the password.

### SSH finds the IP address, but not the name that ends in `.local`

The Raspberry Pi announces its name with the service Avahi, which the first boot installs. Without a network at the first boot, that installation fails, and `cloud-init status --wait` ends with `status: error`.

- **Check:** `ssh <user>@<robot-ip>` works with the IP address from the router's list.
- **Fix:** log in with the IP address and install the service: `sudo apt install avahi-daemon`
- **Then:** `ssh <user>@<hostname>.local` connects, from PCs that look up `.local` names.

### `docker` commands fail with *permission denied* for `docker.sock`

Your user isn't in the group `docker` yet, or the login is older than the group.

- **Check:** `groups` lists `docker`.
- **Fix:** log out and in again after the `usermod` command of [step 5](setup.md#5-docker-engine).
- **Then:** `docker run --rm hello-world` prints *Hello from Docker!*

[Logs](usage.md#logs) explains where to find the robot's messages.
