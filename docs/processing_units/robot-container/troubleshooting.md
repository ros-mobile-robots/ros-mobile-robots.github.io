# Troubleshooting

What you might see while setting up or starting the [robot container](index.md), how to check the cause, and how to fix it. The commands run on the Raspberry Pi, in `~/diffbot/.docker/robot`, unless the row says *on the PC*.

| What you see | Check | Fix |
|:--|:--|:--|
| `docker compose up` stops with *No such image*, after *denied* or *not found* | `ROBOT_IMAGE` in `.env` | Correct the name or the version, or remove the line for the newest |
| `docker compose up` or `pull` can't reach `ghcr.io` | `ping ghcr.io` | Connect the Raspberry Pi to the internet |
| SSH can't connect | The router lists the Raspberry Pi. The first boot takes a few minutes | Wait. If it never appears, check the Wi-Fi name and password in Imager and write the card again |
| `docker` commands fail with *permission denied* for `docker.sock` | `groups` lists `docker` | Log out and in again after `usermod` |
| Compose stops with *bind source path does not exist* | `ls -ld ~/robot-data` | `mkdir -p ~/robot-data`, or set `ROBOT_DATA` |
| Compose stops with *error gathering device information* for `/dev/teensy` or `/dev/rplidar` | `ls -l /dev/teensy /dev/rplidar`, `lsusb` | Plug the device in, install the udev rules ([step 7](setup.md#7-stable-device-names)). Without a lidar, remove `COMPOSE_PROFILES=lidar` |
| A setting in `.env` has no effect | `docker compose config --environment` shows the value that Compose uses | A variable with the same name in the shell wins over `.env`: remove its `export`, for example from `~/.bashrc`, log in again, then `docker compose up -d --no-build` ([When Compose reads `.env`](setup.md#when-compose-reads-env)) |
| The bringup's output shows *Permission denied* for `/dev/ttyACM0` | `stat -Lc %g /dev/teensy` | Write that number into `DIALOUT_GID` |
| The bringup stops with *cannot create log directory* and *Permission denied* for `/home/ros/.ros/log` | `ls -ldn ~/robot-data ~/robot-data/log`: UID 1000 must be able to write, for example owner `1000` with `drwx` | `sudo chown -R 1000:1000 ~/robot-data`, then `sudo chmod -R u+rwX ~/robot-data` |
| The bringup or the lidar stops with *unable to contact ROS master* | `roscore` runs on the PC. `ping` the PC's address from the Raspberry Pi | Start `roscore` first, then `docker compose up -d --no-build`. Correct `ROS_MASTER_URI`. A PC with WSL 2 needs [mirrored networking](../ros-network-setup.md#work-machine-on-windows-wsl-2) |
| The PC lists the robot's topics, but `rostopic hz` shows no messages | `ROS_IP` on each machine is its own address | Correct `ROS_IP` on the robot or the PC. Both directions must work ([ROS Network Setup](../ros-network-setup.md)) |
| The bringup's output shows *Unable to sync with device* | The Teensy has the firmware, the USB cable carries data | Flash the firmware ([Microcontroller](../teensy-mcu.md)) |
| No `/diffbot/scan` | `docker compose ps -a` lists `lidar` as running | Set `COMPOSE_PROFILES=lidar`, then `docker compose up -d --no-build`. If it exited, its output in `docker compose logs lidar` tells why |
| The lidar still runs after removing `COMPOSE_PROFILES=lidar` | `docker compose ps -a` | `docker compose --profile lidar down`, then `docker compose up -d --no-build` |

After changing `.env`, run `docker compose up -d --no-build` again: Compose recreates the containers whose settings changed. To switch the lidar off, run `docker compose down` first, while `COMPOSE_PROFILES=lidar` is still set.

[Logs](usage.md#logs) explains where to find the robot's messages.
