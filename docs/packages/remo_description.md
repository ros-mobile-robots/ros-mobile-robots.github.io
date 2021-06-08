# Remo Description


## Camera Types

The [`remo.urdf.xacro`]({{ remo_repo_url }}/urdf/remo.urdf.xacro) accepts a `camera_type`
[xacro arg](http://wiki.ros.org/xacro#Rospack_commands) which lets you choose between the following different camera types

| Raspicam v2 with IMX219 | OAK-1 | OAK-D |
|:-----------------------:|:-----:|:-----:|
| [<img src="{{ asset_dir }}/remo/camera_type/raspi-cam.png" width="700">]({{ asset_dir }}/remo/camera_type/raspi-cam.png) | [<img src="{{ asset_dir }}/remo/camera_type/oak-1.png" width="700">]({{ asset_dir }}/remo/camera_type/oak-1.png) | [<img src="{{ asset_dir }}/remo/camera_type/oak-d.png" width="700">]({{ asset_dir }}/remo/camera_type/oak-d.png) |

## Single Board Computer Types

Another xacro argument is the `sbc_type` wher you can select between `jetson` and `rpi`.

| Jetson Nano | Raspberry Pi 4 B |
|:-----------------------:|:-----:|:-----:|
| [<img src="{{ asset_dir }}/remo/sbc_type/jetson.png" width="700">]({{ asset_dir }}/remo/sbc_type/jetson.png) | [<img src="{{ asset_dir }}/remo/sbc_type/raspi.png" width="700">]({{ asset_dir }}/remo/sbc_type/raspi.png) |