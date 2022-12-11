# 3D Printing

This page provides help to 3D print the parts of Remo robot. It contains information
about configuring your slicer, suggestions to orient the parts and where support material is recommended to avoid
failing prints and wasted PLA material.

The table below gives an overview of the required parts of Remo and the average printing time:

!!! note
    Remo robot is a modular robotics platform which means you don't have to print all parts.
    It is often possible to choose between different variants. For example:

    - SBC Deck
    - LiDAR Platform
    - Camera Mount


| Qt | Part                                                |  File                                                | Material | Time  | Notes                                 |
|:--:|:----------------------------------------------------|:-----------------------------------------------------|:---------|:-----:|:--------------------------------------|
| 1  | [Chassis](#chassis)                                 | `chassis.stl`                                        | PLA      |       |                                       |
| 1  | [Caster wheel](#caster-wheel)                       | `caster_base_65mm.stl` and `caster_shroud_65mm.stl`  | PLA      |       | Print both parts of the caster wheel  |
| 1  | [SBC Decks](#sbc-decks)                             | `raspberry_pi_deck.stl` or `jetson_nano_deck.stl`    | PLA      |       | Select one depending on used SBC      |
| 1  | [LiDAR Platform](#lidar-platform)                   | `platfom_rplidar_a2.stl` or `platfom_rplidar_a1.stl` | PLA      |       | Select one dependin on used LiDAR     |
| 1  | [SLAMTEC USB to Serial holder](#lidar-platform)     | `slamtec_holder.stl` or `jetson_nano_deck.stl`       | PLA      |       |                                       |
| 1  | [Camera Mount](#camera-mount)                       | `camera_mount.stl`                                   | PLA      |       |                                       |

## Chassis

<script src="https://embed.github.com/view/3d/ros-mobile-robots/remo_description/main/meshes/remo/chassis.stl"></script>


## Caster Wheel

| Qty | Part | 3D View |
|:---:|:-----|:-------:|
| 1   | [`caster_base_65mm.stl`](https://github.com/ros-mobile-robots/remo_description/blob/main/meshes/remo/caster/caster_base_65mm.stl) | <script src="https://embed.github.com/view/3d/ros-mobile-robots/remo_description/main/meshes/remo/caster/caster_base_65mm.stl"></script> |

## SBC Decks

Currently there are two different decks you can print, depending on which SBC you are will use:

=== "RPi Deck"

    | Qty | Part | 3D View |
    |:---:|:-----|:-------:|
    | 1   | [`platfom_rplidar_a2.stl`](https://github.com/ros-mobile-robots/remo_description/blob/main/meshes/remo/deck/raspberry_pi_deck.stl) | <script src="https://embed.github.com/view/3d/ros-mobile-robots/remo_description/main/meshes/remo/deck/raspberry_pi_deck.stl"></script> |



=== "Jetson Nano Deck"

    | Qty | Part | 3D View |
    |:---:|:-----|:-------:|
    | 1   | [`jetson_nano_deck.stl`](https://github.com/ros-mobile-robots/remo_description/blob/main/meshes/remo/deck/jetson_nano_deck.stl) | <script src="https://embed.github.com/view/3d/ros-mobile-robots/remo_description/main/meshes/remo/deck/jetson_nano_deck.stl"></script> |


## LiDAR Platform


=== "RPLiDAR A2 M8"

    | Qty | Part | 3D View |
    |:---:|:-----|:-------:|
    | 1   | [`platfom_rplidar_a2.stl`](https://github.com/ros-mobile-robots/remo_description/blob/main/meshes/remo/lidar_platform/platfom_rplidar_a2.stl) | <script src="https://embed.github.com/view/3d/ros-mobile-robots/remo_description/main/meshes/remo/lidar_platform/platfom_rplidar_a2.stl"></script> |

    | Qty | Part | 3D View |
    |:---:|:-----|:-------:|
    | 1   | [`slamtec_holder.stl`](https://github.com/ros-mobile-robots/remo_description/blob/main/meshes/remo/lidar_platform/slamtec_holder.stl) | <script src="https://embed.github.com/view/3d/ros-mobile-robots/remo_description/main/meshes/remo/lidar_platform/slamtec_holder.stl"></script> |


=== "RPLiDAR A1 M8"

    To be announced.


## Camera Mount

| Qty | Part | 3D View |
|:---:|:-----|:-------:|
| 1   | [`camera_mount.stl`](https://github.com/ros-mobile-robots/remo_description/blob/main/meshes/remo/camera_mount/camera_mount.stl) | <script src="https://embed.github.com/view/3d/ros-mobile-robots/remo_description/main/meshes/remo/camera_mount/camera_mount.stl"></script> |


You can choose between the following camera adjustment mounts:

=== "Raspi Camera v2"

    | Qty | Part | 3D View |
    |:---:|:-----|:-------:|
    | 1   | [`Raspberry_pi_CAM_holder.stl`](https://github.com/ros-mobile-robots/remo_description/blob/main/meshes/remo/camera_mount/Raspberry_pi_CAM_holder.stl) | <script src="https://embed.github.com/view/3d/ros-mobile-robots/remo_description/main/meshes/remo/camera_mount/Raspberry_pi_CAM_holder.stl"></script> |

=== "OAK 1"

    | Qty | Part | 3D View |
    |:---:|:-----|:-------:|
    | 1   | [`OAK-1_adjustment_mount.stl`](https://github.com/ros-mobile-robots/remo_description/blob/main/meshes/remo/camera_mount/OAK-1_adjustment_mount.stl) | <script src="https://embed.github.com/view/3d/ros-mobile-robots/remo_description/main/meshes/remo/camera_mount/OAK-1_adjustment_mount.stl"></script> |

=== "OAK D"

    | Qty | Part | 3D View |
    |:---:|:-----|:-------:|
    | 1   | [`OAK-D_adjustment_mount.stl`](https://github.com/ros-mobile-robots/remo_description/blob/main/meshes/remo/camera_mount/OAK-D_adjustment_mount.stl) | <script src="https://embed.github.com/view/3d/ros-mobile-robots/remo_description/main/meshes/remo/camera_mount/OAK-D_adjustment_mount.stl"></script> |