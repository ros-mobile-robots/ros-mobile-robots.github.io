These are the instructions to setup the official Ubuntu 18.04 image from Nvidia on a Jetson Nano.

!!! note
    In case you don't have a Jetson Nano, it is also possible to create the robot using a Raspberry Pi 4 B.
    See the [related section in the documentation](./rpi-setup.md).

## Installing the JetPack SDK

The [JetPack](https://developer.nvidia.com/embedded/jetpack) SDK is a tool that installs the software tools and operating system for a Jetson Development Kit.

!!! note
    JetPack SDK includes the latest Jetson Linux Driver Package (L4T) with Linux operating system and CUDA-X accelerated libraries and APIs for Deep Learning, Computer Vision, Accelerated Computing and Multimedia. It also includes samples, documentation, and developer tools for both host computer and developer kit, and supports higher level SDKs such as DeepStream for streaming video analytics and Isaac for robotics.[^jetpack]

Start by downloading the 6 GB SD card image for the Jetson Nano Developer Kit 
from the [Nvidia Developer website](https://developer.nvidia.com/embedded/jetpack) ([direct download link](https://developer.nvidia.com/jetson-nano-sd-card-image)).
Alternatively you can get the Jetpack from the [Jetson Download Center](https://developer.nvidia.com/embedded/downloads). 

!!! note
    In case you want to use the [NVIDIA SDK Manager](https://developer.nvidia.com/nvsdk-manager) you need to be part of the NVIDIA Developer Program. 
    You can use this manager to configure the Jetpack image before flashing it.

Next use a tool to flash the image onto the SD card. One option is to use [balenaEtcher](https://www.balena.io/etcher/) that is available on Ubuntu, Mac and Windows.

!!! info
    [Flashing](https://en.wikipedia.org/wiki/Firmware#Flashing) in this context, means the transfer of software data, also refered to as Firmware, 
    from your computer to a device, such as a the sd card in this case. The term “to flash” comes from the Flash storage component of a 
    device where the Firmware is stored.

## Power Supply

There are four ways you can power the Jetson Nano.

1. Provide 2 Amps at 5 Volts to the micro USB connector
2. 4 Amps at 5 Volts to the barrel jack connector.
3. 5 Volts on the GPIO headers, where each of the two 5 Volt pins can handle up to 3 A. The Nano has two of these 5 Volt pins,
which means you could consume 6 A total at 5 V by having the two pins connected to a power supply that is capable of delivering 6 A at 5 V,
corresponding to 30 Watts.
4. [Power Over Ethernet (POE)](https://en.wikipedia.org/wiki/Power_over_Ethernet)

In its default power consumption state ('mode 0') the Nano itself uses 10 Watts, which is 2 A at 5 V.
This, however, is only for the Jetson Nano compute module and doesn't include the peripherals.
To provide power to peripherals, for example over the USB connectors, you need to operate the Nano itself in 5 Watts mode ('mode 1' or MAXN).
This way there are 5 Watts left for peripherals via the micro USB connector, that provides 2 A at 5 V.

The following table[^1] shows the power modes for the Jetson Nano predefined by NVIDIA and the associated caps on use of the module's resources.

| Property                           | MAXN[^2]             | 5W              |
|:----------------------------------:|:--------------------:|:---------------:|
| Power Budget                       | 10 Watts             | 5 Watts         |
| Mode ID                            | 0                    | 1               |
| Online CPU                         | 4                    | 2               |
| CPU Maximal Frequency (MHz)        | 1479                 | 918             |
| GPU TPC                            | 1                    | 1               |
| GPU Maximal Frequency (MHz)        | 921.6                | 640             |
| Memory Maximal Frequency (MHz)     | 1600                 | 1600            |

| SOC clocks maximal frequency (MHz) | All modes     |             |             |
|:---------------------------------:|:--------------:|:-----------:|:-----------:|
|                                    | adsp 844.8    | csi 750     | se 627.2    |
|                                    | ape 499.2     | nvdec 716.8 | tsec 408    |
|                                    | host1x 408    | nvenc 716.8 | tsecb 627.2 |
|                                    | isp 793.6     | nvjpg 627.2 | vi 793.6    |
|                                    | display 665.6 | pcie 500    | vic03 627.2 |

  

!!! note 
    In practice, when running from the micro-USB connector, we should be running in 5V mode to power the rest of the sensors, like the laser scanner.
    The drawback is that this will slow down the computations on the Jetson Nano.
    See the section below on how to do that.


To develop and test code that requires high power, it is convenient to use the second option. 
To connect the 4 Amps @ 5 Volts barrel jack connector to the Jetson Nano a Jumper is required.

!!! note
    Make sure to use a 5 V 4 Amps switching power supply. 
    For example the [Mean Well GST25E05-P1J](https://amzn.to/3sFAOQx) or the [AC/DC Desktop Adapter 5 V from Adafruit](https://www.adafruit.com/product/1466).
    DC barrel jack 5.5 mm OD / 2.1 mm ID / 9.5 mm length, center pin positive.

Connect the jumper on J48. J48 is located between the Barrel Jack connector and the Camera connector. 
This jumper tells the Nano to use the Barrel Jack instead of the micro-USB port. 
Then plug the power supply into the Barrel Jack, and the Nano boots.

TODO add image of Jumper location

!!! note
    There are two variants of the Jetson Nano. The older A02 and the revised B01.
    Depending on the variant the location of the jumper is slightly different.
    - For an A02 carrier board (pre-2020) J48 is the solo header next to the camera port.
    - For a B01 carrier board (2020+, has two camera ports) J48 is a solo header behind the barrel jack and the HDMI port.


## Prepare Ubuntu

After flashing the image to the sd card insert it to the Jetson Nano, hook it up to a monitor via HDMI and power it up by pluggin in the micro USB or even better barrel jack connector (don't forget the jumper on J48). The follow the instructions on the screen to setup Ubuntu 18.04 Bionic Beaver.
For this you will need to accept the Nvidia End User License Agreement, set the desired language, keyboard, time zone, login credentials, APP Partition size (choose max possible), 
and delete unused bootloader that is done automatically with the new QSPI image (MaxSPI) of Jetpack 4.5. This QSPI update will take about two mins.
Finally select the NVPModel mode explained [above](./jetson-nano-setup.md#power-supply). First we go with the default MAXN 10 Watts mode ('mode 0').
We will changed this later at runtime - when powering the robot over the power bank - using the nvpmodel GUI or nvpmodel command line utility. 
Refer to the NVIDIA Jetson Linux Developer Guide for further information. 

Once finished, follow the next steps to install ROS Melodic.

!!! note
    To proceed with the next steps on installing ROS and other related dependencies you can run a bash script.
    Just clone this repository: https://github.com/fjp/diffbot.git and run `ubuntu-setup.sh`. But to learn more, you should follow the instructions on the following pages.



## Resources

- [JetPack SDK](https://developer.nvidia.com/embedded/jetpack)
- [Jetson Nano – Use More Power! - Jetson Hacks](https://www.jetsonhacks.com/2019/04/10/jetson-nano-use-more-power/)

[^jetpack]: https://developer.nvidia.com/embedded/jetpack
[^1]: [Nvidia Jetson Nano: Supported Modes and Power Efficiency](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/power_management_nano.html#wwpID0E0FL0HA)
[^2]: The default mode is MAXN (power budget 10 watts, mode ID 0).