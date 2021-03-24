These are the instructions to setup a custom Ubuntu 20.04 Focal Fossa on Raspberry Pi 4 B.

## Obtain Ubuntu 20.04 Mate Image for Raspberry Pi

To install the long term supported (LTS) Ubuntu 20.04 on the Raspberry Pi 4B we make use of [arm64 version](https://ubuntu-mate.org/download/arm64/focal/) 
of [Ubuntu Mate](https://ubuntu-mate.org/). 

Download the latest release of the image and flash it to an empty sd card. To do this follow the instructions on the 
[Raspberry Pi documentation](https://www.raspberrypi.org/documentation/installation/installing-images/) or 
[balenaEtcher](https://www.balena.io/etcher/). 
Another way is to use the [Raspberry Pi Imager](https://www.raspberrypi.org/downloads/) explained [here](https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/).

!!! info
    [Flashing](https://en.wikipedia.org/wiki/Firmware#Flashing) in this context, means the transfer of software data, also refered to as Firmware, 
    from your computer to a device, such as a the sd card in this case. The term “to flash” comes from the Flash storage component of a 
    device where the Firmware is stored.

## Wifi Issues

So far there are no known issues using WiFi with Ubuntu Mate 20.04 arm64 on the Raspberry Pi 4B.

<details>
  <summary>Possible issues with other images</summary>
  
  If you are not in the US it is possible that you encounter connection problems when connected to a 5Ghz Wifi network.
  If you are in a different country than the US you need to update your regulatory country. 5Ghz needs this to know the right bands to use.

This can be changed by editing the value of `REGDOMAIN` in the file `/etc/default/crda` ([Central Regulatory Domain Agent](https://wireless.wiki.kernel.org/en/developers/regulatory/crda)) to the code for your country [ref](https://github.com/TheRemote/Ubuntu-Server-raspi4-unofficial/issues/98).
</details>



## Prepare Ubuntu

After flashing the image to the sd card insert it to the Pi, hook it up to a monitor via HDMI and power it up by pluggin in the USB-C connector.
Then you should follow the [installation instructions](https://ubuntu-mate.org/raspberry-pi/install/) on the screen.

Once finished, follow the next steps to install ROS Noetic.

!!! note
    To proceed with the next steps on installing ROS and other related dependencies you can run a bash script.
  Just clone this repository: https://github.com/fjp/diffbot.git and run `ubuntu-setup.sh`. But to learn more, you should follow the instructions on the following pages.
