These are the instructions to setup a custom Ubuntu 20.04 Focal Fossa on Raspberry Pi 4 B.

!!! note
    In case you don't have a Raspberry Pi 4 B, it is also possible to create the robot using a Jetson Nano from Nvidia.
    See the [related section in the documentation](./jetson-nano-setup.md).

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

There might be some wifi issues where the connection is lost after some time. This might be a firmware issue reported [here](https://github.com/raspberrypi/linux/issues/3849).
Although the wifi power save management is turned off by default, make sure to also do this for external wifi usb dongles by editing the following file

```
sudo vim /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf
```

It contains:

```
[connection]
wifi.powersave = 3
```

Set this to `2`. 

Possible values for the `wifi.powersave` field are:

```
NM_SETTING_WIRELESS_POWERSAVE_DEFAULT (0): use the default value
NM_SETTING_WIRELESS_POWERSAVE_IGNORE  (1): don't touch existing setting
NM_SETTING_WIRELESS_POWERSAVE_DISABLE (2): disable powersave
NM_SETTING_WIRELESS_POWERSAVE_ENABLE  (3): enable powersave
```

(Informal source on [GitHub](https://gist.github.com/jcberthon/ea8cfe278998968ba7c5a95344bc8b55) for these values.)

You can check if it is turned off with `iwconfig`:

```
wlan0     IEEE 802.11  ESSID:"wifiname"  
          Mode:Managed  Frequency:5.18 GHz  Access Point: E0:28:6D:48:33:5D   
          Bit Rate=433.3 Mb/s   Tx-Power=31 dBm   
          Retry short limit:7   RTS thr:off   Fragment thr:off
          Power Management:off
          Link Quality=70/70  Signal level=-27 dBm  
          Rx invalid nwid:0  Rx invalid crypt:0  Rx invalid frag:0
          Tx excessive retries:1  Invalid misc:0   Missed beacon:0

lo        no wireless extensions.

wlan1     IEEE 802.11AC  ESSID:"wifiname"  Nickname:"WIFI@RTL8821CU"
          Mode:Managed  Frequency:5.18 GHz  Access Point: E0:28:6D:48:33:5D   
          Bit Rate:434 Mb/s   Sensitivity:0/0  
          Retry:off   RTS thr:off   Fragment thr:off
          Power Management:off
          Link Quality=64/100  Signal level=-26 dBm  Noise level=0 dBm
          Rx invalid nwid:0  Rx invalid crypt:0  Rx invalid frag:0
          Tx excessive retries:0  Invalid misc:0   Missed beacon:0

eth0      no wireless extensions.
```

This is the output when there is one external usb WiFi dongle connected to the Raspberry Pi 4 B and no ethernet cable.


Sources

- https://www.raspberrypi.org/forums/viewtopic.php?t=194619
- https://unix.stackexchange.com/a/315400/50268
- https://github.com/raspberrypi/linux/issues/3849


## Prepare Ubuntu

After flashing the image to the sd card insert it to the Pi, hook it up to a monitor via HDMI and power it up by pluggin in the USB-C connector.
Then you should follow the [installation instructions](https://ubuntu-mate.org/raspberry-pi/install/) on the screen.

Once finished, follow the next steps to install ROS Noetic.

!!! note
    To proceed with the next steps on installing ROS and other related dependencies you can run a bash script.
    Just clone this repository: https://github.com/fjp/diffbot.git and run `ubuntu-setup.sh`. But to learn more, you should follow the instructions on the following pages.
