These are the instructions to set up Ubuntu 20.04 Focal Fossa on the robot's computer, a Raspberry Pi 4 B.

!!! note
    In case you don't have a Raspberry Pi 4 B, it is also possible to create the robot using a Jetson Nano from Nvidia.
    See the [related section in the documentation](./jetson-nano-setup.md).

## Ubuntu 20.04 image for the Raspberry Pi

ROS Noetic needs Ubuntu 20.04. Its standard support ended in May 2025 (see [ROS Setup](ros-setup.md)), and Raspberry Pi Imager no longer lists it. The official image from Ubuntu is still available: Ubuntu Server 20.04.5, without a desktop, which the robot doesn't need, because you work on it over SSH from your PC. The experimental [Robot Container](robot-container.md) runs Noetic on a current OS instead, Ubuntu Server 24.04.

Download `ubuntu-20.04.5-preinstalled-server-arm64+raspi.img.xz` from [Ubuntu's 20.04 release folder](https://cdimage.ubuntu.com/ubuntu/releases/20.04/release/), and compare its checksum with the `SHA256SUMS` file there.

!!! warning "Not yet tested with DiffBot and Remo"
    The robots were set up with Ubuntu MATE 20.04.1, which has a desktop. Ubuntu MATE no longer offers it for download (its support for 20.04 ended in April 2023); it's only in the archive on [releases.ubuntu-mate.org](https://releases.ubuntu-mate.org/). Ubuntu Server hasn't been tested with the robots' hardware yet: I2C, the serial ports, the LiDAR and the camera. If your robot already runs Ubuntu MATE 20.04.1, you don't need to reinstall it to follow these docs.

Flash the image to an empty microSD card with [Raspberry Pi Imager](https://www.raspberrypi.com/software/): under the operating system, choose **Use custom** and select the downloaded file. [balenaEtcher](https://etcher.balena.io/) works too.

!!! info
    [Flashing](https://en.wikipedia.org/wiki/Firmware#Flashing) in this context, means the transfer of software data, also referred to as Firmware, 
    from your computer to a device, such as a the sd card in this case. The term “to flash” comes from the Flash storage component of a 
    device where the Firmware is stored.

## First boot

Before the first boot, set up the Wi-Fi: on the card's `system-boot` partition, open `network-config` and remove the `#` at the start of the `wifis:` example's lines, down to the first access point. Replace `myhomewifi` with your network's name, keeping the colon, and `S3kr1t` with its password. Indent with spaces, not tabs. The image applies this file only at the first boot.

Insert the card and power up the Raspberry Pi. Log in with the user `ubuntu` and the password `ubuntu`, either over SSH (see below; the robot's IP address is in your router's list of devices) or with a monitor and keyboard. You have to choose a new password right away.

Changes to the boot configuration, such as the I2C lines on [Hardware Interfaces](hardware-interfaces.md), go into `usercfg.txt` on the `system-boot` partition (`/boot/firmware/usercfg.txt`): the image's `config.txt` says not to edit it. `dtparam=i2c_arm=on` is already set.

## Wifi Issues

**Country setting for 5 GHz networks:** outside the US, the Raspberry Pi may not connect to a 5 GHz network until it knows your country. On the `system-boot` partition, add `cfg80211.ieee80211_regdom=AT` to the end of the single line in `cmdline.txt`, with your country's code instead of `AT`, as the image's `network-config` describes.

??? note "Robots with Ubuntu MATE: Wi-Fi power save"
    So far there are no known issues using WiFi with Ubuntu MATE 20.04 arm64 on the Raspberry Pi 4B. Ubuntu MATE manages the Wi-Fi with NetworkManager; Ubuntu Server uses netplan instead, and we haven't tested whether it needs a power-save setting.

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

    Set this to `2` to turn power management off. 

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

## Instructions to install Wifi drivers

In case you use a Realtek USB Wifi dongle it might not be directly supported by the linux kernel.
To install the correct driver, you first have to figure out the driver id. When plugging in the USB Wifi dongle and running `dmesg` afterwards
should output something similar to:

```
[ 1430.931258] usb 1-1.3: new high-speed USB device number 13 using xhci_hcd
[ 1431.031913] usb 1-1.3: New USB device found, idVendor=0bda, idProduct=c811, bcdDevice= 2.00
[ 1431.031925] usb 1-1.3: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[ 1431.031932] usb 1-1.3: Product: 802.11ac NIC
[ 1431.031939] usb 1-1.3: Manufacturer: Realtek
```

With the `idVendor=0bda` and `idProduct=c811` you can search for the correct driver. In this case it is the

```
RTL8821cu 	USB 	0bda:c811 	Realtek 	WEP WPA WPA2
```

For it the instructions in the [`https://github.com/brektrou/rtl8821CU`](https://github.com/brektrou/rtl8821CU) repository can be used to install the driver.

!!! warning
    These instructions are specifically for the Realtek (id c811) USB wifi dongle. In case you use another USB dongle it might work directly or there exists
    a ubuntu package. In general try to use Google for instructions on how to install. 
    The following is only an example on what might be needed to get the driver  working.

After cloning this package, the Makefile needs to be adapted to work for the Raspberry Pi 4B:

```
###################### Platform Related #######################
CONFIG_PLATFORM_I386_PC = n
CONFIG_PLATFORM_ARM_RPI = n
CONFIG_PLATFORM_ARM64_RPI = y
```

Then follow the instructions from the README.md:

```
# For AArch64
sudo cp /lib/modules/$(uname -r)/build/arch/arm64/Makefile /lib/modules/$(uname -r)/build/arch/arm64/Makefile.$(date +%Y%m%d%H%M)
sudo sed -i 's/-mgeneral-regs-only//' /lib/modules/$(uname -r)/build/arch/arm64/Makefile
```

and finally make and install the driver:

```
make
sudo make
```

After a reboot the USB Wifi dongle should be detected and two wifi adapters should show up - the internal wifi module on the RPi and the USB dongle.

Sources

- [Disable power management in Stretch](https://www.raspberrypi.org/forums/viewtopic.php?t=194619)
- [How to turn off Wireless power management permanently](https://unix.stackexchange.com/a/315400/50268)
- [RPi4: WiFi client crashes often (brcmf_fw_crashed: Firmware has halted or crashed)](https://github.com/raspberrypi/linux/issues/3849)
- [WLAN Karten Realtek (german)](https://wiki.ubuntuusers.de/WLAN/Karten/Realtek/)


## Remote access with SSH

To work on the robot from your PC, without a monitor and keyboard on the robot, log in with SSH ([OpenSSH server](https://ubuntu.com/server/docs/how-to/security/openssh-server/)). The robot runs the SSH server, and your PC is the client. Both must be in the same network. On the robot, install the server (Ubuntu Server has it already):

```console
sudo apt install openssh-server
```

On your PC, in Linux or in the Ubuntu in WSL 2, the `ssh` client is usually already installed. Log in with your user name on the robot and its IP address, which `hostname -I` on the robot shows. For example:

```console
ssh your-user@192.168.0.20
```

Next, on the robot: install ROS Noetic, as described in [ROS Setup](ros-setup.md).
