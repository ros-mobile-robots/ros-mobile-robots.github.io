## ROS Network Setup

ROS is a distributed computing environment. This allows running compute expensive tasks such as visualization or path planning for navigation
on machines with more performance and sending goals to robots operating on less performant hardware like DiffBot with its Raspberry Pi 4 B.

For detailed instructions see [ROS Network Setup](http://wiki.ros.org/ROS/NetworkSetup) and the 
[ROS Environment Variables](http://wiki.ros.org/ROS/EnvironmentVariables).

The setup between the work machine that handles compute heavy tasks and DiffBot is as follows:

```mermaid
graph TB
  subgraph PC [Work machine: your PC]
    M[roscore:<br/>ROS master]
    C[RViz, SLAM,<br/>navigation]
  end
  subgraph ROBOT [Robot: Raspberry Pi]
    B[Bringup: drivers,<br/>hardware interface]
  end
  T[Teensy: motors<br/>and encoders]
  C -. 1. register, look up .-> M
  B -. 1. register, look up .-> M
  C ---|2. topics and services,<br/>directly, both ways| B
  B ---|USB, rosserial| T
```

The work machine runs the ROS master (`roscore`). Every node first registers with the master and asks it where the other nodes are (1). After that, the nodes send their topics and services directly to each other, in both directions (2). So the robot must be able to reach the work machine, and the work machine the robot: ROS needs "full bi-directional connectivity, on all ports" between them ([ROS NetworkSetup](http://wiki.ros.org/ROS/NetworkSetup)). Both kinds of traffic go over Wi-Fi or your LAN.

If the work machine runs the [dev container](../development/dev-container.md), the container shares the PC's network ([Network](../development/dev-container-internals.md#network)). On Windows with WSL 2, the robot can't reach it without extra setup, see [Work machine on Windows (WSL 2)](#work-machine-on-windows-wsl-2) below.


On DiffBot we configure the `ROS_MASTER_URI` to be the IP address of the work machine.

```console
export ROS_MASTER_URI=http://192.168.0.9:11311/
```

To test run the master on the work machine using the `roscore` command.
Then run the the `listener` from the `roscpp_tutorials` package in another terminal:

```console
fjp@workmachine:~/git/diffbot/ros$ rosrun roscpp_tutorials listener

```

Then switch to a terminal on DiffBot's Raspberry Pi and run the `talker` from `roscpp_tutorials`:

```console
fjp@diffbot:~/git/diffbot/ros$ rosrun roscpp_tutorials talker 
[ INFO] [1602018325.633133449]: hello world 0
[ INFO] [1602018325.733137152]: hello world 1
[ INFO] [1602018325.833112540]: hello world 2
[ INFO] [1602018325.933114483]: hello world 3
[ INFO] [1602018326.033114093]: hello world 4
[ INFO] [1602018326.133112684]: hello world 5
[ INFO] [1602018326.233112183]: hello world 6
[ INFO] [1602018326.333113126]: hello world 7
[ INFO] [1602018326.433113680]: hello world 8
[ INFO] [1602018326.533113031]: hello world 9
[ INFO] [1602018326.633110140]: hello world 10
[ INFO] [1602018326.733108954]: hello world 11
[ INFO] [1602018326.833113267]: hello world 12
[ INFO] [1602018326.933164505]: hello world 13
[ INFO] [1602018327.033119135]: hello world 14
[ INFO] [1602018327.133113559]: hello world 15
[ INFO] [1602018327.233111003]: hello world 16
[ INFO] [1602018327.333110705]: hello world 17
[ INFO] [1602018327.433126425]: hello world 18
[ INFO] [1602018327.533111498]: hello world 19
[ INFO] [1602018327.633107978]: hello world 20
[ INFO] [1602018327.733110736]: hello world 21
[ INFO] [1602018327.833107605]: hello world 22
[ INFO] [1602018327.933111659]: hello world 23
[ INFO] [1602018328.033108065]: hello world 24
[ INFO] [1602018328.133110379]: hello world 25
[ INFO] [1602018328.233150191]: hello world 26
[ INFO] [1602018328.333135986]: hello world 27
[ INFO] [1602018328.433153558]: hello world 28
[ INFO] [1602018328.533154557]: hello world 29
[ INFO] [1602018328.633151667]: hello world 30
[ INFO] [1602018328.733128777]: hello world 31
[ INFO] [1602018328.833170108]: hello world 32
[ INFO] [1602018328.933172402]: hello world 33
```

Looking back at the terminal on the work machine you should see the output:

```console
fjp.github.io git:(master) rosrun roscpp_tutorials listener
[ INFO] [1602018328.330070016]: I heard: [hello world 27]
[ INFO] [1602018328.430244670]: I heard: [hello world 28]
[ INFO] [1602018328.530173113]: I heard: [hello world 29]
[ INFO] [1602018328.630251690]: I heard: [hello world 30]
[ INFO] [1602018328.730334064]: I heard: [hello world 31]
[ INFO] [1602018328.830346566]: I heard: [hello world 32]
[ INFO] [1602018328.930009032]: I heard: [hello world 33]
```

Note that it can take some time receiving the messages from DiffBot on the work machine, which we can see from the time stamps in the outputs above.

### Work machine on Windows (WSL 2)

If your work machine runs Windows and ROS runs in WSL 2, for example in the [dev container](../development/dev-container.md), the robot usually can't connect to it.

**Why:** by default, WSL 2 runs behind network address translation (NAT). The Ubuntu in WSL gets its own private IP address, often `172.x.x.x`, which only Windows itself can reach ([WSL networking](https://learn.microsoft.com/en-us/windows/wsl/networking)). Connections from WSL to the robot work, but not from the robot to WSL. ROS 1 needs both directions: the robot's nodes connect to the master on your PC, and to your PC's nodes to receive their topics, for example the velocity commands from navigation. Forwarding single ports from Windows to WSL doesn't help either, because every ROS node listens on random ports.

**Fix: mirrored networking.** On Windows 11 22H2 or later, WSL can mirror Windows' network interfaces. The Ubuntu in WSL then has the same IP address as Windows, and devices on your network can connect to it directly ([mirrored mode](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking)). The dev container uses this network too, because it shares WSL's network.

1. In Windows, create or edit the file `%UserProfile%\.wslconfig` ([WSL settings](https://learn.microsoft.com/en-us/windows/wsl/wsl-config)):

    ```ini
    [wsl2]
    networkingMode=mirrored
    ```

2. Run `wsl --shutdown` in PowerShell and open Ubuntu again. `hostname -I` in Ubuntu should now show your PC's address on the network, for example `192.168.0.9`.
3. Allow the robot through the Hyper-V firewall, which filters connections to WSL. In PowerShell as administrator, with the robot's IP address:

    ```powershell
    New-NetFirewallHyperVRule -Name ROS-robot -DisplayName "ROS robot" `
      -Direction Inbound -Action Allow -RemoteAddresses 192.168.0.20 `
      -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
    ```

    This only lets the robot in. The `-VMCreatorId` is WSL's fixed ID ([Hyper-V firewall](https://learn.microsoft.com/en-us/windows/security/operating-system-security/network-security/windows-firewall/hyper-v-firewall)). Microsoft's [mirrored mode](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking) section also shows how to allow all incoming connections to WSL.

4. Tell ROS which addresses to use ([ROS environment variables](http://wiki.ros.org/ROS/EnvironmentVariables)):

    ```bash
    # On the PC, in the container
    export ROS_MASTER_URI=http://192.168.0.9:11311
    export ROS_IP=192.168.0.9

    # On the robot
    export ROS_MASTER_URI=http://192.168.0.9:11311
    export ROS_IP=192.168.0.20
    ```

5. Run the talker and listener test from above in both directions: the talker on the robot with the listener on the PC, and the talker on the PC with the listener on the robot. In both, the robot's node has to reach the master on your PC, so behind NAT neither works. The second direction also checks that the robot can connect to a node on your PC, on the random ports ROS picks for it.

!!! warning "Not tested with the robot yet"
    Mirrored networking is Microsoft's fix for exactly this problem, but nobody has tested this setup with DiffBot or Remo yet. If you try it, please tell us in the comments below whether it works.
