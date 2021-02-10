# Welcome to DiffBot Documentation

This project guides you on how to build an autonomous two wheel differential drive robot. [![image](https://img.shields.io/github/stars/fjp/diffbot?style=social)](https://github.com/fjp/diffbot)
The robot is equipped with a [Raspberry Pi 4 B](https://de.aliexpress.com/item/32858825148.html?spm=a2g0o.productlist.0.0.5d232e8bvlKM7l&algo_pvid=2c45d347-5783-49a6-a0a8-f104d0b78232&algo_expid=2c45d347-5783-49a6-a0a8-f104d0b78232-0&btsid=0100feb4-37d7-453a-8ff8-47a0e2fbdef7&ws_ab_test=searchweb0_0,searchweb201602_9,searchweb201603_52) running [ROS Noetic](http://wiki.ros.org/noetic) middleware on Ubuntu Mate 20.04.
With a motor driver and two actuators it can drive autonomously to a desired location while sensing its environment using sensors, 
such as a camera and an ultrasonic ranger to avoid obstacles. Speed sensors combined with an inertial measurement unit (IMU) are used for localization.
The project is split into multiple parts, to adress the following main aspects of the robot.

- [Part list](/projects/diffbot/components/) and the theory behind the parts.
- [Assembly](/projects/diffbot/assembly/) of the robot platform and the components.
- Raspberry Pi 4 B setup using ROS Noetic, which will be the brain of the robot.
- [Modeling the Robot](/projects/diffbot/URDF) in Blender and URDF to simulate it in Gazebo.
- ROS packages and nodes: 
  - Hardware drivers to interact with the hardware components
  - High level nodes for perception, navigation, localization and control.

Use the menu on the left to learn more about the ROS packages and other components of the robot.

!!! note
    Using a [Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) instead of a Raspberry Pi is also possible.


## Source Code

The source code for this project can be found in [this GitHub repository](https://github.com/fjp/diffbot).

## Best Practices and REP

The project tries to follow the [ROS best practices](http://wiki.ros.org/Tutorials/Best%20Practices) as good as possible. 
This includes examples and patterns on producing and contributing high quality code, 
as well as on testing, and other quality oriented practices, like continuous integration. 
You can read more about it on the [ROS Quality wiki](http://wiki.ros.org/Quality). This includes also following the advices given
in the [ROS Enhancement Proposals (REPs)](https://www.ros.org/reps/rep-0000.html). Throughout the documentation links to corresponding REPs are given.

The wiki section [ROS developer's guide](http://wiki.ros.org/DevelopersGuide) is a good starting point for getting used to the common practices for developing components to be shared with the community. It includes links to [naming conventions](http://wiki.ros.org/ROS/Patterns/Conventions#Naming_ROS_Resources) (e.g. for packages) and ROS [C++](http://wiki.ros.org/CppStyleGuide) and [Python](http://wiki.ros.org/PyStyleGuide) style guides.

Other good resources to learn more about ROS best practices is the [Autonomous Systems Lab](https://github.com/ethz-asl/ros_best_practices/wiki) of ETH Zurich.

!!! Note
    Your contributions to the code or documentation are most welcome but please try to follow the mentioned best pratices where possible.

## Testing and CI

This repository makes use of automated builds when new code is pushed or a pull reuqest is made to this repository.
For this the Travis and GitHub actions configurations (yml files) from [ROS Industrial CI](https://github.com/ros-industrial/industrial_ci) are used.

## References

Helpful resources to bring your own robots into ROS are:

- Understand [ROS Concepts](https://wiki.ros.org/ROS/Concepts)
- Follow [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) such as [Using ROS on your custom Robot](http://wiki.ros.org/ROS/Tutorials#Using_ROS_on_your_custom_Robot)
- Books:
  - [**Robot Operating System (ROS) for Absolute Beginners**](https://link.springer.com/book/10.1007/978-1-4842-3405-1) from Apress by [Lentin Joseph](https://lentinjoseph.com/)
  - [**Programming Robots with ROS** A Practical Introduction to the Robot Operating System](http://shop.oreilly.com/product/0636920024736.do) from O'Reilly Media
  - [**Mastering ROS for Robotics Programming** Second Edition](https://www.packtpub.com/eu/hardware-and-creative/mastering-ros-robotics-programming-second-edition) from Packt
  - [**Elements of Robotics** Robots and Their Applications](https://www.springer.com/de/book/9783319625324) from Springer
