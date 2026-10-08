# Welcome to DiffBot Documentation

This project guides you on how to build an autonomous two wheel differential drive robot. [![image](https://img.shields.io/github/stars/ros-mobile-robots/diffbot?style=social)](https://github.com/ros-mobile-robots/diffbot)
The robot can operate on a [Raspberry Pi 4 B](https://de.aliexpress.com/item/32858825148.html?spm=a2g0o.productlist.0.0.5d232e8bvlKM7l&algo_pvid=2c45d347-5783-49a6-a0a8-f104d0b78232&algo_expid=2c45d347-5783-49a6-a0a8-f104d0b78232-0&btsid=0100feb4-37d7-453a-8ff8-47a0e2fbdef7&ws_ab_test=searchweb0_0,searchweb201602_9,searchweb201603_52) or [NVIDIA Jetson Nano Developer Kit](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) 
running [ROS Noetic](http://wiki.ros.org/noetic) or [ROS Melodic](http://wiki.ros.org/melodic) middleware on Ubuntu Mate 20.04 and Ubuntu 18.04, respectively.
With a motor driver that actuates two brushed motors the robot can drive autonomously to a desired location while sensing its environment using sensors, 
such as a laser scanner to avoid obstacles and a camera to detect objects. Odometry wheel encoders (also referred to as speed sensors) 
combined with an inertial measurement unit (IMU) are used together with the laser scanner for localization in a previously stored map. 
Unseen environments can be mapped with the laser scanner, making use of open source SLAM algorithms such as `gmapping`. 

The following video gives an overview of the robot's components:

<iframe width="560" height="315" data-consent-src="https://www.youtube-nocookie.com/embed/6aAEbtfVbAk" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

The project is split into multiple parts, to address the following main aspects of the robot.

- [Bill of Materials (BOM)](./components.md) and the theory behind the parts.
- [Theory of (mobile) robots](./theory/index.md).
- [Assembly](./hardware_setup/assembly.md) of the robot platform and the components.
- Setup of ROS (Noetic or Melodic) on either Raspberry Pi 4 B or Jetson Nano, 
  which are both [Single Board Computers (SBC)](https://en.wikipedia.org/wiki/Single-board_computer) and are the brain of the robot.
- [Modeling the Robot](robot-description.md) in Blender and URDF to simulate it in Gazebo.
- ROS packages and nodes: 
  - Hardware drivers to interact with the hardware components
  - High level nodes for perception, navigation, localization and control.

Use the menu to learn more about the ROS packages and other components of the robot.

!!! note
    Using a [Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) instead of a Raspberry Pi is also possible.
    See the [Jetson Nano Setup section](processing_units/jetson-nano-setup.md) in this documentation for more details. 
    To run ROS Noetic [Docker](https://www.docker.com/) is needed.


## Source Code

The source code for this project can be found in the [ros-mobile-robots/diffbot](https://github.com/ros-mobile-robots/diffbot) GitHub repository.

## Remo Robot

You can find Remo robot (Research Education Modular/Mobile Open robot), a 3D printable and modular robot description package available at [ros-mobile-robots/remo_description](https://github.com/ros-mobile-robots/remo_description). 
The stl files are freely available from the repository and stored inside the git lfs (Git large file system) on GitHub. 
The bandwidth limit for open source projects on GitHub is 1.0 GB per month, 
which is why you might not be able to clone/pull the files because the quota is already exhausted this month. 
To support this work and in case you need the files immediately, you can access them through the following link:


<a class="gumroad-button" href="https://gumroad.com/l/GnMpU?wanted=true" data-gumroad-single-product="true">Access Remo STL files</a>

## Contributing

Contributions to the code and the documentation are welcome. The [Contributing](development/index.md) section explains how changes get in,
the best practices the project follows, testing and debugging, the CI checks, and how these docs are written.

## References

Helpful resources to bring your own robots into ROS are:

- Understand [ROS Concepts](https://wiki.ros.org/ROS/Concepts)
- Follow [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) such as [Using ROS on your custom Robot](http://wiki.ros.org/ROS/Tutorials#Using_ROS_on_your_custom_Robot)
- Books:
    - [*Mastering ROS for Robotics Programming: Best practices and troubleshooting solutions when working with ROS, 3rd Edition*][amazon_book_mastering_ros_ger] (affiliate link) this book contains also a chapter about about [Remo](packages/remo_description.md)
    - [*Introduction to Autonomous Robots (free book)*](https://github.com/Introduction-to-Autonomous-Robots/Introduction-to-Autonomous-Robots)
    - [**Robot Operating System (ROS) for Absolute Beginners**](https://link.springer.com/book/10.1007/978-1-4842-3405-1) from Apress by [Lentin Joseph](https://lentinjoseph.com/)
    - [**Programming Robots with ROS** A Practical Introduction to the Robot Operating System](http://shop.oreilly.com/product/0636920024736.do) from O'Reilly Media
    - [**Mastering ROS for Robotics Programming** Second Edition](https://www.packtpub.com/eu/hardware-and-creative/mastering-ros-robotics-programming-second-edition) from Packt
    - [**Elements of Robotics** Robots and Their Applications](https://www.springer.com/de/book/9783319625324) from Springer
    - [**ROS Robot Programming Book for Free!** Handbook from Robotis written by Turtlebot3 Developers](https://community.robotsource.org/t/download-the-ros-robot-programming-book-for-free/51)
- Courses:
    - [Robocademy](https://robocademy.com/)
    - [ROS Online Course for Beginner](https://discourse.ros.org/t/new-ros-online-course-for-beginner/5320)
    - [Udacity Robotics Software Engineer](https://www.udacity.com/course/robotics-software-engineer--nd209)
    - [Self-Driving Cars with Duckietown](https://www.edx.org/course/self-driving-cars-with-duckietown) by ETH Zurich
