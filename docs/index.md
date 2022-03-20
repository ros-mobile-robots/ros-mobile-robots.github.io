# Welcome to DiffBot Documentation

This project guides you on how to build an autonomous two wheel differential drive robot. [![image](https://img.shields.io/github/stars/ros-mobile-robots/diffbot?style=social)](https://github.com/ros-mobile-robots/diffbot)
The robot can operate on a [Raspberry Pi 4 B](https://de.aliexpress.com/item/32858825148.html?spm=a2g0o.productlist.0.0.5d232e8bvlKM7l&algo_pvid=2c45d347-5783-49a6-a0a8-f104d0b78232&algo_expid=2c45d347-5783-49a6-a0a8-f104d0b78232-0&btsid=0100feb4-37d7-453a-8ff8-47a0e2fbdef7&ws_ab_test=searchweb0_0,searchweb201602_9,searchweb201603_52) or [NVIDIA Jetson Nano Developer Kit](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) 
running [ROS Noetic](http://wiki.ros.org/noetic) or [ROS Melodic](http://wiki.ros.org/melodic) middleware on Ubuntu Mate 20.04 and Ubuntu 18.04, respectively.
With a motor driver that actuates two brushed motors the robot can drive autonomously to a desired location while sensing its environment using sensors, 
such as a laser scanner to avoid obstacles and a camera to detect objects. Odometry wheel encoders (also refered to as speed sensors) 
combined with an inertial measurement unit (IMU) are used together with the laser scanner for localization in a previously stored map. 
Unseen enviornments can be mapped with the laser scanner, making use of open source SLAM algorithms such as `gmapping`. 

The following video gives an overview of the robot's components:

<iframe width="560" height="315" src="https://www.youtube.com/embed/6aAEbtfVbAk" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

The project is split into multiple parts, to adress the following main aspects of the robot.

- [Bill of Materials (BOM)](./components.md) and the theory behind the parts.
- [Theory of (mobile) robots](./theory/index.md).
- [Assembly](./hardware_setup/assembly.md) of the robot platform and the components.
- Setup of ROS (Noetic or Melodic) on either Raspberry Pi 4 B or Jetson Nano, 
  which are both [Single Board Computers (SBC)](https://en.wikipedia.org/wiki/Single-board_computer) and are the brain of the robot.
- [Modeling the Robot](/projects/diffbot/URDF) in Blender and URDF to simulate it in Gazebo.
- ROS packages and nodes: 
  - Hardware drivers to interact with the hardware components
  - High level nodes for perception, navigation, localization and control.

Use the menu to learn more about the ROS packages and other components of the robot.

!!! note
    Using a [Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) instead of a Raspberry Pi is also possible.
    See the [Jetson Nano Setup section](./jetson-nano-setup.md) in this documentation for more details. 
    To run ROS Noetic [Docker](https://www.docker.com/) is needed.


## Source Code

The source code for this project can be found in the [ros-mobile-robots/diffbot](https://github.com/ros-mobile-robots/diffbot) GitHub repository.

## Remo Robot

You can find Remo robot (Research Education Modular/Mobile Open robot), a 3D printable and modular robot description package available at [ros-mobile-robots/remo_description](https://github.com/ros-mobile-robots/remo_description). 
The stl files are freely availabe from the repository and stored inside the git lfs (Git large file system) on GitHub. 
The bandwith limit for open source projects on GitHub is 1.0 GB per month, 
which is why you might not be able to clone/pull the files because the quota is already exhausted this month. 
To support this work and in case you need the files immediately, you can access them through the following link:


<a class="gumroad-button" href="https://gumroad.com/l/GnMpU?wanted=true" data-gumroad-single-product="true">Access Remo STL files</a>

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

## Testing, Debugging and CI

For a ROS catkin workspace explaining gTest and [rostest](http://wiki.ros.org/rostest) see [Ros-Test-Example](https://github.com/steup/Ros-Test-Example) and its [documentation](https://github.com/steup/Ros-Test-Example/blob/master/src/cars/doc/slides/slides.pdf).
To run tests with catkin-tools, see [Building and running tests](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html#building-and-running-tests).

To get a workspace that allows a debugger to stop at breakpoints, it is required to build the catkin workspace with Debug Symbols. 
For this the command `catkin build --save-config --cmake-args -DCMAKE_BUILD_TYPE=Debug` is used, mentioned in the [catkin-tools cheat sheet](https://catkin-tools.readthedocs.io/en/latest/cheat_sheet.html).


This repository makes use of automated builds when new code is pushed or a pull reuqest is made to this repository.
For this the Travis and GitHub actions configurations (yml files) from [ROS Industrial CI](https://github.com/ros-industrial/industrial_ci) are used.

## Documentation

The documentation is using [material design theme](https://squidfunk.github.io/mkdocs-material/), which is based on [MkDocs](https://www.mkdocs.org/).
Future code documentation will make use of [doxygen](http://wiki.ros.org/Doxygen) and [rosdoc_lite](http://wiki.ros.org/rosdoc_lite).

## References

Helpful resources to bring your own robots into ROS are:

- Understand [ROS Concepts](https://wiki.ros.org/ROS/Concepts)
- Follow [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) such as [Using ROS on your custom Robot](http://wiki.ros.org/ROS/Tutorials#Using_ROS_on_your_custom_Robot)
- Books:
    - [*Mastering ROS for Robotics Programming: Best practices and troubleshooting solutions when working with ROS, 3rd Edition*](https://amzn.to/3tpmT55) this book contains also a chapter about about [Remo](packages/remo_description/)
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
