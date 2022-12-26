## Simultaneous Localization and Mapping (SLAM) Overview 


There are several different types of Simultaneous Localization and Mapping (SLAM) algorithms, which can be classified based on the approach they use to estimate the pose and map of the robot or camera within an unknown environment. Some common types of SLAM algorithms include:

- Graph-based SLAM: These algorithms build a graph representation of the environment and use optimization techniques to estimate the pose and map of the robot. Graph-based SLAM algorithms are typically well-suited for environments with a high degree of structure and can be used with a variety of sensors. Examples include GTSAM, GraphSLAM, ORB-SLAM, Cartographer, and RTAB-Map.

- Kalman filter-based SLAM: These algorithms use a Kalman filter to estimate the pose and map of the robot in real-time. Kalman filter-based SLAM algorithms are typically well-suited for environments with a moderate degree of motion and can be used with a variety of sensors. An example of a Kalman filter-based SLAM algorithm is Okvis.

- Particle filter-based SLAM: These algorithms use a particle filter to estimate the pose and map of the robot in real-time. Particle filter-based SLAM algorithms are typically well-suited for dynamic environments and can be used with a variety of sensors. Examples of particle filter-based SLAM algorithms include FastSLAM and GMapping.

- Direct method-based SLAM: These algorithms use a direct method approach that does not require feature extraction or tracking. Direct method-based SLAM algorithms are typically well-suited for use in challenging environments and can be used with visual sensors. Examples of direct method-based SLAM algorithms include DSO and SVO.

- Feature-based SLAM: These algorithms use features in the environment to track the pose of the robot or camera. Feature-based SLAM algorithms are typically well-suited for use in structured environments and can be used with visual sensors. An example of a feature-based SLAM algorithm is ORB-SLAM.

Volumetric SLAM: These algorithms use a volumetric representation of the environment to track the pose of the robot or camera. Volumetric SLAM algorithms are typically well-suited for use in unstructured environments and can be used with visual. An example of a volumetric SLAM algorithm is ElasticFusion.

<figure markdown>
  ![Visual SLAM Roadmap](https://raw.githubusercontent.com/changh95/visual-slam-roadmap/main/img/getting-familiar.png){ width="300" }
  <figcaption markdown>Getting familiar with SLAM (https://github.com/changh95/visual-slam-roadmap)</figcaption>
  
</figure>

### Summary of SLAM libraries and algorithms

summary of the Simultaneous Localization and Mapping (SLAM) libraries and algorithms that I mentioned, with additional information about their approach and key features:

- [GTSAM](https://gtsam.org/): A library of algorithms and data structures for SLAM, implemented in C++ and designed for efficiency and scalability. GTSAM uses a graph-based optimization approach and includes a range of algorithms for different types of sensors and environments.

- [GMapping](https://openslam-org.github.io/gmapping.html): An open-source SLAM algorithm implemented in ROS, based on a particle filter and designed for use with laser scanners and odometry sensors. GMapping uses a Monte Carlo localization approach to create a map of the environment and determine the location of the robot.

- GraphSLAM: A library for implementing SLAM algorithms that use graph-based optimization, implemented in C++ and open-source. GraphSLAM algorithms build a graph representation of the environment and use optimization techniques to estimate the pose and map of the robot.

- FastSLAM: A real-time SLAM algorithm based on a particle filter, implemented in C++ and open-source. FastSLAM uses a Bayesian filtering approach to estimate the pose and map of the robot in real-time, and is well-suited for use in dynamic environments.

- Hector SLAM: A ROS-based SLAM library designed for use with laser scanners and odometry sensors, implemented in C++ and open-source. Hector SLAM uses a scan matching approach to create a map of the environment and determine the location of the robot.

- ORB-SLAM: A real-time SLAM library that uses visual information from cameras, implemented in C++ and open-source. ORB-SLAM is a feature-based SLAM algorithm that uses ORB features and a pose graph optimization approach to estimate the pose and map of the robot, a monocular, stereo, or RGB-D camera. It is based on the ORB (Oriented FAST and Rotated BRIEF) feature descriptor, which is fast to compute and has good performance in terms of repeatability and robustness to noise and partial occlusion. ORB-SLAM uses an efficient method for feature extraction and matching by combining the FAST corner detector with the ORB descriptor, and by using a fast and efficient matching algorithm based on the Hamming distance. This allows ORB-SLAM to extract and match features quickly and accurately, even in real-time applications with high frame rates. ORB-SLAM uses a sliding window optimization approach to estimate the camera's pose in real-time, and includes support for loop closure detection and mapping.

- OpenSLAM: A collection of open-source SLAM algorithms and tools, implemented in C++. OpenSLAM includes a range of SLAM algorithms for different sensors and environments, including graph-based, Kalman filter-based, and particle filter-based approaches.

- Cartographer: A real-time SLAM library developed by Google, designed for use with a variety of sensors, implemented in C++ and open-source. Cartographer uses a graph-based optimization approach and includes support for lidar, radar, and visual sensors.

- LeGO-LOAM: A real-time lidar-based SLAM algorithm for aerial robots and ground vehicles, implemented in C++ and open-source. LeGO-LOAM uses a scan matching and optimization approach to estimate the pose and map of the robot in real-time.

- LOAM: A real-time lidar-based SLAM algorithm for aerial robots and ground vehicles, implemented in C++ and open-source. LOAM uses a scan matching and optimization approach to estimate the pose and map of the robot in real-time.

- DSO: A real-time visual SLAM algorithm based on direct methods, implemented in C++ and open-source. DSO uses a direct method approach that does not require feature extraction or tracking, and is well-suited for use in challenging environments.

- RTAB-Map: A ROS-based SLAM library designed for use with RGB-D cameras and lidar sensors, implemented in C++ and open-source. RTAB-Map uses a graph-based optimization approach and includes support for loop closure detection.

- Okvis: A real-time visual SLAM algorithm based on a Kalman filter, implemented in C++ and open-source. Okvis uses a Kalman filter-based approach and includes support for multiple cameras and inertial sensors.

- SVO: A real-time visual SLAM algorithm based on a semi-direct approach, implemented in C++ and open-source. SVO uses a semi-direct method approach that does not require feature extraction or tracking, and is well-suited for use in challenging environments.

- VINS-Mono: A real-time visual SLAM algorithm for monocular cameras, implemented in C++ and open-source. VINS-Mono uses an optimization-based approach and includes support for inertial sensors.

- VINS-Fusion: A real-time visual SLAM algorithm for multiple cameras and sensors, implemented in C++ and open-source. VINS-Fusion uses an optimization-based approach and includes support for multiple cameras and inertial sensors.

- ElasticFusion: A real-time visual SLAM algorithm based on a volumetric representation of the environment, implemented in C++ and open-source. ElasticFusion uses a volumetric approach to represent the environment and track the camera's pose in real-time.


### Resources

#### Online

- [OpenSLAM.org](https://openslam-org.github.io/)
- [tzutalin/awesome-visual-slam](https://github.com/tzutalin/awesome-visual-slam)
- [changh95/visual-slam-roadmap](https://github.com/changh95/visual-slam-roadmap)

#### Books

- [Probabilistic Robotics, Sebastian Thrun, Wolfram Burgard, Dieter Fox](https://amzn.to/2ZdwgrN), [MIT Press](https://mitpress.mit.edu/books/probabilistic-robotics)

