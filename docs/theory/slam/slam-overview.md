## Simultaneous Localization and Mapping (SLAM) Overview

Simultaneous Localization and Mapping (SLAM) is a field of robotics and computer vision that is concerned with the problem of building a map of an unknown environment while simultaneously estimating the pose (position and orientation) of a robot or camera within that environment. SLAM algorithms typically use data from sensors such as cameras, LIDARs, or inertial measurement units (IMUs) to estimate the pose and build a map of the environment in real-time as the robot or camera moves through the environment.

SLAM algorithms have a wide range of applications, including autonomous navigation and exploration, augmented reality, and 3D modeling. They are used in a variety of robotic platforms, including ground vehicles, aerial vehicles, and mobile robots.

There are several different approaches to solving the SLAM problem, including graph-based SLAM, Kalman filter-based SLAM, particle filter-based SLAM, and direct method-based SLAM.

### Frontend-backend architecture

In Simultaneous Localization and Mapping (SLAM), it is common to divide the overall SLAM system into a frontend and a backend. The frontend is responsible for processing the raw measurements from the sensors, such as cameras, LIDARs, or inertial measurement units (IMUs), and extracting features or other relevant information. The backend is responsible for estimating the pose and structure of the environment based on the processed measurements and any additional constraints.

The frontend typically includes tasks such as feature extraction, feature matching, and initial pose estimation, while the backend includes tasks such as graph optimization, pose graph optimization, or bundle adjustment. The frontend and backend may operate in a loop, with the frontend providing new measurements to the backend, and the backend updating the estimates of the poses and structures based on the new measurements.

The frontend and backend are often implemented separately in SLAM systems, as this allows for more flexibility in terms of the specific algorithms and techniques that are used for each task. It also allows for the separation of the real-time processing tasks (such as feature extraction and matching) from the more computationally intensive optimization tasks (such as graph optimization or bundle adjustment).

The frontend and backend of a SLAM system may be implemented as separate modules or components, and may communicate with each other through a common interface or API. In some cases, the frontend and backend may be implemented as part of a single monolithic SLAM system, with the different tasks being integrated into a single codebase.

### Types of SLAM algorithms


There are several different types of Simultaneous Localization and Mapping (SLAM) algorithms, which can be classified based on the approach they use to estimate the pose and map of the robot or camera within an unknown environment. Some common types of SLAM algorithms include:

- Graph-based SLAM: These algorithms store robot poses (and landmarks) as nodes of a graph and measurements as constraints between them, then solve for all of them at once with nonlinear least squares. Loop closures add constraints that correct the drift accumulated along the way. Examples include GraphSLAM, Cartographer and RTAB-Map; GTSAM and g2o are libraries for the optimization.

- Kalman filter-based SLAM: These algorithms keep a single Gaussian estimate of the current pose and the map, or of a window of recent poses, and update it with every measurement. In EKF SLAM the covariance matrix grows quadratically with the number of landmarks, which limits the map size. Examples are EKF SLAM and, for visual-inertial odometry, MSCKF and ROVIO.

- Particle filter-based SLAM: These algorithms sample possible robot trajectories with a particle filter; each particle carries its own map, conditioned on its path (Rao-Blackwellization). This lets several hypotheses about the trajectory exist side by side. Like most SLAM methods, they assume a static world; moving objects need extra handling. Examples include FastSLAM and GMapping.

- Direct method-based SLAM: These algorithms estimate the camera motion by minimizing the photometric error of pixel intensities directly, instead of extracting and matching features. They can use image regions with little texture that feature-based methods ignore, but they are sensitive to exposure changes and need a good initial estimate. Examples include LSD-SLAM and DSO; SVO is semi-direct.

- Feature-based SLAM: These algorithms extract keypoints with descriptors, for example ORB, and match them between frames; the matches feed pose estimation and bundle adjustment. Matching copes with larger motions and lighting changes, but needs textured scenes. An example is ORB-SLAM.

- Dense SLAM: These algorithms build a dense map of the environment, for example a volumetric (TSDF) or surfel map, usually from RGB-D cameras. Examples are KinectFusion and ElasticFusion.

<figure markdown>
  ![Visual SLAM Roadmap](https://raw.githubusercontent.com/changh95/visual-slam-roadmap/main/img/getting-familiar.png){ width="300" }
  <figcaption markdown>Getting familiar with SLAM (https://github.com/changh95/visual-slam-roadmap)</figcaption>
  
</figure>

### Summary of SLAM libraries and algorithms

The SLAM libraries and algorithms above, with their approach and key features:

- [GTSAM](https://gtsam.org/): A library of algorithms and data structures for SLAM, implemented in C++ and designed for efficiency and scalability. GTSAM uses a graph-based optimization approach and includes a range of algorithms for different types of sensors and environments.

- [GMapping](https://openslam-org.github.io/gmapping.html): Laser-based SLAM with a Rao-Blackwellized particle filter: each particle carries its own occupancy grid map. It needs laser scans and odometry and is available in ROS as `slam_gmapping`.

- GraphSLAM: An algorithm (Thrun and Montemerlo, 2006) that stores robot poses and measurements as a graph of constraints and solves for the trajectory and map with nonlinear least squares. Libraries such as GTSAM and g2o solve this kind of problem.

- FastSLAM: An algorithm (Montemerlo et al., 2002) that uses a Rao-Blackwellized particle filter: the particles sample the robot's path, and each particle keeps its own landmark estimates in small Kalman filters.

- [Hector SLAM](https://github.com/tu-darmstadt-ros-pkg/hector_slam): Laser-based SLAM for ROS that matches scans against the map. It doesn't need wheel odometry, which makes it useful for handheld or flying platforms.

- [ORB-SLAM](https://github.com/UZ-SLAMLab/ORB_SLAM3): Feature-based visual SLAM for monocular, stereo and RGB-D cameras; ORB-SLAM3 adds visual-inertial modes. It tracks ORB features (FAST corners with a binary descriptor), refines keyframes with local bundle adjustment and closes loops with a pose graph.

- [OpenSLAM](https://openslam-org.github.io/): A website that collects open-source SLAM implementations from many research groups, including GMapping.

- [Cartographer](https://github.com/cartographer-project/cartographer): Real-time SLAM from Google for 2D and 3D lidar. Odometry is optional; an IMU is optional in 2D and required in 3D. It builds local submaps and optimizes them in a pose graph with loop closure.

- [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM): Lightweight, ground-optimized LOAM for ground vehicles: it segments the ground plane and runs in real time on embedded computers.

- LOAM: Lidar odometry and mapping (Zhang and Singh, 2014). It matches edge and planar features between scans: fast odometry at a high rate, plus slower, more accurate mapping.

- [DSO](https://github.com/JakobEngel/dso): Direct Sparse Odometry, a visual odometry that minimizes the photometric error of image pixels instead of matching features.

- RTAB-Map: A ROS-based SLAM library designed for use with RGB-D cameras and lidar sensors, implemented in C++ and open-source. RTAB-Map uses a graph-based optimization approach and includes support for loop closure detection.

- [OKVIS](https://github.com/ethz-asl/okvis): Keyframe-based visual-inertial odometry for stereo or multi-camera rigs with an IMU, using nonlinear optimization over a sliding window of keyframes.

- [SVO](https://github.com/uzh-rpg/rpg_svo): Semi-direct visual odometry: direct image alignment for motion estimation, plus features for mapping. It is very fast, which suits drones.

- [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono): Monocular visual-inertial SLAM with sliding-window optimization, relocalization and loop closure.

- [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion): VINS-Mono extended to stereo cameras, with optional IMU and GPS fusion.

- [ElasticFusion](https://github.com/mp3guy/ElasticFusion): Dense RGB-D SLAM that builds a surfel map and corrects it with non-rigid deformation instead of a pose graph.


| Algorithm      | Type                                  | Source code / website                                    |
|----------------|---------------------------------------|----------------------------------------------------------|
| GTSAM          | Graph optimization library            | https://github.com/borglab/gtsam                         |
| GraphSLAM      | Graph-based SLAM (algorithm)          | Probabilistic Robotics, chapter 11                       |
| ORB-SLAM       | Feature-based visual SLAM             | https://github.com/UZ-SLAMLab/ORB_SLAM3                  |
| Cartographer   | Graph-based lidar SLAM                | https://github.com/cartographer-project/cartographer     |
| RTAB-Map       | Graph-based SLAM                      | https://introlab.github.io/rtabmap/                      |
| OKVIS          | Visual-inertial odometry (optimization) | https://github.com/ethz-asl/okvis                      |
| FastSLAM       | Particle filter-based SLAM (algorithm) | Probabilistic Robotics, chapter 13                      |
| GMapping       | Particle filter-based SLAM            | https://openslam-org.github.io/gmapping.html             |
| DSO            | Direct visual odometry                | https://github.com/JakobEngel/dso                        |
| SVO            | Semi-direct visual odometry           | https://github.com/uzh-rpg/rpg_svo                       |
| LSD-SLAM       | Direct visual SLAM                    | https://github.com/tum-vision/lsd_slam                   |
| PTAM           | Feature-based visual SLAM             | https://github.com/Oxford-PTAM/PTAM-GPL                  |
| DTAM           | Dense direct visual SLAM (algorithm)  | Newcombe, Lovegrove and Davison, ICCV 2011               |
| VINS-Mono      | Visual-inertial SLAM (optimization)   | https://github.com/HKUST-Aerial-Robotics/VINS-Mono       |
| VINS-Fusion    | Visual-inertial SLAM (optimization)   | https://github.com/HKUST-Aerial-Robotics/VINS-Fusion     |
| ElasticFusion  | Dense RGB-D SLAM (surfels)            | https://github.com/mp3guy/ElasticFusion                  |




### Resources

#### Online

- [OpenSLAM.org](https://openslam-org.github.io/)
- [tzutalin/awesome-visual-slam](https://github.com/tzutalin/awesome-visual-slam)
- [changh95/visual-slam-roadmap](https://github.com/changh95/visual-slam-roadmap)

#### Books

- [Probabilistic Robotics, Sebastian Thrun, Wolfram Burgard, Dieter Fox][amazon_book_probabilistic_robotics_ger] (affiliate link), [MIT Press](https://mitpress.mit.edu/books/probabilistic-robotics)

