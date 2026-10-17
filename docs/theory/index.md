# Theory behind Robotics

The following pages will provide an overview of the following important topics of robotics:
For more details be sure to check out the [Self Driving cars with Duckietown edX MOOC](https://www.edx.org/course/self-driving-cars-with-duckietown).
Also don't forget to read through the resources listed on the [main page](../index.md#references) of this documentation and consult your robotics
engineering text books of choice.

- Modeling and Control

    * Introduction to control systems
    * Representations and models
    * PID control

- Robot Vision

    * Introduction to projective geometry
    * Camera modeling and calibration
    * Image processing

- Object Detection

    * Convolutional neural networks
    * One and two stage object detection

- State Estimation and Localization

    * Bayes filtering framework
    * Parameterized methods (Kalman filter)
    * Sampling-based methods (Particle and histogram filter)

- Planning I

    * Planning formalization
    * Graphs

- Planning II

    * Probabilistic roadmaps
    * Sampling-based planning

- Learning by Reinforcement

    * Markov decision processes
    * Value functions
    * Policy gradients
    * Domain randomization

- Learning by Imitation

    * Behaviour cloning
    * Online imitation learning
    * Safety and uncertainty



## Resources

- **Books:**
    - *Probabilistic Robotics* by Sebastian Thrun, Wolfram Burgard and Dieter Fox, MIT Press 2005 ([Amazon.com][amazon_book_probabilistic_robotics_us], [Amazon.de][amazon_book_probabilistic_robotics_ger], affiliate links): the standard book on state estimation, localization and SLAM for mobile robots. [Probability](preliminaries/probability.md), [Recursive State Estimation](state-estimation/recursive-state-estimation.md) (chapter 2), [Nonparametric Filters](state-estimation/nonparametric-filters.md) (chapter 4), [Markov and Gaussian Localization](localization/markov-and-gaussian-localization.md) (chapter 7) and [SLAM Overview](slam/slam-overview.md) cite it. The [Simulation Test](../development/simulation-test.md#code-walkthrough) describes the robot's motion with its odometry motion model.
- **Courses:**
    - [Self Driving cars with Duckietown edX MOOC](https://www.edx.org/course/self-driving-cars-with-duckietown)
    - [Uni Freiburg SLAM Course, Robot Mapping - WS 2013/14](http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/)
