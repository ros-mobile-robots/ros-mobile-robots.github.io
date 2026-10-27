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
    * Motion models (velocity and odometry)

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
    - *Probabilistic Robotics* by Sebastian Thrun, Wolfram Burgard and Dieter Fox, MIT Press 2005 ([Amazon.com][amazon_book_probabilistic_robotics_us], [Amazon.de][amazon_book_probabilistic_robotics_ger], affiliate links): the standard book on state estimation, localization and SLAM for mobile robots. [Probability](preliminaries/probability.md), [Recursive State Estimation](state-estimation/recursive-state-estimation.md) (chapter 2), [Nonparametric Filters](state-estimation/nonparametric-filters.md) (chapter 4), [Robot Motion](state-estimation/robot-motion.md) (chapter 5), [Markov and Gaussian Localization](localization/markov-and-gaussian-localization.md) (chapter 7) and [SLAM Overview](slam/slam-overview.md) cite it. Robot Motion shows where DiffBot's ROS packages use its two motion models: diff_drive_controller's odometry follows the velocity motion model, and amcl moves its particles with the odometry motion model.
    - *Feedback Systems: An Introduction for Scientists and Engineers* by Karl Johan Åström and Richard M. Murray, 2nd edition, Princeton University Press 2021 ([free online at fbsbook.org](https://fbsbook.org); [Amazon.com][amazon_book_feedback_systems_us], [Amazon.de][amazon_book_feedback_systems_ger], affiliate links): feedback, stability and PID control. [Wheel Dynamics in Simulation](modeling-control/wheel-dynamics-simulation.md) cites chapter 11 for the PID and integral windup.
    - *Modern Robotics: Mechanics, Planning, and Control* by Kevin M. Lynch and Frank C. Park, Cambridge University Press 2017 ([free preprint and videos at modernrobotics.org](http://modernrobotics.org); [Amazon.com][amazon_book_modern_robotics_us], [Amazon.de][amazon_book_modern_robotics_ger], affiliate links): rigid-body motion and dynamics, and wheeled mobile robots in chapter 13. [Wheel Dynamics in Simulation](modeling-control/wheel-dynamics-simulation.md) cites section 8.2 for a wheel's inertia.
- **Courses:**
    - [Self Driving cars with Duckietown edX MOOC](https://www.edx.org/course/self-driving-cars-with-duckietown)
    - [Uni Freiburg SLAM Course, Robot Mapping - WS 2013/14](http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/)
