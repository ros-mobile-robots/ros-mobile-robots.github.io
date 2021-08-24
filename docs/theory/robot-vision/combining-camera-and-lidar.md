## Combining Camera and LiDAR

The main problem with a single-sensor approach is its reduced reliability as each sensor has its weaknesses under certain situations.
To improve the tracking process results we can make use of both, camera and LiDAR sensors, and can combine the data from these sensors.

The first step in the fusion process will be to combine the tracked feature points within the camera images with the 3D Lidar points. 
For this we use homogeneous coordinates and the transformation matrices related to cameras to geometrically project the LiDAR points 
into the camera in such a way that we know the position of each 3D Lidar point on the image sensor.


### Project LiDAR Points into the Camera Image

The basic steps to project LiDAR points into the camera image are the following:

0. Preparation
   a. Make sure to synchronize all your LiDAR measurements to the exact recording timestamp of the camrea.
   b. Typically the camera is moving, which is why we need to compensate for this motion artifact. This is because a typical laser takes some short amount of time
   to scan the the entire view around it (usually 100 milliseconds). In this time the robot traveled a bit, which needs to be taken into account,
   so that the LiDAR point falls into the correct location in the image plane.
3. Convert LiDAR points into homogeneous coordinates.
4. Map (transform) all those points onto the image plane using the intrinsic camera matrix and extrinsic camera matrix that relates the camera frame to the LiDAR frame.
5. Move the LiDAR points $x$ and $y$ back from homogeneous coordinates to the Euclidean coordinate system to get the position where the LiDAR point hits the image plane in pixel coordinates.

- Homogeneous coordinates
- Euclidean to homogeneous
- Homogeneous to Euclidean


### Intrinsic Parameters

### Extrinsic Parameters



## References

### Papers

- [PointPainting: Sequential Fusion for 3D Object Detection by Vora et. al., 2020](https://arxiv.org/pdf/1911.10150.pdf)
- [3D LIDAR–camera intrinsic and extrinsic calibration: Identifiability and analytical least-squares-based initialization by Mirzaei et. al., 2012](https://www-users.cs.umn.edu/~stergios/papers/IJRR-2012-LidarCameraCalib.pdf)
- [LiDAR-Camera Calibration using 3D-3D Point correspondences](https://arxiv.org/pdf/1705.09785v1.pdf)
