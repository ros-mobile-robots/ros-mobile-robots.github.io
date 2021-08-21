## Camera Calibration

As mentioned previously, the pin-hole camera model describes a light-proof box with a small aperture that allows a limited amount of light
reflecting off of objects in the scene to pass through and strike the sensor plane. The result is an idealized, yet surprisingly useful approximation
of the cameras commonly used on a variety of robots.

The model describes how points in the world are mapped to image-space coordinates. This is done, by first transorming points given in
world coordinates into the camera's reference frame, and then projecting the points onto the image plane. 
In order to use this model, we need to know its parameters, such as the focal length $f$ and skew $s$ of the camera (intrinsic parameters),
and its pose relative to the frame of the world or the robot (extrinsic parameters). Calibration refers to the process of estimating these parameters.


!!! note
    TODO missing transformation image

$$
\mathbf{x} = \underbrace{\begin{bmatrix} 
      f_x & s & p_x \\
      0   & f_y & p_y \\
      0   & 0   & 1 \\
     \end{bmatrix}}_{\textbf{intrinsics} \\ \text{5 DOF + lens distortion}}
     \underbrace{\begin{bmatrix}
      R | t
     \end{bmatrix}}_{\text{\textbf{extrinsics}} \\ \text{6 DOF}}
     \mathbf{X} = P \mathbf{X}
$$


Having a mathematical expression for perspective projection allows us to reason over a robot's three-dimensional world from two-dimensional images.

Suppose an algorithm that detects lane markings in an image received from the robot's camera. In order to use these detections to keep the robot in its lane,
it would be useful to understand where tehy are in relation to the robot. 
As we will see, calibrating the camera allows us to transform these detections into the robot's reference frame.
The result can then be used to understand the lane geometry and where the robot is relative to the lane.

There are different approaches to calibrating a camera. The pin-hole camera model can be represendet as a product of two matrices,
one defined in terms of up to five parameters intrinsic to the camera, and the other specifying the camera six degrees of freedom pose.
Ignoring lens distortions, there is a total of 11 parameters that define the camera model.

Nonlinear intrinsic parameters such as lens distortion are also important although they cannot be included in the linear camera model described by the intrinsic parameter matrix. 
Many modern camera calibration algorithms estimate these intrinsic parameters as well in the form of non-linear optimisation techniques. This is done in the form of optimising the camera and distortion parameters in the form of what is generally known as bundle adjustment.[^wiki]


The calibration process typically involves associating the coordinates of points in the scene with their corresponding projections onto the image plane.
Each correspondence $\mathbf{x}_i \leftrightarrow \mathbf{X}_i$ provides two constraints on the projection model. One for each of the two image coordinates.

With 11 degrees-of-freedom in the model, we need at least six point-pairs but have to be careful about avoiding degeneracies.
For example, the three points can't all lie on the same line or the same plane.

In practice, we use a calibration target that provides a well-defined set of 3D points that are easy to detect in the image, 
often involving one or more checkerboard patterns. One option for calibrating the camera is to directly estimate
the entries of the three-by-four camera matrix P that maps 3D scene points to their 2D image coordinates.
While the matrix has 12 entries, there are only 11 degrees-of-freedom, since perspective projection is only defined up to scale.



### Calibration via Direct Linear Transformation

### Calibration as an Optimization Problem

### Homographies and Homography Estimation

## References

- [Self-Driving Cars with Duckietown](https://learning.edx.org/course/course-v1:ETHx+DT-01x+1T2021/block-v1:ETHx+DT-01x+1T2021+type@sequential+block@a43815226b7242b7a071a10bbdc8ffa2/block-v1:ETHx+DT-01x+1T2021+type@vertical+block@17abe5f8b31049a2ad37c52c1e9d1107)
- [Mathworks Camera Calibration](https://de.mathworks.com/help/vision/ug/camera-calibration.html)
- [Pin-hole Camera Calibration with Matlab Toolbox Documentation](http://www.vision.caltech.edu/bouguetj/calib_doc/)

[^wiki]: [Wikipedia Camera Resectioning](https://en.wikipedia.org/wiki/Camera_resectioning)
