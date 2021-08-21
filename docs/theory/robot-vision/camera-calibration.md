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
     \end{bmatrix}}_{\text{\textbf{intrinsics} \\ 5 DOF + lens distortion}}
     \underbrace{\begin{bmatrix}
      R | t
     \end{bmatrix}}_{\text{\textbf{extrinsics} \\ 6 DOF}}
     \mathbf{X} = P \mathbf{X}
$$

### Calibration via Direct Linear Transformation

### Calibration as an Optimization Problem

### Homographies and Homography Estimation

## References

- [Self-Driving Cars with Duckietown]()
- [Mathworks Camera Calibration](https://de.mathworks.com/help/vision/ug/camera-calibration.html)
- [Pin-hole Camera Calibration with Matlab Toolbox Documentation](http://www.vision.caltech.edu/bouguetj/calib_doc/)
