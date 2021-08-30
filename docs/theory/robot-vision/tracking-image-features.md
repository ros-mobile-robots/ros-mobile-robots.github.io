## Tracking Image Features

### Corner Detectors

A corner can be located by following these steps:

- Calculate the gradient for a small window of the image, using sobel-x and sobel-y operators (without applying binary thesholding).
- Use vector addition to calculate the magnitude and direction of the total gradient from these two values.
- Apply this calculation as you slide the window across the image, calculating the gradient of each window. When a big variation in the direction & magnitude of the gradient has been detected - a corner has been found!

### Intensity Gradient and Filtering

### Harris Corner Detector

- [OpenCV Harris tutorial](https://docs.opencv.org/4.5.3/dc/d0d/tutorial_py_features_harris.html)

### Overview of Popular Keypoint Detectors


### Descriptors (Feature Vector)

Descriptors also known als feature vectors provide distinctive information on the surrounding area of a keypoint.
The literature differentiates between gradient-based descriptors and binary descriptors, 
with the latter being a relatively new addition with the clear advantage of computational speed. 

- An example of a gradient-based descriptor is the Scale Invariant Feature Transform (SIFT).
- A representative of binary descriptors is the Binary Robust Invariant Scalable Keypoints (BRISK).

### HOG Descriptors (SIFT and SURF, etc.)

### Binary Descriptors (BRISK, ORB, etc.)


#### Oriented FAST and Rotated BRIEF (ORB)

ORB is a combination of FAST and BRIEF. FAST is a feature detection algorithm, while BRIEF is a feature vector creation algorithm.

ORB starts by detecting special regions in an image called keypoints, which is a small region in an image, that is particularly distinctive.
Such keypoints are for example corners, where the pixel values sharply change from light to dark.

!!! note
    TODO image showing ORB features.
    
After locating keypoints in an image, ORB calculates a corresponding feature vector for each keypoint.
The ORB algorithm creates feature vectors that contain only ones and zeros, which is why they are called
**binary feature vectors**:

$$
V1 = [01011100100110\dots]\\
V2 = [10010100110100\dots]\\
V3 = [11000100101110\dots]\\
\hdots
$$

The sequence of ones and zeros varies, according to what a specific keypoint and its surrounding pixel area looks like.
The vector represents the pattern of intensity around a keypoint. So multiple feature vectors can be used to identify a larger area
and even a specific object in an image.

ORB is not only incredible fast but it is also impervious to noise illumination, and image transformations such as rotations.

## References

### Text Books and Papers

- Peter Corke, Robotics, Vision, and Control. ([Official website](https://petercorke.com/books/robotics-vision-control-all-versions/)) 
  This book has a particular emphasis on computer vision for robotics, but as the title suggests, it goes beyond robot vision.
- Richard Szeliski, Computer Vision: Algorithms and Applications, Springer. The [official website](https://szeliski.org/Book/) provides drafts of recent updates to the book.
- Richard Hartley and Andrew Zisserman, Multiple View Geometry,  Cambridge University Press. ([Official website](https://www.robots.ox.ac.uk/~vgg/hzbook/))
- David Forsyth and Jean Ponce, Computer Vision: A Modern Approach, Pearson. ([Publisher's website](https://www.pearson.com/us/higher-education/program/Forsyth-Computer-Vision-A-Modern-Approach-2nd-Edition/PGM111082.html))
- [Evaluation of Several Feature Detectors/Extractors on Underwater Images towards vSLAM](https://www.mdpi.com/1424-8220/20/15/4343).

### Courses

- [Udacity Sensor Fusion Engineer Nanodegree](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313)
- Stanford University, CS231A: Computer Vision, From 3D Reconstruction to Recognition ([Course website](https://web.stanford.edu/class/cs231a/))
- Georgia Tech, CS 6476: Computer Vision ([Course website](https://www.cc.gatech.edu/~hays/compvision/))
- MIT, 6.819/6.869: Advances in Computer Vision ([Course website](http://6.869.csail.mit.edu/sp21/))

### Online Resources

- OpenCV Tutorials ([website](https://docs.opencv.org/master/d9/df8/tutorial_root.html))
- OpenCV Python Tutorials ([website](https://docs.opencv.org/4.5.2/d6/d00/tutorial_py_root.html))
- [Medium SIFT (Scale Invariant Feature Transform)](https://towardsdatascience.com/sift-scale-invariant-feature-transform-c7233dc60f37)
- [Detecting lane lines on the road](https://towardsdatascience.com/teaching-cars-to-see-advanced-lane-detection-using-computer-vision-87a01de0424f)

