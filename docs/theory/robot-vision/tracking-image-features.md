## Tracking Image Features

### Features
 
Features and feature extraction form the basis for many computer vision applications and it is an important task in computer vision
to select the right features.
The idea is to represent the content of an image as a piece of information, named a feature. Thus defining a set of data, 
such as a set of images, as a smaller, simpler model made of a combination of visual features: a few colors and shapes.
Features as measurable pieces of data in an image that help distinguish between different classes of images.
 
There are two main types of features, which can be useful individually or in combination:

1. Color-based and
2. Shape-based

Edges are one of the simplest shapes that you can detect; 
edges often define the boundaries between objects but they may not provide enough information to find and identify small features on those objects.


### Types of Features

1. Edges: Areas with a high intensity gradient
2. Corners: Intersection of two edges
3. Blobs: Region based features; areas of extreme brightness or unique texture


Corners are the most repeatable feature, which means they are easy to recognize given two or more images of the same scene.
Corners match exactly and are good features, therefore we are interested in finding this type of features.

### Corner Detectors

A corner can be located by following these steps:

- Calculate the gradient for a small window of the image, using sobel-x and sobel-y operators (without applying binary thesholding).
- Use vector addition to calculate the magnitude and direction of the total gradient from these two values.
- Apply this calculation as you slide the window across the image, calculating the gradient of each window. When a big variation in the direction & magnitude of the gradient has been detected - a corner has been found!

### Intensity Gradient and Filtering

### Harris Corner Detector

- [OpenCV Harris tutorial](https://docs.opencv.org/4.5.3/dc/d0d/tutorial_py_features_harris.html)

### Overview of Popular Keypoint Detectors

### HOG Descriptors (SIFT and SURF, etc.)

### Binary Descriptors (BRISK, ...)


## References

### Text Books

- Peter Corke, Robotics, Vision, and Control. ([Official website](https://petercorke.com/books/robotics-vision-control-all-versions/)) 
  This book has a particular emphasis on computer vision for robotics, but as the title suggests, it goes beyond robot vision.
- Richard Szeliski, Computer Vision: Algorithms and Applications, Springer. The [official website](https://szeliski.org/Book/) provides drafts of recent updates to the book.
- Richard Hartley and Andrew Zisserman, Multiple View Geometry,  Cambridge University Press. ([Official website](https://www.robots.ox.ac.uk/~vgg/hzbook/))
- David Forsyth and Jean Ponce, Computer Vision: A Modern Approach, Pearson. ([Publisher's website](https://www.pearson.com/us/higher-education/program/Forsyth-Computer-Vision-A-Modern-Approach-2nd-Edition/PGM111082.html))

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

