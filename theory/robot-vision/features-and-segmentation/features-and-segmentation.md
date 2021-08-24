## Features
 
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

## Segmentation

- [Contours in OpenCV](https://docs.opencv.org/4.5.3/d3/d05/tutorial_py_table_of_contents_contours.html)
