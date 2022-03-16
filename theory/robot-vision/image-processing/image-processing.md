## Image Filtering

The concept of (spacial) frequency in images is important to filter certain frequencies and thus enhance or dampen certain features of an image,
such as corners or edges.

### Frequency in Images

As with 1-D signals, frequency in images describes a rate of change in pixel intensities.

The Fourier Transform (FT) is an important image processing tool which is used to decompose an image into its frequency components. 
The output of an FT represents the image in the frequency domain, while the input image is the spatial domain (x, y) equivalent.

- [OpenCV Fourier Transform](https://docs.opencv.org/4.5.3/de/dbc/tutorial_py_fourier_transform.html)

### Linear Filtering and Convolution


- [OpenCV Filtering Tutorial](https://docs.opencv.org/4.5.2/d4/d13/tutorial_py_filtering.html)


#### Gradient and Sobel Filter

Gradients are a measure of intensity change in an image. As images are treated as functions $I(x,y)$, 
the gradient is the derivative of this function $I'(x,y)$, describing the change in intensity $\Delta I$ at pixel locations $x$ and $y$.

The Sobel filter is very commonly used in edge detection and in finding patterns in intensity in an image. 
Applying a Sobel filter to an image is a way of taking (an approximation) of the derivative of the image in the $x$ or $y$ direction.

$$
S_x = \begin{matrix}
        -1 & 0 & 1 \\
        -2 & 0 & 2 \\
        -1 & 0 & 1 \\
      \end{matrix}
$$

$$
S_y = \begin{matrix}
        -1 & -2 & -1 \\
        0 & 0 & 0 \\
        1 & 2 & 1 \\
      \end{matrix}
$$

!!! note
    TODO add image examples
    
    
- Magnitude: $S = \sqrt{S_x^2 + S_y^2}$
- Direction: $\theta = \atan2{S_y, S_x}$

For more details see also the [Sobel operator on Wikipedia](https://en.wikipedia.org/wiki/Sobel_operator).

### Image Blurring and Low-pass Filter

To block noise in an image, use a filter that filters high frequencies, e.g. specle or discoloration, and let low frequency components of an image pass, such as smooth surfaces.

Without blurring the image and therefore removing the high-frequnecy noise, a preceding high-pass filtering step would also amplify the noise.
To enhance only the high-frequency edges, it is common to first apply a low-pass filter, which basically takes an average of neighbouring pixels.

A simple filter to blur or low-pass filter an image is the normalized average filter:

$$
1/9 \begin{matrix}
     1 & 1 & 1 \\
     1 & 1 & 1 \\
     1 & 1 & 1 \\
    \end{matrix}
$$

!!! note 
    Normalization is important to avoid brightening the image and therefore maintain the same energy.
    For the averiging filter the pixel values sum to one: $1/9 (1+1+1 + 1+1+1 + 1+1+1)=1$

### Gaussian Kernels

Gaussian kernels are used to blur/smooth an image and therefore block high frequency parts of an image.
Compared to an averageing filter, Gaussian kernels better preserve edges.

The gaussian kernel can be seen as a weighted average, which gives the most weight to the center pixel.

$$
1/16 \begin{matrix}
     1 & 2 & 1 \\
     2 & 4 & 2 \\
     1 & 2 & 1 \\
    \end{matrix}
$$

As with the average filter, the values of the gaussian filter sum to one $1/16 (1+2+1 + 2+4+2 + 1+2+1)=1$. 

!!! note
    An even better method to preserve edges are [bilateral filters](https://docs.opencv.org/4.5.3/d4/d86/group__imgproc__filter.html#ga9d7064d478c95d60003cf839430737ed).
    It is highly effective in noise removal while keeping edges sharp. But the operation is slower compared to other filters.
    

Read more about the math behind [Gaussian filters on Wikipedia](https://en.wikipedia.org/wiki/Gaussian_blur) 
and also see the OpenCV documentation of the [`GaussianBlur`](https://docs.opencv.org/4.5.3/d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1) function.


### Convolutional Neural Networks

Convolutional neural networks consist of convolutional layers which are made up of similar filters defined previously.
The only difference is that neural networks learn to create filters themselfes through gradient descent and error back propagation.
The weights of a convolutional neural network that it updates during training are the values inside the filter kernels.

In deep convolutional neural networks the individual filter kernels are stacked which increases the depth of a convolutional filter (or kernel), 
similar to the channels of a color image are stacked. Using stacked filters yields more features from an input image but also increases the number of learnable weights (filter values). In traditional filters the values of filter weights were set explicitly, 
but neural networks will actually learn the best filter weights as they train on a set of image data.


### Canny Edge Detection

OpenCV provides the [`Canny`](https://docs.opencv.org/4.5.3/da/d22/tutorial_py_canny.html) edge detector, which 
is a widely used edge detection algorithm that performs the following steps:

1. **Filters out noise** using a Gaussian blur
2. **Finds the strength and direction of edges** using Sobel filters
3. Applies **non-maximum suppression** to isolate the strongest edges and thin them to one-pixel wide lines.
4. Uses **hysteresis to isolate the best edges** using low and high thresholds to cut/pass intensity values.

Canny edge detection eliminates weak edges and noise and isolates edges that are part of an object boundary.
Read more about the [Canny edge detection algorithm on Wikipedia](https://en.wikipedia.org/wiki/Canny_edge_detector).

### Shape Detection

So far we defined image filters for smoothing images and detecting the edges (high-frequency) components of objects in an image. 
Using this knowledge about pattern recognition in images enables one to begin identifying unique shapes and then objects.

Edges to Boundaries and Shapes
To find unifying boundaries around objects to separate and locate multiple objects in a given image, the Hough transform can be used.
It transforms image data from the x-y coordinate system into Hough space, where you can easily identify simple boundaries like lines and circles.

The Hough transform is used in a variety of shape-recognition applications. A Hough transform can find the edges of driving lanes.

See [this resource](https://homepages.inf.ed.ac.uk/rbf/HIPR2/hough.htm) on Hough transform and also the 
[Wikipedia article](https://en.wikipedia.org/wiki/Hough_transform) about it. An example is found in the [OpenCV Hough lines tutorial](https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html)
    

## References

- [Self-driving cars with Duckietown](https://learning.edx.org/course/course-v1:ETHx+DT-01x+1T2021/block-v1:ETHx+DT-01x+1T2021+type@sequential+block@c443df0997224ccab9f2c3f762fcc086/block-v1:ETHx+DT-01x+1T2021+type@vertical+block@bc207d642e644b67989c59dbbcb9a0c6)
- [OpenCV Fourier Transform](https://docs.opencv.org/4.5.3/de/dbc/tutorial_py_fourier_transform.html)
- [OpenCV Filtering tutorial](https://docs.opencv.org/4.5.3/d4/d13/tutorial_py_filtering.html)
- [Wikipedia Sobel operator](https://en.wikipedia.org/wiki/Sobel_operator)
