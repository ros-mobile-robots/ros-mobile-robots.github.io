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

### Gaussian Kernels

## References

- [Self-driving cars with Duckietown](https://learning.edx.org/course/course-v1:ETHx+DT-01x+1T2021/block-v1:ETHx+DT-01x+1T2021+type@sequential+block@c443df0997224ccab9f2c3f762fcc086/block-v1:ETHx+DT-01x+1T2021+type@vertical+block@bc207d642e644b67989c59dbbcb9a0c6)
- [OpenCV Fourier Transform](https://docs.opencv.org/4.5.3/de/dbc/tutorial_py_fourier_transform.html)
- [OpenCV Filtering tutorial](https://docs.opencv.org/4.5.3/d4/d13/tutorial_py_filtering.html)
- [Wikipedia Sobel operator](https://en.wikipedia.org/wiki/Sobel_operator)
