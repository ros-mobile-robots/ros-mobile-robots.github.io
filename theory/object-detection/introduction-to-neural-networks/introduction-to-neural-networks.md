## Introduction to Neural Networks

Convolutional neural networks can learn features through supervised learning and can be categorized into three different types:

1. Classification 
2. Object detection
3. Segmentation

The underlying basis of all these neural networks is the convolutional layer.

A classification CNN takes in an input image and outputs a distribution of class scores.
This is done by feeding the input image through convolutional layers, which is the backbone of a CNN.
These layers are used to filter the input image, and the filters are also known as convolution kernels.
A kernel has a certain (small) size and one or more channels, specifying its depth.
Each filter extracts different features from an image, such as edges or colors.
As a CNN trains it updates the weights that define the image filters in this convolutional layer using [back propagation](https://en.wikipedia.org/wiki/Backpropagation).
To learn more about backprogagation also visit this blog post.

## References

- [Self-Driving Cars with Duckietown](https://learning.edx.org/course/course-v1:ETHx+DT-01x+1T2021/block-v1:ETHx+DT-01x+1T2021+type@sequential+block@e82fa6cafbee43feaa24c77408ea5658/block-v1:ETHx+DT-01x+1T2021+type@vertical+block@05e2bb20a6f34d77be200ce92047e6d8)
- https://ujjwalkarn.me/2016/08/11/intuitive-explanation-convnets/
- [Gradient Descent](https://en.wikipedia.org/wiki/Gradient_descent)
- [Yes you should understand backprop by Andrej Karpathy](https://karpathy.medium.com/yes-you-should-understand-backprop-e2f06eab496b)
- [Hacker's guide to Neural Networks](http://karpathy.github.io/neuralnets/)
- [CS231n taught by Andrej Karpathy](https://www.youtube.com/watch?v=NfnWJUyUJYU&list=PLkt2uSq6rBVctENoVBg1TpCC7OQi31AlC)
