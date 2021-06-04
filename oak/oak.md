# OpenCV AI Kit

The [OpenCV AI Kit](https://store.opencv.ai/) (OAK) is an open source hardware and software project for spatial AI and is a result of a very successful [Kickstarter Campaign](https://www.kickstarter.com/projects/opencv/opencv-ai-kit/description).
The hardware can basically be separated into two devices:

- OAK—D is a spatial AI powerhouse, capable of simultaneously running advanced neural networks while providing depth from two stereo cameras and color information from a single 4K camera in the center.
  The OAK—D hardware comes with a 1 meter USB-C cable and a 5V power supply.

- OAK—1 is the tiny-but-mighty 4K camera capable of running the same advanced neural networks as OAK—D, but in an even more minuscule form factor for projects where space and power are at a premium.
  Each OAK—1 kit includes the OAK—1 module with aluminum enclosure, 1 meter USB 3 Type-A to Type-C cable, and getting started guide.
  
!!! note
    There are also options for onboard Wifi and Power over Ethernet (POE).

!!! quote What is spacial AI and 3D Object Localization?
    First, it is necessary to define what 'Object Detection' is:
    It is the technical term for finding the bounding box of an object of interest, in pixel space (i.e. pixel coordinates), in an image.
    
    3D Object Localization (or 3D Object Detection), is all about finding such objects in physical space, instead of pixel space. 
    This is useful when trying to real-time measure or interact with the physical world.


This part of the documentation explains the following

- Installing software for OAK on your host (Linux, macOS, Windows, Raspberry Pi)
- Interacting with OAK and how to run the demo programs
- Deploy a custom model with [OpenVINO Toolkit](https://docs.openvinotoolkit.org/latest/index.html)
- Training your own Model with [SuperAnnotate](https://superannotate.com/)
- Train your own Model on a GPU, either locally or in the Cloud, using e.g. Tensorflow Keras.
- Project using OAK


## Setup of OAK-1 and OAK-D

The documentation of OAK can be found at [docs.luxonis.com](https://docs.luxonis.com/en/latest/).
To install the software for both devices on your different host platforms please follow the instructions in the [Python API](https://docs.luxonis.com/en/latest/pages/api/).


- TODO install command for  different os
- Test installation instructions

## Interaction with OAK

- Demo Program Overview
- Examples
  - Object Detection with Yolo-v3
  - ...
  
  
## Python API: DepthAI


## Deplyinig Neural Network Models to OAK

## SuperAnnotate

## Training Custom Models
