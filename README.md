### 2D Feature Tracking in Camera Images for Sensor Fusion

<img src="images/keypoints.png" width="820" height="248" />

## Goal
The goal of this project is to build a collision detection system.  I have built the feature tracking part and test various detector / descriptor combinations to see which ones perform best. 

## Introduction


## Data Buffer Optimization
Since computer vision algorithms will be deployed in mobile hardware with limited resources, optimizing  the amount of data held in memory is of significant importance. For this purpose, i have implemented a data buffer modeled based on Queue data structure. Here the first image to enter will also leave first to accommodate next image and maintain a constant queue size, in this case 2.






## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.
