# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

## Tracking Performance Evaluation
### Evaluation 1: Keypoints Detection
* Table 1: Number of keypoints detected on the preceding vehicle for 10 consecutive images, plus description
    * Note: within the table, since the **`ORB`** interfaces requires an input of maximum feature to be detected, the values are all fixed number 3000 (same as the input)

| Detector | Image 1 | Image 2 | Image 3 | Image 4 | Image 5 | Image 6 | Image 7 | Image 8 | Image 9 | Image 10 | Description |
| ---      | ---     | ---     | ---     | ---     | ---     | ---     | ---     | ---     | ---     | ---      | ---         |
| SHITOMASI    | 1370    | 1301    | 1361    | 1358    | 1333    | 1284    | 1322    | 1366    | 1389    | 1339    | Keypoints are dense on vehicle plate and boundary, rare on rear window |
| HARRIS  | 492    | 502   |  516   | 524    | 523   | 511   | 505   | 510   | 529   | 520   | Keypoints are dense on boundary, roof and rear lights, rare on rear window |
| FAST   | 1824   | 1832   | 1810   | 1817   | 1793   | 1796   | 1788   | 1695   | 1749   | 1770   | Keypoints are very dense on vehicle boundary, dense on roof and rear lights, rare on rear window |
| BRISK  | 2757   | 2777   | 2741   | 2735   | 2757   | 2695   | 2715   | 2628   | 2639   | 2672   | Keypoints are very dense on vehicle boundary, roof, dense on rear lights, rare on plate, some on shadow and lower part of the vehicle|
| ORB*   | 3000   | 3000   | 3000   | 3000   | 3000   | 3000   | 3000   | 3000   | 3000   | 3000   | Keypoints are very dense on vehicle boundary, roof, dense on rear lights, rare on rear window |
| AKAZE | 1351  | 1327   | 1311   | 1351   | 1360   | 1347   | 1363   | 1331   | 1358   | 1331   | Keypoints are dense on boundary, roof and rear lights, rare on rear window, some on lower part of the vehicle |
| SIFT  | 1438  | 1371   | 1380   | 1335   | 1305   | 1369   | 1396   | 1382   | 1463   | 1422   | Keypoints are dense on boundary, roof rear lights and vechicle plate, rare on rear window, some on shadow and lower part of the vehicle |
### Evaluation 2: Combination of Keypoints Detection, Description and Matching
* Table 2: Number of keypoints matched on the preceding vehicle for 10 consecutive images

| Combination (Detector + Descriptor + Matcher) | Image 1 | Image 2 | Image 3 | Image 4 | Image 5 | Image 6 | Image 7 | Image 8 | Image 9 | Image 10 | Description |
| ---      | ---     | ---     | ---     | ---     | ---     | ---     | ---     | ---     | ---     | ---      | ---         |
### Evaluation 3: Overall Keypoints Detection / Description Recommendation
* Table 1: Process time of keypoints detection and descriptor extraction on the preceding vehicle for 10 consecutive images

## Background

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

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