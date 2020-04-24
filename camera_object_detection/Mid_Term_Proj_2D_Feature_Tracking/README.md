# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

## Tracking Performance Evaluation
### Evaluation 1: Keypoints Detection
* Table 1: Number of keypoints detected on the preceding vehicle for 10 consecutive images, plus description of keypoints distribution
    * Note 1: within the table, since the `ORB` interfaces requires an input of maximum feature to be detected, the values are all fixed number 3000 (same as the input)

| Detector | Shi-Tomasi | Harris | FAST    | BRISK    | ORB*    | Akaze   | SIFT |
| ---      | ---        | ---    | ---     | ---      | ---     | ---     | ---  |
| Image 1  | 1370       | 492    | 1824    | 2757     | 3000    | 1351    | 1438 |
| Image 2  | 1301       | 502    | 1832    | 2777     | 3000    | 1327    | 1371 |
| Image 3  | 1361       | 516    | 1810    | 2741     | 3000    | 1311    | 1380 |
| Image 4  | 1358       | 524    | 1817    | 2735     | 3000    | 1351    | 1335 |
| Image 5  | 1333       | 523    | 1793    | 2757     | 3000    | 1360    | 1305 |
| Image 6  | 1284       | 511    | 1796    | 2695     | 3000    | 1347    | 1369 |
| Image 7  | 1322       | 505    | 1788    | 2715     | 3000    | 1363    | 1396 |
| Image 8  | 1366       | 510    | 1695    | 2628     | 3000    | 1311    | 1382 |
| Image 9  | 1389       | 529    | 1749    | 2639     | 3000    | 1358    | 1463 |
| Image 10 | 1339       | 520    | 1770    | 2672     | 3000    | 1331    | 1422 |
| Distribution | Dense on vehicle plate and boundary, rare on rear window | Dense on boundary, roof and rear lights, rare on rear window | Very dense on vehicle boundary, dense on roof and rear lights, rare on rear window | Very dense on vehicle boundary, roof, dense on rear lights, rare on plate, shadow and lower part of the vehicle| Very dense on vehicle boundary, roof, dense on rear lights, rare on rear window | Dense on boundary, roof and rear lights, rare on rear window, lower part of vehicle | Dense on boundary, roof rear lights and vechicle plate, rare on rear window, shadow and lower part of the vehicle |

### Evaluation 2: Keypoints Matching
* Table 2: total number of keypoints matched on the preceding vehicle for 10 consecutive images with different combinations of detectors and descriptors
    * Note 0: the row represents detectors, and the colomn represents descriptors
    * Note 1: among all the tests here, the BF (brutal force) approach is used with the descriptor distance ratio set to 0.8
    * Note 2: as there're quite a lot combinations, here only the total matched keypoints are shown instead of at each image pair
    * There're some known issues about some imcompatible combinations (listed as **N/A** in the table below), such as **`SIFT` + `ORB`** and **`Shi-Tomasi`/`Harris`/`FAST`/`BRISK`/`ORB`/`SIFT` + `Akaze`**. The reason is mainly about how **OpenCV** implemented these algorithms, for more details one can refer to [amroamroamro's answer in this github issue](https://github.com/kyamagu/mexopencv/issues/351)

| Combinations | Shi-Tomasi | Harris | FAST     | BRISK | ORB* | Akaze | SIFT |
| ---          | ---        | ---    | ---      | ---   | ---  | ---   | ---  |
| **BRISK**    | 690        | 355    | 776      | 1298  | 2154 | 1110  | 536  |
| **BRIEF**    | 816        | 403    | 883      | 1344  | 1454 | 1087  | 597  |
| **ORB**      | 765        | 392    | 859      | 913   | 1677 | 922   | N/A  |
| **FREAK**    | 575        | 303    | 657      | 1090  | 1142 | 966   | 500  |
| **AKAZE**    | N/A        | N/A    | N/A      | N/A   | N/A  | 1172  | N/A  |
| **SIFT**     | 927        | 459    | 1046     | 1646  | 2651 | 1270  | 800  |

### Evaluation 3: Overall Keypoints Detection / Description Recommendation
* Table 3: total process time of keypoints detection and descriptor extraction on the preceding vehicle for 10 consecutive images
    * Note 0: the row represents detectors, and the colomn represents descriptors
    * Note 1: among all the tests here, the BF (brutal force) approach is used with the descriptor distance ratio set to 0.8
    * Note 2: as there're quite a lot combinations, here only the total matched keypoints are shown instead of at each image pair
    * There're some known issues about some imcompatible combinations (listed as **N/A** in the table below), such as **`SIFT` + `ORB`** and **`Shi-Tomasi`/`Harris`/`FAST`/`BRISK`/`ORB`/`SIFT` + `Akaze`**. The reason is mainly about how **OpenCV** implemented these algorithms, for more details one can refer to [amroamroamro's answer in this github issue](https://github.com/kyamagu/mexopencv/issues/351)

| Combinations | Shi-Tomasi | Harris | FAST     | BRISK    | ORB*    | Akaze   | SIFT    |
| ---          | ---        | ---    | ---      | ---      | ---     | ---     | ---     |
| **BRISK**    | 103.49     | 114.56 | 22.69    | 2963.04  | 245.16  | 452.53  | 602.07  |
| **BRIEF**    | 102.86     | 108.10 | 14.48    | 2890.82  | 224.19  | 461.82  | 705.15  |
| **ORB**      | 124.15     | 131.98 | 38.42    | 3132.75  | 321.26  | 491.42  | N/A     |
| **FREAK**    | 431.02     | 425.90 | 348.30   | 3365.57  | 560.51  | 760.99  | 1001.85 |
| **AKAZE**    | N/A        | N/A    | N/A      | N/A      | N/A     | 758.23  | N/A     |
| **SIFT**     | 165.02     | 151.20 | 133.25   | 3191.54  | 579.96  | 537.14  | 928.13  |

### Final Recommendation for Detector / Descriptor Combinations as for Vehicle Detection
* TOP 3 choices: **`FAST` + `BRIEF`**, **`FAST` + `BRISK`** and **`FAST` + `ORB`**
    * Reason for choosing these three combinations are:
        * Very fast: processing speed is the most important factor for choosing them, which are 14.48 ms, 22.69 ms and 38.42 ms respectively to complete the tests
        * Fairly sufficient number of keypoints detected & matched: second criteria is to have good amount of keypoints detected and matched, these three provide 883, 776 and 859 respectively, although not the best but good enough
        * Diverse distribution within the image: the distribution of `FAST` detector includes keypoints detected dense on vehicle boundary, dense on roof and rear lights, rare on rear window, which is a good diversity

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