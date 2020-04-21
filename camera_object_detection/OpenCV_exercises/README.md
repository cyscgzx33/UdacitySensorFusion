# OpenCV Exercise

## Install OpenCV in Linux
* Suggest to install **OpenCV 4+**
* Tested on a local machine, Ubuntu 16.04
* Suppose the dependencies are already installed, some reference can be found in [this cite](https://askubuntu.com/questions/1123955/install-opencv4-in-ubuntu-16)
* Suggest to build from source:
    * `git clone https://github.com/opencv/opencv.git`
    * `cd opencv && mkdir build && cd build`
    * `cmake ..`
    * `make`
    * `sudo make install`
* Finally check if the installation is successful, refer to [this cite](https://docs.opencv.org/2.4/doc/tutorials/introduction/linux_gcc_cmake/linux_gcc_cmake.html)

## Some Tips for Using OpenCV
* OpenCV has some confusion part about row / col definitions:
    * For example, `cv::Mat::at(i,j)` is using `(i,j)` as `(row, column)` but `cv::Point(x,y)` is using `(x,y)` as `(column, row)`
    * Refer to [a StackOverfolow page](https://stackoverflow.com/questions/25642532/opencv-pointx-y-represent-column-row-or-row-column) as reference