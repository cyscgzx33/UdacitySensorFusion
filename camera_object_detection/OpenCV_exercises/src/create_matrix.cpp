#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


using namespace std;

void createMatrix1()
{
    // create matrix
    int nrows = 480, ncols = 640;
    cv::Mat m1_8u;
    m1_8u.create(nrows, ncols, CV_8UC1); // two-channel matrix with 8bit unsigned elements
    m1_8u.setTo(cv::Scalar(255));        // white

    // STUDENT TASK (done):
    // Create a variable of type cv::Mat* named m3_8u which has three channels with a
    // depth of 8bit per channel. Then, set the first channel to 255 and display the result.
    // Yusen's Note: here's the naming system from opencv doc: https://docs.opencv.org/4.1.0/d6/d6d/tutorial_mat_the_basic_image_container.html
    // CV_[The number of bits per item][Signed or Unsigned][Type Prefix]C[The channel number]
    // For instance, CV_8UC3 means we use unsigned char types that are 8 bit long and each pixel has three of these to form the three channels. 
    cv::Mat m3_8u;
    m3_8u.create(nrows, ncols, CV_8UC3);
    m3_8u.setTo(cv::Scalar(255, 0, 0));  // set 3 dims

    // show result
    string windowName = "First steps in OpenCV (m1_8u)";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, m1_8u);
    cv::waitKey(0); // wait for keyboard input (any input) before continuing

    // STUDENT TASK (done):
    // Display the results from the STUDENT TASK above
    string windowName_m3_8u = "First steps in OpenCV (m3_8u)";
    cv::namedWindow(windowName_m3_8u, 1); // create window
    cv::imshow(windowName_m3_8u, m3_8u);
    cv::waitKey(0); // wait for keyboard input (any input) before continuing
}


int main()
{
    createMatrix1();
    return 0;
}