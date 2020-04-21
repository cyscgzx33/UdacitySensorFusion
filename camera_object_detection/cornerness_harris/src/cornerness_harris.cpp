#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

using namespace std;

void cornernessHarris()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // visualize results
    string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow(windowName, 4);
    cv::imshow(windowName, dst_norm_scaled);
    cv::waitKey(0);

    // TODO (done): Your task is to locate local maxima in the Harris response matrix 
    // and perform a non-maximum suppression (NMS) in a local neighborhood around 
    // each maximum. The resulting coordinates shall be stored in a list of keypoints 
    // of the type `vector<cv::KeyPoint>`.
    vector<cv::KeyPoint> keypoints;
    for (int x = 1; x < dst_norm_scaled.rows - 1; x++)
    {
        for (int y = 1; y < dst_norm_scaled.cols - 1; y++)
        {
            if ( dst_norm_scaled.at<unsigned int>(x, y) > dst_norm_scaled.at<unsigned int>(x - 1, y) &&
                 dst_norm_scaled.at<unsigned int>(x, y) > dst_norm_scaled.at<unsigned int>(x + 1, y) &&
                 dst_norm_scaled.at<unsigned int>(x, y) > dst_norm_scaled.at<unsigned int>(x, y - 1) &&
                 dst_norm_scaled.at<unsigned int>(x, y) > dst_norm_scaled.at<unsigned int>(x, y + 1) )
                 keypoints.emplace_back(x, y, 1); // what should be the size of keypoint???
        }
    }

}

int main()
{
    cornernessHarris();
}