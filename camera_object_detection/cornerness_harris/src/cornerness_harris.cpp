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
    double max_overlap = 0.0; // max permissible overlap between two features in %,
                              // will be used in non-maximum supression (NMS)
    for (int x = 0; x < dst_norm.rows; x++)
    {
        for (int y = 0; y < dst_norm.cols; y++)
        {
            int response = (int)dst_norm.at<float>(x, y);
            if (response <= minResponse)
                continue;
            // only store points above the threshold minResponse
            cv::KeyPoint new_keypoint;
            new_keypoint.pt = cv::Point2f(y, x); // fix: for some reason, here it must be (y, x)
                                                 // guess it's the definition by opencv
                                                 // So, this means that src.at(i,j) is using (i,j) as (row,column) but Point(x,y) is using (x,y) as (column,row)
                                                 // refer to StackOverfollow page for reference: https://stackoverflow.com/questions/25642532/opencv-pointx-y-represent-column-row-or-row-column
            new_keypoint.size = 2 * apertureSize;
            new_keypoint.response = response;

            // perform non-maximum supression (NMS) in local neighbourhood around new key point
            bool b_overlap = false;
            for (auto& keypoint : keypoints)
            {
                double kpt_overlap = cv::KeyPoint::overlap(new_keypoint, keypoint); // calculate the overlap between keypoints
                if (kpt_overlap > max_overlap)
                {
                    b_overlap = true;
                    if (new_keypoint.response > keypoint.response) // if overlap is higher than threshold AND
                                                                   // response of new keypoint is higher
                    {
                        keypoint = new_keypoint;                   // replace the old keypoint with the new one and quit the loop
                        break;
                    }
                }
            }

            // add the new keypoint if no overlap is found
            if (b_overlap == false)                                
                keypoints.push_back(new_keypoint);
        }
    }

    // visualize results
    windowName = "Harris Corner Detector Detection Results";
    cv::namedWindow(windowName, 5);
    cv::Mat vis_image = dst_norm_scaled.clone();
    cv::drawKeypoints(dst_norm_scaled, keypoints, vis_image, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow(windowName, vis_image);
    cv::waitKey(0);
}

void cornernessHarrisSolution()
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

    // STUDENTS NEET TO ENTER THIS CODE (C3.2 Atom 4)

    // Look for prominent corners and instantiate keypoints
    vector<cv::KeyPoint> keypoints;
    double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
    for (size_t j = 0; j < dst_norm.rows; j++)
    {
        for (size_t i = 0; i < dst_norm.cols; i++)
        {
            int response = (int)dst_norm.at<float>(j, i);
            if (response > minResponse)
            { // only store points above a threshold

                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                bool bOverlap = false;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        bOverlap = true;
                        if (newKeyPoint.response > (*it).response)
                        {                      // if overlap is >t AND response is higher for new kpt
                            *it = newKeyPoint; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }
                    }
                }
                if (!bOverlap)
                {                                     // only add new key point if no overlap has been found in previous NMS
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }
            }
        } // eof loop over cols
    }     // eof loop over rows

    // visualize keypoints
    windowName = "Harris Corner Detection Results";
    cv::namedWindow(windowName, 5);
    cv::Mat visImage = dst_norm_scaled.clone();
    cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow(windowName, visImage);
    cv::waitKey(0);
    // EOF STUDENT CODE
}


int main()
{
    cornernessHarris();
}