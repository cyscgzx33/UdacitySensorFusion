
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = 0.1;        // time between two measurements in seconds
    double laneWidth = 4.0; // assumed width of the ego lane

    // find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;

    // an alternative strategy to filter outliers: using 2 percentile point (Step 1 ~ Step 3)
    // reason: assume 98% of the lidar points fall in the trustable region
    std::vector<LidarPoint> lidarPointsPrevCopy, lidarPointsCurrCopy;
    
    // Step 1: copy temp vectors for sorting
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        // Extended a rule based filter: 
        // Only considering Lidar points within a narrow corridor in ego lane
        if (it->y > laneWidth / 2.0 || it->y < -laneWidth / 2.0)
            continue;
        lidarPointsPrevCopy.push_back(*it);
    }
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        // Extended a rule based filter: 
        // Only considering Lidar points within a narrow corridor in ego lane
        if (it->y > laneWidth / 2.0 || it->y < -laneWidth / 2.0)
            continue;
        lidarPointsCurrCopy.push_back(*it);
    }

    // Step 2: sort lidar points from closer to ego vehicle to far away
    sort( lidarPointsPrevCopy.begin( ), lidarPointsPrevCopy.end( ), [ ]( const LidarPoint& lhs, const LidarPoint& rhs )
    {
        return lhs.x < rhs.x;
    });
    sort( lidarPointsCurrCopy.begin( ), lidarPointsCurrCopy.end( ), [ ]( const LidarPoint& lhs, const LidarPoint& rhs )
    {
        return lhs.x < rhs.x;
    });

    // Step 3: compare the quantile point between prev frame and curr frame
    int n_prev = lidarPointsPrevCopy.size();
    int n_curr = lidarPointsCurrCopy.size();
    minXPrev = lidarPointsPrevCopy[n_prev/50].x;
    minXCurr = lidarPointsCurrCopy[n_curr/50].x;
    
    // Debug
    // printf("---------------------------------------------\n");
    // printf("minXCurr = %f, minXPrev = %f\n", minXCurr, minXPrev);

    /* Initial method w/o considering outliers */
    // for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    // {
    //     // Extended a rule based filter: 
    //     // Only considering Lidar points within a narrow corridor in ego lane
    //     if (it->y > laneWidth / 2.0 || it->y < -laneWidth / 2.0)
    //         continue;

    //     minXPrev = minXPrev > it->x ? it->x : minXPrev;
    // }

    // for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    // {
    //     // Extended a rule based filter: 
    //     // Only considering Lidar points within a narrow corridor in ego lane
    //     if (it->y > laneWidth / 2.0 || it->y < -laneWidth / 2.0)
    //         continue;

    //     minXCurr = minXCurr > it->x ? it->x : minXCurr;
    // }

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}


// Implemented matching bboxes using unordered_map of unordered_map
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    cout << "matchBoundingBoxes() is called!" << endl;
    // keypoints of prev & curr frames
    auto& prev_kpts = prevFrame.keypoints;
    auto& curr_kpts = currFrame.keypoints;
    
    // bboxes of prev & curr frames
    std::vector<BoundingBox>& prev_bboxes = prevFrame.boundingBoxes;
    std::vector<BoundingBox>& curr_bboxes = currFrame.boundingBoxes;

    // a container counting occurrances of each pair of matches
    std::unordered_map<int, std::unordered_map<int, int>> occurrances;

    // iterate to count occurrances of pairs of matching
    for (cv::DMatch& match : matches)
    {
        // access prev_kpt & curr_kpt by reference
        int prev_kpt_id = match.queryIdx;
        int curr_kpt_id = match.trainIdx;
        cv::KeyPoint& prev_kpt = prev_kpts[prev_kpt_id];
        cv::KeyPoint& curr_kpt = curr_kpts[curr_kpt_id];

        // find out which bbox contains prev_kpt & curr_kpt
        vector<int> prev_bbox_indices, curr_bbox_indices;
        for (auto& prev_bbox : prev_bboxes)
        {
            if (prev_bbox.roi.contains(prev_kpt.pt))
                prev_bbox_indices.push_back(prev_bbox.boxID);
        }
        for (auto& curr_bbox : curr_bboxes)
        {
            if (curr_bbox.roi.contains(curr_kpt.pt))
                curr_bbox_indices.push_back(curr_bbox.boxID);
        }

        // count pairs
        for (auto prev_bbox_id : prev_bbox_indices)
        {
            for (auto curr_bbox_id : curr_bbox_indices)
            {
                if (occurrances.count(prev_bbox_id) == 0)
                    occurrances.insert({prev_bbox_id, unordered_map<int, int>()});
                occurrances[prev_bbox_id][curr_bbox_id]++;
            }
        }
    }

    // iterate the occurrances to obtain final result of bbBestMatches
    for (auto it = occurrances.begin(); it != occurrances.end(); it++)
    {
        int prev_bbox_id = it->first;
        std::unordered_map<int,int>& prev_id_matches = it->second;

        int max_occurrance = 0;
        int max_occurrance_curr_id = -1;
        
        // find the curr id with highest occurance value
        for (auto it_m = prev_id_matches.begin(); it_m != prev_id_matches.end(); it_m++)
        {
            int curr_bbox_id = it_m->first;
            int match_occurrance = it_m->second;
            if (match_occurrance > max_occurrance)
            {
                max_occurrance = match_occurrance;
                max_occurrance_curr_id = curr_bbox_id;
            }
        }
        // assign bbBestMatches with the knowledge found above
        bbBestMatches[prev_bbox_id] = max_occurrance_curr_id;
    }


    // (Debug) check if the bbBestMatches assignments make sense
    // printf("The best matches are (boxID in prev frame -> boxID in curr frame): \n");
    // for (auto it = bbBestMatches.begin(); it != bbBestMatches.end(); it++)
    // {
    //     printf("%d -> %d.\n", it->first, it->second);
    // }
}
