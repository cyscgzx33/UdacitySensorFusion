#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <queue> // for priority_queue


#include "dataStructures.h"
#include "structIO.hpp"

using namespace std;

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
    double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();

    double dT = 1 / frameRate;
    TTC = -dT / (1 - meanDistRatio);

    // STUDENT TASK (replacement for meanDistRatio, done)
    priority_queue<double> pq; // max heap, the top() is largest
    int n_ratio = distRatios.size();
    int n_thres = n_ratio / 2 + 1;
    for (auto ratio : distRatios)
    {
        if (pq.size() > n_thres)
        {
            if (pq.top() > ratio)
            {
                pq.pop();
                pq.push(ratio);
            }
        }
        else
            pq.push(ratio);
    }

    double medianDistRatio;
    if (n_ratio == 0)
        medianDistRatio = 0.0;
    else if (n_ratio % 2 == 1)
        medianDistRatio = pq.top();
    else
    {
        medianDistRatio += pq.top();
        pq.pop();
        medianDistRatio += pq.top();
        medianDistRatio /= 2.0;
    }

    dT = 1 / frameRate;
    TTC = -dT / (1 - medianDistRatio);

    // Another version (solution)
    // STUDENT TASK (replacement for meanDistRatio)
    // std::sort(distRatios.begin(), distRatios.end());
    // long medIndex = floor(distRatios.size() / 2.0);
    // double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    // dT = 1 / frameRate;
    // TTC = -dT / (1 - medDistRatio);
    // EOF STUDENT TASK

    // compare the results
    // cout << "n_ratio = " << n_ratio << endl;
    // cout << "medDistRatio = " << medDistRatio << ", medianDistRatio = " << medianDistRatio << endl;
    // cout << "distRatios[1506] = " << distRatios[1506] << ", " << ", distRatios[1507]" << distRatios[1507] << ", distRatios[1508]" << distRatios[1508] << ", distRatios[1509]" << distRatios[1509] << endl;
    // cout << ", distRatios[1407]" << distRatios[1407] << endl;
    // cout << "pq.size() = " << pq.size() << endl;
}

int main()
{
    vector<cv::KeyPoint> kptsSource, kptsRef;
    readKeypoints("../dat/C23A5_KptsSource_AKAZE.dat", kptsSource); // readKeypoints("./dat/C23A5_KptsSource_SHI-BRISK.dat"
    readKeypoints("../dat/C23A5_KptsRef_AKAZE.dat", kptsRef); // readKeypoints("./dat/C23A5_KptsRef_SHI-BRISK.dat"

    vector<cv::DMatch> matches;
    readKptMatches("../dat/C23A5_KptMatches_AKAZE.dat", matches); // readKptMatches("./dat/C23A5_KptMatches_SHI-BRISK.dat", matches);
    double ttc; 
    computeTTCCamera(kptsSource, kptsRef, matches, 10.0, ttc);
    cout << "ttc = " << ttc << "s" << endl;
}