//
// Created by oem on 18-8-28.
//

#ifndef PRACICE_PROJECT_ORBMATCHERF_H
#define PRACICE_PROJECT_ORBMATCHERF_H

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

class ORBmatcherHF {
public:
    ORBmatcherHF() = default;
    ~ORBmatcherHF() = default;
    bool matchFundamental(std::vector<cv::KeyPoint> keypoints1, cv::Mat descriptors1,
                          std::vector<cv::KeyPoint> keypoints2, cv::Mat descriptors2, int distance,
                          std::vector<cv::DMatch> &good_matches,std::vector<cv::KeyPoint> &inliers1,std::vector<cv::KeyPoint> &inliers2);

    bool matchHomography(std::vector<cv::KeyPoint> keypoints1, cv::Mat descriptors1,
                         std::vector<cv::KeyPoint> keypoints2, cv::Mat descriptors2,
                         std::vector<cv::DMatch> &good_matches,std::vector<cv::KeyPoint> &inliers1,std::vector<cv::KeyPoint> &inliers2);

    bool matchMultiHomohraphy(std::vector<cv::KeyPoint> keypoints1, cv::Mat descriptors1,
                              std::vector<cv::KeyPoint> keypoints2, cv::Mat descriptors2,
                              std::vector<cv::DMatch> &good_matches,std::vector<cv::KeyPoint> &inliers1,std::vector<cv::KeyPoint> &inliers2);

    bool matchHash(std::vector<cv::KeyPoint> keypoints1, cv::Mat descriptors1,
                   std::vector<cv::KeyPoint> keypoints2, cv::Mat descriptors2,
                   std::vector<cv::DMatch> &good_matches,std::vector<cv::KeyPoint> &inliers1,std::vector<cv::KeyPoint> &inliers2){}
};


#endif //PRACICE_PROJECT_ORBMATCHERF_H
