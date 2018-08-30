//
// Created by oem on 18-8-28.
//

#ifndef PRACICE_PROJECT_ORBMATCHERBF_H
#define PRACICE_PROJECT_ORBMATCHERBF_H

#include <opencv2/features2d/features2d.hpp>


class ORBmatcherBF {
public:
    ORBmatcherBF()= default;
    ~ORBmatcherBF()= default;
    bool match(const cv::Mat &descriptors1, const cv::Mat &descriptors2,std::vector<cv::DMatch> &result) const;
    bool match_lowe_ratio(const cv::Mat &descriptors1, const cv::Mat &descriptors2, double ratio,
                     std::vector<cv::DMatch> &result) const;
    bool match_cross_verify(const cv::Mat &descriptors1, const cv::Mat &descriptors2,
                            std::vector<cv::DMatch> &result) const ;
};


#endif //PRACICE_PROJECT_ORBMATCHERBF_H
