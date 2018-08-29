//
// Created by oem on 18-8-28.
//

#include "ORBmatcherBF.h"
#include <iostream>


bool ORBmatcherBF::match(const cv::Mat &descriptors1, const cv::Mat &descriptors2,std::vector<cv::DMatch> &result) const {

    //Simple OpenCV BF matcher
    cv::BFMatcher matcher(cv::NORM_HAMMING2);
    matcher.match(descriptors1,descriptors2,result);
    if (result.size() < 10)
    {
        std::cout<<"Matches too less !"<<std::endl;
        return false;
    } else{
        return true;
    }
}


bool ORBmatcherBF::match_lowe_ratio(const cv::Mat &descriptors1, const cv::Mat &descriptors2, double ratio,
                                    std::vector<cv::DMatch> &result) const
{
    //Use Lowe Ratio to filter some matches
    //Good Match: dis(nearest_point) / dis(second_nearest) < ratio
    //ratio=0.6 strict
    //ratio=0.6~0.8 normal
    //ratio<0.4 no matches

    cv::BFMatcher matcher(cv::NORM_HAMMING2);
    std::vector<std::vector<cv::DMatch>> all_matches;

    //find the nearest two points
    matcher.knnMatch(descriptors1,descriptors2,all_matches,2);
    //filtered matches
    for (int i = 0; i < all_matches.size(); ++i) {
        if(all_matches[i][0].distance < all_matches[i][1].distance * ratio ) {
            result.push_back(all_matches[i][0]);
        }
    }

    if (result.size() < 10)
    {
        std::cout<<"Matches too less !"<<std::endl;
        return false;
    } else{
        return true;
    }
}

bool ORBmatcherBF::match_cross_verify(const cv::Mat &descriptors1, const cv::Mat &descriptors2,
                                      std::vector<cv::DMatch> &result) const {

    //Sample Code From OpenCV samples.
    cv::BFMatcher matcher(cv::NORM_HAMMING2);
    std::vector<std::vector<cv::DMatch>> matches12,matches21;
    matcher.knnMatch(descriptors1,descriptors2,matches12,2);
    matcher.knnMatch(descriptors2,descriptors1,matches21,2);

    for (size_t m = 0; m < matches12.size() ; m++) {
        bool findCrossCheck = false;
        for (size_t fk = 0; fk < matches12[m].size() ; fk++) {
            cv::DMatch forward = matches12[m][fk];
            for(size_t bk = 0; bk< matches21[forward.trainIdx].size();bk++)
            {
                cv::DMatch backward = matches21[forward.trainIdx][bk];
                if(backward.trainIdx == forward.queryIdx)
                {
                    result.push_back(forward);
                    findCrossCheck = true;
                    break;
                }
            }
            if(findCrossCheck) break;
        }

    }

    if (result.size() < 10)
    {
        std::cout<<"Matches too less !"<<std::endl;
        return false;
    } else{
        return true;
    }
}


