//
// Created by LiQing on 2018/8/20.
//

#include "Matcher.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>


class MatcherBoW : public Matcher
{
public:
    MatcherBoW(){
    }

    virtual bool match4initialize(const GSLAM::FramePtr& lastKF,const GSLAM::FramePtr& curFrame,
                                  std::vector<std::pair<int,int> >& matches)const
    {

        std::cout << "test match " << std::endl;

        GSLAM::GImage descriptors_1 = lastKF->getDescriptor();
        GSLAM::GImage descriptors_2 = curFrame->getDescriptor();


        cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
        std::vector<cv::DMatch> matches_;
        matcher->match(descriptors_1, descriptors_2, matches_);

        double min_dist=10000, max_dist=0;

        for ( int i = 0; i < descriptors_1.rows; i++ )
        {
            double dist = matches_[i].distance;
            if ( dist < min_dist ) min_dist = dist;
            if ( dist > max_dist ) max_dist = dist;
        }

        min_dist = std::min_element( matches_.begin(), matches_.end(), [](const cv::DMatch& m1, const cv::DMatch& m2) {return m1.distance<m2.distance;} )->distance;
        max_dist = std::max_element( matches_.begin(), matches_.end(), [](const cv::DMatch& m1, const cv::DMatch& m2) {return m1.distance<m2.distance;} )->distance;

//        std::vector< cv::DMatch > good_matches;
        for ( int i = 0; i < descriptors_1.rows; i++ )
        {
            if ( matches_[i].distance <= std::max ( 2*min_dist, 30.0 ) )
            {
                matches.push_back ( std::pair<int, int>(matches_[i].queryIdx, matches_[i].trainIdx) );
            }
        }

        std::cout << matches.size() << std::endl;
        return true;
    }
};

REGISTER_MATCHER(MatcherBoW, bow);