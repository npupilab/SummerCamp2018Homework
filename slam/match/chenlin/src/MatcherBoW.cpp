#include "Matcher.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>


class MatcherBoW : public Matcher
{
public:
    MatcherBoW(){}

    virtual bool match4initialize(const GSLAM::FramePtr& lastKF,const GSLAM::FramePtr& curFrame,
                                  std::vector<std::pair<int,int> >& matches)const
    {
        std::cout << "test match " << std::endl;

        GSLAM::GImage descriptors_1 = lastKF->getDescriptor();
        GSLAM::GImage descriptors_2 = curFrame->getDescriptor();

        cv::FlannBasedMatcher matcher;
        std::vector<cv::DMatch> matches_;
        matcher.match(descriptors_1, descriptors_2, matches_);

        double min_dist = std::min_element( matches_.begin(), matches_.end())->distance;
        for(auto i : matches_)
        {
            if ( i.distance <= std::max ( 2*min_dist, 30.0 ) )
            {
                matches.push_back ( std::pair<int, int>(i.queryIdx, i.trainIdx) );
            }
        }
        std::cout << matches.size() << std::endl;
        return true;
    }
};

REGISTER_MATCHER(MatcherBoW, bow);
