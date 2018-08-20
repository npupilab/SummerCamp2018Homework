#include "Matcher.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>

class FeatureMatcher:public Matcher
{
public:
    FeatureMatcher():Matcher(){}
    virtual bool match4initialize(const GSLAM::FramePtr &lastKF,
                                  const GSLAM::FramePtr &curFrame,
                                  std::vector<std::pair<int, int> > &matches) const;
};

bool FeatureMatcher::match4initialize(const GSLAM::FramePtr &lastKF,
                                      const GSLAM::FramePtr &curFrame,
                                      std::vector<std::pair<int, int> > &matches) const
{
    std::vector<cv::DMatch> matches_temp;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    cv::Mat descriptors1=lastKF->getDescriptor();
    cv::Mat descriptors2=curFrame->getDescriptor();
    matcher.match(descriptors1,descriptors2,matches_temp);
    double min_dis=1000,max_dis=0;
    for(int i=0;i<descriptors1.rows;i++)
    {
        double dist = matches_temp[i].distance;
        if(dist<min_dis) min_dis = dist;
        if(dist>max_dis) max_dis = dist;
    }
    for(int i=0;i<descriptors1.rows;i++)
    {
        if(matches_temp[i].distance <=std::max(2*min_dis,30.0)){
        matches.push_back(std::pair<int,int>(matches_temp[i].queryIdx,matches_temp[i].trainIdx));
        }
    }
    if(matches.size()>10)
    {
        return true;
    }
    else
    {
        return false;
    }
}

REGISTER_MATCHER(FeatureMatcher,ZGMF_X20A)
