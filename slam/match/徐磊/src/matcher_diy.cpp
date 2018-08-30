#include "Matcher.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include "ORBmatcherHF.h"
#include <iostream>

using namespace std;


class MatcherDIY:public Matcher
{
public:
    MatcherDIY():Matcher(){}
    virtual bool match4initialize(const GSLAM::FramePtr &lastKF,
                                  const GSLAM::FramePtr &curFrame,
                                  std::vector<std::pair<int, int> > &matches) const;
};

bool MatcherDIY::match4initialize(const GSLAM::FramePtr &lastKF,
                                      const GSLAM::FramePtr &curFrame,
                                      std::vector<std::pair<int, int> > &matches) const
{   
//    std::vector<cv::DMatch> result_matches;
//    std::vector<cv::KeyPoint> inliers1,inliers2;
//    ORBmatcherHF  matcher;

//    std::vector<GSLAM::KeyPoint> g_keypoint1,g_keypoint2;
//    if(!lastKF->getKeyPoints(g_keypoint1) && !curFrame->getKeyPoints(g_keypoint2))
//    {
//        cout<<"Can not getKeyPoints"<<endl;
//        return false;
//    }
//    std::vector<cv::KeyPoint> keypoints1,keypoints2;
//    //Get KeyPoints and Descriptors

//    keypoints1=*(std::vector<cv::KeyPoint>*)&g_keypoint1;
//    keypoints2=*(std::vector<cv::KeyPoint>*)&g_keypoint2;
//    cv::Mat des1 = lastKF->getDescriptor();
//    cv::Mat des2 = curFrame->getDescriptor();

//    bool succeed=matcher.matchHomography(keypoints1,des1,keypoints2,des2,result_matches,inliers1,inliers2);
//    if(!succeed)
//    {
//        cout<<"matcher failed"<<endl;
//        return false;
//    }
//    lastKF->setKeyPoints(*(std::vector<GSLAM::KeyPoint>*)&inliers1);
//    curFrame->setKeyPoints(*(std::vector<GSLAM::KeyPoint>*)&inliers2);

//    for(int i=0;i<result_matches.size();i++)
//    {
//        cv::DMatch a;
//        matches.push_back(std::pair<int,int>(result_matches[i].queryIdx,result_matches[i].trainIdx));
//    }

//    if(matches.size()>10)
//    {
//        return true;
//    }
//    else
//    {
//        return false;
//    }
    return false;
}

REGISTER_MATCHER(MatcherDIY,UNICORN)
