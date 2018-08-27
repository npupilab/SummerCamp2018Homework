#include "Matcher.h"
#include "GSLAM/core/Timer.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class MatcherBoW : public Matcher
{
public:
    bool match4initialize(const GSLAM::FramePtr& lastKF,const GSLAM::FramePtr& curFrame,
                                      std::vector<std::pair<int,int> >& matches)const;
};

bool MatcherBoW::match4initialize(const GSLAM::FramePtr& lastKF,const GSLAM::FramePtr& curFrame,
                                  std::vector<std::pair<int,int> >& matches)const
{
    cv::Mat des1 = lastKF->getDescriptor(-1);
    cv::Mat des2 = curFrame->getDescriptor(-1);
    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
    std::vector<cv::DMatch> matches1;

    matcher->match(des1,des2,matches1);

    double min_dist=10000, max_dist=0;
    min_dist = min_element( matches1.begin(), matches1.end(), [](const cv::DMatch& m1, const cv::DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    max_dist = max_element( matches1.begin(), matches1.end(), [](const cv::DMatch& m1, const cv::DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    std::vector< cv::DMatch > good_matches;
    for ( int i = 0; i < des1.rows; i++ )
    {
        if ( matches1[i].distance <= max ( 2*min_dist, 40.0 ) )
        {
            good_matches.push_back ( matches1[i] );
        }
    }

    matches.resize(good_matches.size());
    for (int i=0; i<good_matches.size(); ++i) {
        matches[i].first = good_matches[i].queryIdx;
        matches[i].second = good_matches[i].trainIdx;
    }

    if (matches.size()!=0)
        return true;
    else return false;
}


REGISTER_MATCHER(MatcherBoW,bow);
