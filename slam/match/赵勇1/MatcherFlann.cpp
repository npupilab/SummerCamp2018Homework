#ifdef HAS_OPENCV
#include <opencv2/features2d/features2d.hpp>

#include "GSLAM/core/Estimator.h"
#include <GSLAM/core/Timer.h>
#include <GSLAM/core/Vocabulary.h>

#include "Matcher.h"
using std::vector;

class MatcherFlann : public Matcher
{
public:
    MatcherFlann()
    {
        _estimator=GSLAM::Estimator::create();
        if(!_estimator)
        {
            LOG(ERROR)<<"MatcherBoW failed to create Estimator.";
        }
    }
    virtual bool match4initialize(const GSLAM::FramePtr& lastKF,const GSLAM::FramePtr& curFrame,
                                  std::vector<std::pair<int,int> >& matches)const {
        SCOPE_TIMER

        std::vector<cv::DMatch> matches12,matches21;
        int            nmatches=0;

        const cv::Mat& des1=lastKF->getDescriptor();
        const cv::Mat& des2=curFrame->getDescriptor();
        {
            cv::Ptr<cv::DescriptorMatcher> matcher(new cv::FlannBasedMatcher());
            matcher->match(des1,des2,matches12);
            matcher->match(des2,des1,matches21);
        }
        //block filter
        {
            matches.clear();
            matches.reserve(matches12.size());
            for(auto m:matches12)
            {
                assert(m.queryIdx<des1.rows&&m.trainIdx<des2.rows);
                if(matches21[m.trainIdx].trainIdx==m.queryIdx)
                    matches.push_back(std::pair<int,int>(m.queryIdx,m.trainIdx));
            }
        }

        if(matches.size()<100) return false;

        // filt with fundamental
        vector<u_char>         mask;
        {
            GSLAM::Camera  cam1=lastKF->getCamera();
            GSLAM::Camera  cam2=curFrame->getCamera();
            double F[9];
            vector<GSLAM::Point2d> points1,points2;
            for(auto& m:matches)
            {
                GSLAM::Point2f pt;
                if(!lastKF->getKeyPoint(m.first,pt)) LOG(FATAL)<<"MapFrame::getKeyPoint() not implemented!";
                GSLAM::Point3d p3d=cam1.UnProject(pt.x,pt.y);
                points1.push_back(GSLAM::Point2d(p3d.x,p3d.y));

                if(!curFrame->getKeyPoint(m.second,pt)) LOG(FATAL)<<"MapFrame::getKeyPoint() not implemented!";
                p3d=cam2.UnProject(pt.x,pt.y);
                points2.push_back(GSLAM::Point2d(p3d.x,p3d.y));
            }

            _estimator->findFundamental(F,points1,points2,GSLAM::RANSAC,0.01,0.99,&mask);
        }

        // reduce
        if(mask.size())
        {
            int curSize=0;
            for(size_t i=0;i<matches.size();i++){
                if(mask[i])
                    matches[curSize++]=matches[i];
            }
            matches.resize(curSize);
        }


        return matches.size()>100;
    }

    virtual bool findMatchWindow(const GSLAM::GImage& des,const GSLAM::FramePtr& fr,
                                 const float& x,const float& y,const float& r,
                                 int& idx,bool discardMapPoints=true)const
    {
        std::vector<size_t> candidates=fr->getFeaturesInArea(x,y,r);
        if(candidates.empty()) return false;

        float minDistance=1e8;
        float maxDistance=-1;
        if(maxDistance<0)
        {
            if(des.type()==GSLAM::GImageType<float>::Type&&des.cols==128) maxDistance=0.2;//SIFT
            else if(des.type()==GSLAM::GImageType<uchar>::Type&&des.cols==64) maxDistance=50;//ORB

            assert(maxDistance>0);
        }
        for(size_t& i:candidates)
        {
            if(discardMapPoints&&fr->getKeyPointObserve(i)) continue;
            GSLAM::GImage des1=fr->getDescriptor(i);

            float dis=GSLAM::Vocabulary::distance(des1,des);
            if(dis<minDistance)
            {
                minDistance=dis;
                idx=i;
            }
        }

        if(minDistance<maxDistance) return true;
        else return false;
    }

    SPtr<GSLAM::Estimator> _estimator;

};

REGISTER_MATCHER(MatcherFlann,flann);

#endif
