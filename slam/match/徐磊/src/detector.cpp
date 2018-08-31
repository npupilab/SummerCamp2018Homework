#include <GSLAM/core/GSLAM.h>
#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include "FeatureDetector.h"
#include "ORBextractor.h"

using namespace ORB_SLAM2;

class FeatureDetectorMy: public FeatureDetector
{
public:
    FeatureDetectorMy():FeatureDetector(){}
    virtual void operator()( const GSLAM::GImage& image, const GSLAM::GImage& mask,
                             std::vector<GSLAM::KeyPoint>& keypoints,
                             GSLAM::GImage& descriptors,
                             bool useProvidedKeypoints=false ) const;
};

void FeatureDetectorMy::operator ()(const GSLAM::GImage& image, const GSLAM::GImage& mask,
                                    std::vector<GSLAM::KeyPoint>& keypoints,
                                    GSLAM::GImage& descriptors,
                                    bool useProvidedKeypoints) const
{
    ORBextractor  orb(2000,1.2,8,20,7);
    cv::Mat img = image;
    cv::cvtColor(img,img,cv::COLOR_BGR2GRAY);
    cv::Mat descriptors_tmp;
    orb(img,cv::Mat(mask),*(std::vector<cv::KeyPoint>*)&keypoints,
                descriptors_tmp);

    descriptors = descriptors_tmp;
}

REGISTER_FEATUREDETECTOR(FeatureDetectorMy,ZGMF_X10A)
