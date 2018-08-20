#include <GSLAM/core/GSLAM.h>
#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include "FeatureDetector.h"


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
    cv::ORB orb;
    cv::Mat descriptors_tmp;
    orb(cv::Mat(image),cv::Mat(mask),*(std::vector<cv::KeyPoint>*)&keypoints,
                descriptors_tmp,useProvidedKeypoints);
    descriptors = descriptors_tmp;
}

REGISTER_FEATUREDETECTOR(FeatureDetectorMy,ZGMF_X10A)
