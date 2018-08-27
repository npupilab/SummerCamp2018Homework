#include <vector>
#include <list>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

#include "FeatureDetector.h"


class FeatureDetectorCVORB: public ::FeatureDetector
{
public:
    virtual void operator()( const GSLAM::GImage& image, const GSLAM::GImage& mask,
                             std::vector<GSLAM::KeyPoint>& keypoints,
                             GSLAM::GImage& descriptors,
                             bool useProvidedKeypoints=false ) const{

        cv::Mat desc;
        cv::ORB orb;
        orb((cv::Mat)(image),(cv::Mat)(mask),
                  *(std::vector<cv::KeyPoint>*)&keypoints,desc,
                  useProvidedKeypoints);
        descriptors=desc;

    }
};



REGISTER_FEATUREDETECTOR(FeatureDetectorCVORB,CVORB);

