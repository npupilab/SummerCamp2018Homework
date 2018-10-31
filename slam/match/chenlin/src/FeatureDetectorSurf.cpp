#ifdef HAS_OPENCV

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>

#include "FeatureDetector.h"

class FeatureDetectorSurf: public ::FeatureDetector
{
public:
    virtual void operator()( const GSLAM::GImage& image, const GSLAM::GImage& mask,
                             std::vector<GSLAM::KeyPoint>& keypoints,
                             GSLAM::GImage& descriptors,
                             bool useProvidedKeypoints=false ) const
    {
        cv::SurfDescriptorExtractor SurfDescriptor;
        cv::Mat desc;
        SurfDescriptor((cv::Mat)(image),(cv::Mat)(mask),*(std::vector<cv::KeyPoint>*)&keypoints,desc);
        descriptors = desc;
    }

    virtual int descriptorSize()const{return 32;}
    virtual int descriptorType()const{return GSLAM::GImageType<>::Type;}
};


REGISTER_FEATUREDETECTOR(FeatureDetectorSurf,Surf);

#endif // end of HAS_OPENCV
