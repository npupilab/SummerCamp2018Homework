#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>

#include "FeatureDetector.h"

class ORBextractor
{
public:
    ~ORBextractor() {}
};

class FeatureDetectorORB : public ::FeatureDetector
{
public:
    FeatureDetectorORB(){}
    virtual void operator()(const GSLAM::GImage &image, const GSLAM::GImage &mask,
                            std::vector<GSLAM::KeyPoint> &keypoints, GSLAM::GImage &descriptor,
                            bool useProvidedKeypoints = false) const
    {
        cv::ORB orb;
        cv::Mat img = image;
        cv::cvtColor(img,img,cv::COLOR_BGR2GRAY);
        cv::Mat descriptors_tmp;
        orb(img,cv::Mat(mask),*(std::vector<cv::KeyPoint>*)&keypoints,
            descriptors_tmp);

        descriptor = descriptors_tmp;
    }
};

class FeatureDetectorcvORB: public OpenCVDetector
{
public:
    FeatureDetectorcvORB():OpenCVDetector(new cv::ORB()){}
};

REGISTER_FEATUREDETECTOR(FeatureDetectorORB, ORB)
REGISTER_FEATUREDETECTOR(FeatureDetectorcvORB,cvORB);