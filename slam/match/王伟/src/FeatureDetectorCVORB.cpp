#ifdef HAS_OPENCV
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
//        cv::Mat img_1 = image;
//        std::vector<cv::KeyPoint> keypoints_1;
//        cv::Mat descriptors_1;

//        cv::Ptr<cv::FeatureDetector> dect = cv::FeatureDetector::create("SIFT");
//        dect->detect ( img_1, keypoints_1 );
//        std::cout << 1 << std::endl;
//        cv::Ptr<cv::DescriptorExtractor> descriptor = cv::DescriptorExtractor::create("SIFT");
//        std::cout << 2 << std::endl;
//        descriptor->compute ( img_1, keypoints_1, descriptors_1 );

//        for(int i=0; i<keypoints_1.size(); ++i) {
//            keypoints.push_back(GSLAM::KeyPoint(keypoints_1[i].pt.x,
//                                                keypoints_1[i].pt.y,
//                                                keypoints_1[i].size,
//                                                keypoints_1[i].angle,
//                                                keypoints_1[i].response,
//                                                keypoints_1[i].octave,
//                                                keypoints_1[i].class_id));
//        }
//        descriptors = descriptors_1;

    }
};



REGISTER_FEATUREDETECTOR(FeatureDetectorCVORB,CVORB);


#endif
