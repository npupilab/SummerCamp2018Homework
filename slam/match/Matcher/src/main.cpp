#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <GSLAM/core/VecParament.h>
#include <GSLAM/core/Timer.h>

#include "FeatureDetector.h"
#include "Matcher.h"
#include "MapFrame.h"


int Return(int i){
    std::string topic="slam/match";
    std::string name ="demo";
    std::string scoreFile="score.txt";
    int    argc=svar.GetInt("argc");
    char** argv=(char**)svar.GetPointer("argv");
    if(argc>1) topic=argv[1];
    if(argc>2) name =argv[2];
    if(argc>3) scoreFile=argv[3];

    std::vector<std::string> outputInfos={
        "[S]("+topic+"/"+name+")",
        "[C]("+topic+"/evaluation/no_featuredetector.md)",
        "[C]("+topic+"/evaluation/no_matcher.md)",
        "[C]("+topic+"/evaluation/image.md)",
        "[B]("+topic+"/evaluation/keypoints.md)",
        "[B]("+topic+"/evaluation/descriptor.md)",
        "[B]("+topic+"/evaluation/match.md)",
        "[B]("+topic+"/evaluation/few.md)"
    };

    if(argc>3){
        std::ofstream ofs(scoreFile);
        ofs<<outputInfos[i];
    }
    else std::cout<<outputInfos[i];
}

int main(int argc,char** argv){
    svar.ParseMain(argc,argv);
    std::string  featType=svar.GetString("FeatureDetector","Default");
    std::string  matcherType=svar.GetString("Matcher","Default");
    FeatureDetectorPtr featureDetector=FeatureDetector::create(featType);
    MatcherPtr         matcher=Matcher::create(matcherType);

    if(!featureDetector) return Return(1);
    if(!matcher) return Return(2);

    std::string imgFile1=svar.GetString("image1","image1.jpg");
    std::string imgFile2=svar.GetString("image2","image2.jpg");
    GSLAM::GImage image1=cv::imread(imgFile1);
    GSLAM::GImage image2=cv::imread(imgFile2);
    if(image1.empty()||image2.empty()) return Return(3);

    std::vector<GSLAM::KeyPoint> keypoints1,keypoints2;
    GSLAM::GImage                descriptors1,descriptors2;
    (*featureDetector)(image1,GSLAM::GImage(),keypoints1,descriptors1);
    (*featureDetector)(image2,GSLAM::GImage(),keypoints2,descriptors2);

    if(keypoints1.size()<100||keypoints2.size()<100) Return(4);
    if(descriptors1.rows!=keypoints1.size()) Return(5);
    if(descriptors2.rows!=keypoints2.size()) Return(5);

    VecParament<double> campara=svar.get_var("Camera.Paraments",VecParament<double>());
    GSLAM::Camera camera(campara.data);

    SPtr<MapFrame> frame1(new MapFrame(1,0.0,image1,imgFile1,camera,std::vector<double>()));
    SPtr<MapFrame> frame2(new MapFrame(1,1.0,image2,imgFile2,camera,std::vector<double>()));

    frame1->setKeyPoints(keypoints1,descriptors1);
    frame2->setKeyPoints(keypoints2,descriptors2);

    std::vector<std::pair<int,int> > matches;
    GSLAM::TicToc tic;
    tic.Tic();
    if(!matcher->match4initialize(frame1,frame2,matches)) return Return(6);
    double time=tic.Tac();

    if(matches.size()<50) return Return(7);

    std::string img2save=svar.GetString("Image2Save","");
    if(argc<=3||!img2save.empty())
    {
        cv::Mat imgshow=image1;

        GSLAM::KeyPoint kp1,kp2;
        for(std::pair<int,int> m:matches){
            if(!frame1->getKeyPoint(m.first,kp1)) break;
            if(!frame2->getKeyPoint(m.second,kp2)) break;
            cv::line(imgshow,cv::Point(kp1.pt.x,kp1.pt.y),cv::Point(kp2.pt.x,kp2.pt.y),cv::Scalar(255,0,0),imgshow.cols*0.005);
        }
        cv::putText(imgshow,"ProcessTime:"+std::to_string(time),cv::Point(50,50),0,1.,cv::Scalar(0,0,255),2);
        if(argc<=3)
            cv::imshow("match",imgshow);
        else
            cv::imwrite(img2save,imgshow);
    }

    return Return(0);
}
