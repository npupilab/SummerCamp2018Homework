//
// Created by oem on 18-8-28.
//

#include "ORBmatcherHF.h"
#include "ORBmatcherBF.h"
#include <iostream>
using namespace std;
using namespace cv;

bool ORBmatcherHF::matchFundamental(std::vector<cv::KeyPoint> keypoints1, cv::Mat descriptors1,
                                    std::vector<cv::KeyPoint> keypoints2, cv::Mat descriptors2, int distance,
                                    std::vector<cv::DMatch> &good_matches,std::vector<cv::KeyPoint> &inliers1,std::vector<cv::KeyPoint> &inliers2)
{
    ORBmatcherBF matcher;
    vector<DMatch> matches;
    vector<int> index1,index2;
    matcher.match_lowe_ratio(descriptors1,descriptors2,0.7,matches);
    for(auto i:matches)
    {
        index1.push_back(i.queryIdx);
        index2.push_back(i.trainIdx);
    }

    //Compute Fundamental Matrix and inliers
    vector<uchar> mask;
    vector<Point2f> point1,point2;
    KeyPoint::convert(keypoints1,point1,index1);
    KeyPoint::convert(keypoints2,point2,index2);

    assert(point1.size()==index1.size());
    assert(point2.size()==index2.size());

    Mat F_matrix = findFundamentalMat(point1,point2,mask,FM_RANSAC,distance);

    KeyPoint::convert(point1,inliers1);
    KeyPoint::convert(point2,inliers2);
    for (int j = 0; j < inliers1.size() ; ++j) {
        if(mask[j]!=0)
        {
            good_matches.emplace_back(DMatch(j,j,1.0));
        }
    }

    if (good_matches.size() < 10)
    {
        std::cout<<"Matches too less !"<<std::endl;
        return false;
    } else{
        return true;
    }
}

bool ORBmatcherHF::matchHomography(std::vector<cv::KeyPoint> keypoints1, cv::Mat descriptors1,
                                   std::vector<cv::KeyPoint> keypoints2, cv::Mat descriptors2,
                                   std::vector<cv::DMatch> &good_matches, std::vector<cv::KeyPoint> &inliers1,
                                   std::vector<cv::KeyPoint> &inliers2) {
    ORBmatcherBF matcher;
    vector<DMatch> matches;
    vector<int> index1,index2;
    matcher.match_lowe_ratio(descriptors1,descriptors2,0.7,matches);
    for(auto i: matches)
    {
        index1.push_back(i.queryIdx);
        index2.push_back(i.trainIdx);
    }
    vector<uchar> mask;
    vector<Point2f> point1,point2;
    KeyPoint::convert(keypoints1,point1,index1);
    KeyPoint::convert(keypoints2,point2,index2);
    assert(point1.size()==index1.size());
    assert(point2.size()==index2.size());

    Mat H_matrix = findHomography(point1,point2,mask,CV_RANSAC);

    KeyPoint::convert(point1,inliers1);
    KeyPoint::convert(point2,inliers2);
    for (int j = 0; j < inliers1.size() ; ++j) {
        if(mask[j]!=0)
        {
            good_matches.emplace_back(DMatch(j,j,1.0));
        }
    }

    if (good_matches.size() < 10)
    {
        std::cout<<"Matches too less !"<<std::endl;
        return false;
    } else{
        return true;
    }
}



// estimate homography many times ,and get more matches
// Just For Fun!! :) lol
bool ORBmatcherHF::matchMultiHomohraphy(std::vector<cv::KeyPoint> keypoints1, cv::Mat descriptors1,
                          std::vector<cv::KeyPoint> keypoints2, cv::Mat descriptors2,
                          std::vector<cv::DMatch> &good_matches,std::vector<cv::KeyPoint> &inliers1,std::vector<cv::KeyPoint> &inliers2)
{

    //Foolish
    ORBmatcherBF matcher;
    vector<DMatch> matches;
    vector<int> index1,index2;
    matcher.match_lowe_ratio(descriptors1,descriptors2,0.7,matches);
    cout<<"ORBmatcherBF size: "<<matches.size()<<endl;

    for(auto i: matches)
    {
        index1.push_back(i.queryIdx);
        index2.push_back(i.trainIdx);
    }
    vector<Point2f> point1,point2;
    KeyPoint::convert(keypoints1,point1,index1);
    KeyPoint::convert(keypoints2,point2,index2);
    assert(point1.size()==index1.size());
    assert(point2.size()==index2.size());

    bool run_flag = true;

    while(run_flag)
    {
        vector<uchar> mask(point1.size(),0);
        vector<Point2f> point5,point6;
        Mat H_matrix = findHomography(point1,point2,mask,CV_RANSAC);
        for (int i=0; i<point1.size(); i++) {
            if(mask[i] !=0)
            {
                inliers1.emplace_back(KeyPoint(point1[i],2));//set size to 1
                inliers2.emplace_back(KeyPoint(point2[i],2));
                good_matches.emplace_back(DMatch(inliers1.size()-1,inliers1.size()-1,1.0));

            } else{

                point5.emplace_back(point1[i]);
                point6.emplace_back(point2[i]);
            }
        }
        point1=point5;
        point2=point6;
        if(point1.size() < 30)
        {
            run_flag = false;
        }
    }
    cout<<"Multi-Homography Size: "<<inliers1.size()<<endl;

    return true;

}

