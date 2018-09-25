#include <Matcher.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include "GSLAM/core/Timer.h"
#include "GSLAM/core/Vocabulary.h"
#include "GSLAM/core/Estimator.h"
#include "Estimator.h"

using namespace std;

class MatcherBoW : public Matcher
{
public:
    MatcherBoW()
    {
        estimator = GSLAM::Estimator::create();

        if (!estimator)
        {
            estimator = createEstimatorInstance();

            if (!estimator)
                LOG(ERROR) << "MatcherBoW failed to create Estimator.";
        }
    }

    virtual bool match4initialize(const GSLAM::FramePtr& lastKF,const GSLAM::FramePtr& curFrame,
                                  std::vector<std::pair<int,int> >& matches)const;
    virtual bool match4triangulation(const GSLAM::FramePtr& ref,const GSLAM::FramePtr& cur,
                                     std::vector<std::pair<int,int> >& matches)const;
    virtual bool findMatchWindow(const GSLAM::GImage& des,const GSLAM::FramePtr& fr,
                                 const float& x,const float& y,const float& r,
                                 int& idx,bool discardMapPoints=true)const;
    virtual bool findMatchEpipolarLine(const GSLAM::GImage&  des , const GSLAM::FramePtr& fr ,
                                       const GSLAM::Point3d& line, const float& r ,
                                       int& idx, bool discardMapPoints=true)
    {
        return false;
    }

    double DescriptorDistance(cv::Mat &a, cv::Mat &b) const ;

    SPtr<GSLAM::Estimator> estimator;
};

double MatcherBoW::DescriptorDistance(cv::Mat &a, cv::Mat &b) const
{
    double score = 0;

    for(int i = 0; i < a.cols; ++i)
    {
        score += pow((a.at<uint8_t >(i) - b.at<uint8_t >(i)), 2);
    }

    return sqrt(score);
}

bool MatcherBoW::match4initialize(const GSLAM::FramePtr &lastKF, const GSLAM::FramePtr &curFrame, std::vector<
    std::pair<int, int> > &matches) const
{
    std::vector<int> first_pass_match;
    std::vector<int> first_pass_best_match;
    std::vector<int> first_pass_second_match;

    cv::Mat description1 = lastKF->getDescriptor();
    cv::Mat description2 = curFrame->getDescriptor();

    int description1size = description1.rows;
    int description2size = description2.rows;

    first_pass_match.resize(description1size, -1);
    first_pass_best_match.resize(description1size, 255);
    first_pass_second_match.resize(description1size, 255);

    for(int i = 0; i < description1size; ++i)
    {
        double bestdist = INT32_MAX;

        for(int j = 0; j < description2size; ++j)
        {
            cv::Mat a = description1.row(i);
            cv::Mat b = description2.row(j);

            double dist = DescriptorDistance(a, b);

            if(bestdist > dist)
            {
                bestdist = dist;

                first_pass_second_match[i] = first_pass_best_match[i];
                first_pass_best_match[i] = j;
            }
        }
    }

    for(int i = 0; i < description1size; ++i)
    {
        double first_pass_alpha = 0.9;

        cv::Mat a = description1.row(i);
        cv::Mat b = description2.row(first_pass_best_match[i]);
        cv::Mat c = description2.row(first_pass_second_match[i]);

        double dist = DescriptorDistance(a, b);
        double dist2 = DescriptorDistance(a, c);

        double ratio = (double)(dist) / (double)(dist2);

        if(ratio < first_pass_alpha)
        {
            first_pass_match[i] = first_pass_best_match[i];
            matches.push_back(std::make_pair(i, first_pass_match[i]));
        }
    }

    int num_threshold = std::max(50, int(curFrame->keyPointNum() * 0.03));

    if (matches.size() < num_threshold)
    {
        DLOG(ERROR) << "matches number " << matches.size() << "<" << num_threshold;

        if(svar.GetInt("DebugMatcherBow"))
        {
            cv::Mat image = curFrame->getImage();

            for (int i = 0; i < matches.size(); ++i)
            {
                auto m = matches[i];
                GSLAM::Point2f pt1, pt2;
                lastKF->getKeyPoint(m.first, pt1);
                curFrame->getKeyPoint(m.second, pt2);
                cv::line(image, cv::Point2f(pt1.x, pt1.y), cv::Point2f(pt2.x, pt2.y), cv::Scalar(0, 255, 0), 2);
            }
        }

        return false;
    }

    num_threshold = 50;
    if(matches.size() < num_threshold)
    {
        DLOG(ERROR) << "inlier number " << matches.size() << "<" << num_threshold;
        return false;
    }

    return matches.size() >= num_threshold;
}

bool MatcherBoW::match4triangulation(const GSLAM::FramePtr &ref, const GSLAM::FramePtr &cur, std::vector<
    std::pair<int, int> > &matches) const
{
    GSLAM::SE3 cur2ref = ref->getPose().inverse() * cur->getPose();
    GSLAM::SO3 so3 = cur2ref.get_rotation().inv();
    GSLAM::Point3d t = cur2ref.get_translation();
    GSLAM::FeatureVector feature_vector1, feature_vector2;

    if(!ref->getFeatureVector(feature_vector1))
        return false;
    if(!cur->getFeatureVector(feature_vector2))
        return false;

    GSLAM::FeatureVector::iterator it1  = feature_vector1.begin();
    GSLAM::FeatureVector::iterator it2  = feature_vector2.begin();
    GSLAM::FeatureVector::iterator end1 = feature_vector1.end();
    GSLAM::FeatureVector::iterator end2 = feature_vector2.end();

    float max_distance = -1;
    vector<bool> matched(cur->keyPointNum(), false);

    while (it1 != end1 && it2 != end2)
    {
        if(it1->first == it2->first)
        {
            vector<unsigned int> ids1 = it1->second;
            vector<unsigned int> ids2 = it2->second;

            for(unsigned int i1 : ids1)
            {
                if(ref->getKeyPointObserve(i1))
                    continue;

                vector<std::pair<float, int> > distance_idxs;
                GSLAM::GImage des = ref->getDescriptor(i1);

                distance_idxs.reserve(ids2.size());

                if (max_distance < 0)
                {
                    if (des.type() == GSLAM::GImageType<float>::Type && des.cols == 128)
                        max_distance = 0.2;
                    else if (des.type() == GSLAM::GImageType<uchar>::Type && des.cols == 32)
                        max_distance = 50;

                    assert(max_distance > 0);
                }

                for(unsigned int i2 : ids2)
                {
                    if(matched[i2])
                        continue;
                    if(cur->getKeyPointObserve(i2))
                        continue;

                    auto distance = GSLAM::Vocabulary::distance(des, cur->getDescriptor(i2));

                    if(distance > max_distance)
                        continue;

                    distance_idxs.push_back(std::pair<float, int>(distance, i2));
                }

                if(distance_idxs.empty())
                    continue;

                std::sort(distance_idxs.begin(), distance_idxs.end());

                float best_distance = distance_idxs.front().first;
                float distance_threshold = best_distance * 2;
                GSLAM::Point2f ref_kp;

                if(!ref->getKeyPoint(i1, ref_kp))
                    continue;

                GSLAM::Point3d ref_pt = ref->getCamera().UnProject(ref_kp.x, ref_kp.y);
                GSLAM::Point3d tcrosspl(t.y - t.z * ref_pt.y,
                                        t.z * ref_pt.x - t.x,
                                        t.x * ref_pt.y - t.y * ref_pt.x);
                GSLAM::Point3d line = so3 * tcrosspl;
                double invabsqrt = 1./sqrt(line.x * line.x + line.y * line.y);

                for (auto &m : distance_idxs)
                {
                    if(m.first > distance_threshold)
                    {
                        break;
                    }

                    GSLAM::Point2f cur_kp;
                    cur->getKeyPoint(m.second, cur_kp);
                    GSLAM::Point3d cur_pt = cur->getCamera().UnProject(cur_kp.x, cur_kp.y);
                    double distance = fabs(line.x * cur_pt.x + line.y * cur_pt.y + line.z) * invabsqrt;

                    if(distance > 0.02)
                    {
                        continue;
                    }

                    matches.push_back(std::pair<int, int>(i1, m.second));
                    matched[m.second] = true;
                    break;
                }
            }

            it1++;
            it2++;
        }
        else if(it1->first < it2->first)
        {
            it1 = feature_vector1.lower_bound(it2->first);
        }
        else
        {
            it2 = feature_vector2.lower_bound(it1->first);
        }
    }

    return true;
}

bool MatcherBoW::findMatchWindow(const GSLAM::GImage &des, const GSLAM::FramePtr &fr, const float &x, const float &y,
                                 const float &r, int &idx, bool discardMapPoints) const
{
    std::vector<size_t > candidates = fr->getFeaturesInArea(x, y, r);

    if(candidates.empty())
        return false;

    float min_distance = 1e8, min_distance2 = 1e8;
    float max_distance = -1;

    if(max_distance < 0)
    {
        if(des.type() == GSLAM::GImageType<float>::Type && des.cols == 128)
            max_distance = 0.2;
        else if(des.type() == GSLAM::GImageType<uchar>::Type && des.cols == 32)
            max_distance = 80;

        assert(max_distance > 0);
    }

    for(size_t& i : candidates)
    {
        if(discardMapPoints && fr->getKeyPointObserve(i))
            continue;

        GSLAM::GImage des1 = fr->getDescriptor(i);
        float dis = GSLAM::Vocabulary::distance(des1, des);

        if(dis < min_distance)
        {
            min_distance2 = min_distance;
            min_distance = dis;
            idx = i;
        }
        else if(dis < min_distance2)
        {
            min_distance2 = min_distance;
        }
    }

    if(min_distance < max_distance)
        return true;
    else
        return false;
}

REGISTER_MATCHER(MatcherBoW, bow);
