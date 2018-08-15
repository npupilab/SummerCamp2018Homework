#if !defined(HAS_OPENCV)&&defined(HAS_EIGEN3)

#include <GSLAM/core/Estimator.h>
#include "Estimators.h"
#include "RANSAC.h"


namespace GSLAM
{

class EstimatorEigen : public Estimator
{
public:
    EstimatorEigen()
    {
    }

    virtual std::string type()const{return "EstimatorEigen";}

    inline std::vector<Eigen::Vector2d> toEigenArray(const std::vector<Point2d>& input)const
    {
        return *(std::vector<Eigen::Vector2d>*)(&input);
    }

    inline std::vector<Eigen::Vector3d> toEigenArray(const std::vector<Point3d>& input)const
    {
        return *(std::vector<Eigen::Vector3d>*)(&input);
    }

    // 2D corrospondences
    virtual bool findHomography(double* H,//3x3 dof=8
                        const std::vector<Point2d>& srcPoints,
                        const std::vector<Point2d>& dstPoints,
                        int method=0, double ransacReprojThreshold=3,
                        std::vector<uchar>* mask=NULL)const{

        ransac::RANSACOptions options;
        options.max_error=ransacReprojThreshold;
        ransac::LORANSAC<HomographyMatrixEstimator,HomographyMatrixEstimator> hransac(options);
        auto report=hransac.Estimate(toEigenArray(srcPoints),toEigenArray(dstPoints));
        if(!report.success) return false;
        Eigen::Map<Eigen::Matrix<double,3,3> > result(H);
        result=report.model.transpose()/report.model(2,2);// Since matrix is row major
        if(mask)
            *mask=report.inlier_mask;
        return report.success;
    }

    virtual bool findFundamental(double* F,//3x3
                         const std::vector<Point2d>& points1,
                         const std::vector<Point2d>& points2,
                         int method=0, double param1=3., double param2=0.99,
                         std::vector<uchar>* mask=NULL)const
    {
        ransac::RANSACOptions options;
        options.max_error=param1;
        options.confidence=param2;
        if(method==GSLAM::FM_7POINT)
        {
            ransac::LORANSAC<FundamentalMatrixSevenPointEstimator,FundamentalMatrixSevenPointEstimator> hransac(options);
            auto report=hransac.Estimate(toEigenArray(points1),toEigenArray(points2));
            if(!report.success) return false;
            Eigen::Map<Eigen::Matrix<double,3,3> > result(F);
            result=report.model.transpose()/report.model(2,2);// Since matrix is row major
            if(mask)
                *mask=report.inlier_mask;
            return report.success;
        }
        else
        {
            ransac::LORANSAC<FundamentalMatrixEightPointEstimator,FundamentalMatrixEightPointEstimator> hransac(options);
            auto report=hransac.Estimate(toEigenArray(points1),toEigenArray(points2));
            if(!report.success) return false;
            Eigen::Map<Eigen::Matrix<double,3,3> > result(F);
            result=report.model.transpose()/report.model(2,2);// Since matrix is row major
            if(mask)
                *mask=report.inlier_mask;
            return report.success;
        }
        return false;
    }

    virtual bool findEssentialMatrix(double* E,//3x3 dof=5
                             const std::vector<Point2d>& points1,
                             const std::vector<Point2d>& points2,
                             int method=0, double param1=0.01, double param2=0.99,
                             std::vector<uchar>* mask=NULL) const{
        ransac::RANSACOptions options;
        options.max_error=param1;
        options.confidence=param2;
        ransac::RANSAC<EssentialMatrixFivePointEstimator> hransac(options);
        auto report=hransac.Estimate(toEigenArray(points1),toEigenArray(points2));
        if(!report.success) return false;
        Eigen::Map<Eigen::Matrix<double,3,3> > result(E);
        result=report.model.transpose()/report.model(2,2);// Since matrix is row major
        if(mask)
            *mask=report.inlier_mask;
        return report.success;
    }

    virtual bool findPnPRansac(SE3 &world2camera, const std::vector<Point3d> &objectPoints,
                               const std::vector<Point2d> &imagePoints,
                               const Camera &camera, bool useExtrinsicGuess, int iterationsCount,
                               float reprojectionError, int minInliersCount, std::vector<int> *inliers, int flags) const
    {
        ransac::RANSACOptions options;
        options.max_error=reprojectionError;
        options.max_num_trials=iterationsCount;
        options.min_inlier_ratio=minInliersCount/objectPoints.size();

        ransac::RANSAC<P3PEstimator> hransac(options);
        auto report=hransac.Estimate(toEigenArray(imagePoints),toEigenArray(objectPoints));
        if(!report.success) return false;
        auto& m=report.model;
        double t[12]={m(0,0),m(0,1),m(0,2),m(0,3),
                      m(1,0),m(1,1),m(1,2),m(1,3),
                      m(2,0),m(2,1),m(2,2),m(2,3)};
        world2camera.fromMatrix(t);
        if(inliers)
        {
            inliers->reserve(objectPoints.size());
            for(int i=0;i<report.inlier_mask.size();i++)
                if(report.inlier_mask[i]) inliers->push_back(i);
        }
        return report.success;
    }

    virtual bool findPlane(SE3 &plane, const std::vector<Point3d> &points, int method, double ransacThreshold, std::vector<uchar> *mask) const{
        ransac::RANSACOptions options;
        options.max_error=ransacThreshold;
        ransac::RANSAC<SE3PlaneEstimator> hransac(options);
        auto report=hransac.Estimate(points,points);
        if(!report.success) return false;
        plane=report.model;
        if(mask)
            *mask=report.inlier_mask;
        return report.success;
    }

    virtual bool trianglate(const SE3 &t21, const Point3d &xn1, const Point3d &xn2, Point3d &pt) const
    {
        double t1[12]={1.,0.,0.,0,
                       0.,1.,0.,0.,
                       0.,0.,1.,0.};
        double t2[12];
        t21.getMatrix(t2);
        Eigen::Matrix4d A;
        A<<xn1[0]*t1[8]-t1[0], xn1[0]*t1[9]-t1[1], xn1[0]*t1[10]-t1[2], xn1[0]*t1[11]-t1[3],
           xn1[1]*t1[8]-t1[4], xn1[1]*t1[9]-t1[5], xn1[1]*t1[10]-t1[6], xn1[1]*t1[11]-t1[7],
           xn2[0]*t2[8]-t2[0], xn2[0]*t2[9]-t2[1], xn2[0]*t2[10]-t2[2], xn2[0]*t2[11]-t2[3],
           xn2[1]*t2[8]-t2[4], xn2[1]*t2[9]-t2[5], xn2[1]*t2[10]-t2[6], xn2[1]*t2[11]-t2[7];

        Eigen::JacobiSVD<Eigen::Matrix4d> svd(A,Eigen::ComputeFullU | Eigen::ComputeFullV);
        auto v=svd.matrixV();
        if(v(3,3)==0) return false;
        Eigen::Vector4d x3D;
        x3D<<v(0,3),v(1,3),v(2,3),v(3,3);


        pt=pi::Point3d(x3D[0]/x3D[3],x3D[1]/x3D[3],x3D[2]/x3D[3]);
        return true;
    }
};

}

USE_ESTIMATOR_PLUGIN(GSLAM::EstimatorEigen);
#endif
