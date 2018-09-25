#ifndef _OPT_LINEAR_H
#define _OPT_LINEAR_H


#include <iostream>
#include <vector>
#include <string>

#include <GSLAM/core/Estimator.h>
#include <GSLAM/core/Random.h>
#include <GSLAM/core/Array.h>
#include <Eigen/Dense>


using namespace std;
using namespace Eigen;
using namespace GSLAM;


namespace GSLAM{

    class EstimatorEigen : public Estimator
    {
        public:
          EstimatorEigen() = default;
          ~EstimatorEigen()= default;

        virtual bool findHomography(double *H, const std::vector<Point2d> &srcPoints,
                                    const std::vector<Point2d> &dstPoints, int method,
                                    double ransacReprojThreshold, std::vector<uchar> *mask) const;
        virtual bool findFundamental(double *F, const std::vector<Point2d> &points1,
                                     const std::vector<Point2d> &points2, int method,
                                     double param1, double param2, std::vector<uchar> *mask) const;
        virtual bool findEssentialMatrix(double *E, const std::vector<Point2d> &points1,
                                         const std::vector<Point2d> &points2, int method, double param1,
                                         double param2, std::vector<uchar> *mask) const;
        private:
          void Normalize(const std::vector<Point2d> &input_points, std::vector<Point2d> &output_points, Matrix3d &translation) const;
          Matrix3d ComputeH21(std::vector<Point2d> &src_points, std::vector<Point2d> &dst_points) const;
          double CheckHomography(const Matrix3d &homography21, const Matrix3d &homography12,
                                  const std::vector<Point2d> &src_points, const std::vector<Point2d> &dst_points) const;
          Matrix3d ComputeF21(std::vector<Point2d> &src_points, std::vector<Point2d> &dst_points) const;
          double CheckFundamental(const Matrix3d &fundamental21,
                                 const std::vector<Point2d> &src_points, const std::vector<Point2d> &dst_points) const;
          Matrix3d ComputeE21(const std::vector<Point2d> &src_points, const std::vector<Point2d> & dst_points) const;
     };


} //namespace GSLAM

USE_ESTIMATOR_PLUGIN(EstimatorEigen)

#endif
