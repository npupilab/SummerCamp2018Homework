//
// Created by liu on 18-8-29.
//

#ifndef PROJECT_OPT_LINEAR_H
#define PROJECT_OPT_LINEAR_H

#include <iostream>
#include <vector>
#include <GSLAM/core/Estimator.h>
#include <GSLAM/core/Random.h>
#include <GSLAM/core/Array.h>
#include <Eigen/Dense>

using namespace std;
using namespace GSLAM;
using namespace Eigen;

namespace GSLAM
{
    class EstimatorEigen : public Estimator
    {
    public:
        EstimatorEigen()
        {

        }

        virtual bool findHomography(double *H,//3x3 dof=8
                                    const std::vector<Point2d> &srcPoints,
                                    const std::vector<Point2d> &dstPoints,
                                    int method = 0, double ransacReprojThreshold = 3,
                                    std::vector<uchar> *mask = NULL) const;

        virtual bool findFundamental(double *F,//3x3
                                     const std::vector<Point2d> &points1,
                                     const std::vector<Point2d> &points2,
                                     int method = 0, double param1 = 3., double param2 = 0.99,
                                     std::vector<uchar> *mask = NULL) const;

        virtual bool findEssentialMatrix(double* E,  // 3x3 dof=5
                                 const std::vector<Point2d>& points1,
                                 const std::vector<Point2d>& points2,
                                 int method = 0, double param1 = 0.01,
                                 double param2 = 0.99,
                                 std::vector<uchar>* mask = NULL) const;
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
}

    USE_ESTIMATOR_PLUGIN(EstimatorEigen);

#endif //PROJECT_OPT_LINEAR_H
