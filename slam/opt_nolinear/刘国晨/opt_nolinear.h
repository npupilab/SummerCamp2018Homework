//
// Created by liu on 18-9-3.
//

#ifndef PROJECT_OPT_NOLINEAR_H
#define PROJECT_OPT_NOLINEAR_H

#include <GSLAM/core/Optimizer.h>
#include <eigen3/Eigen/Cholesky>

#define RESIDUAL_SIZE 2
#define PARA_SIZE 6

typedef Eigen::Matrix<double, RESIDUAL_SIZE, PARA_SIZE> PnPJacobian;
typedef Eigen::Matrix<double, PARA_SIZE, PARA_SIZE> PnPHessian;
typedef Eigen::Matrix<double, PARA_SIZE, 1> JacobianError;
typedef Eigen::Matrix<double, 2, 3> Matrix23;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

using Eigen::Vector2d;
using GSLAM::SE3;
using GSLAM::Point2d;
using GSLAM::Point3d;

class OptimizerPnP: public GSLAM::Optimizer
{
public:
    virtual bool optimizePnP(const std::vector<std::pair<GSLAM::Point3d, GSLAM::CameraAnchor> > &matches,
                             GSLAM::SE3 &pose, GSLAM::KeyFrameEstimzationDOF dof, double *information)
    {
        std::vector<Point3d> object_points;
        std::vector<Point2d> image_points;
        object_points.reserve(matches.size());
        image_points.reserve(matches.size());

        for (const auto &m:matches)
        {
            object_points.push_back(m.first);
            image_points.push_back(Point2d(m.second.x, m.second.y));
        }

        SE3 world2camera = pose.inverse();
        bool ret = SolvePnP(world2camera, object_points, image_points, GSLAM::Camera({1, 1}));
        pose = world2camera.inverse();

        return ret;
    }

private:
    bool SolvePnP(SE3 &world2camera, const std::vector<Point3d> &object_points,
                  const std::vector<Point2d> &image_points, const GSLAM::Camera &camera);
    double CacuSumError(SE3 &world2camera, const std::vector<Point3d> &object_points,
                        const std::vector<Point2d> &image_points, const GSLAM::Camera &camera,
                        const double &sigma_squared);
    inline double HuberWeight(double error_squared, double sigma_squared)
    {
        if(error_squared < sigma_squared)
            return 1;
        else
            return sqrt(sigma_squared / error_squared);
    }
};

USE_OPTIMIZER_PLUGIN(OptimizerPnP);

#endif //PROJECT_OPT_NOLINEAR_H
