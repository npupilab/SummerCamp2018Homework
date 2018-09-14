//
// Created by liu on 18-9-3.
//

#include "opt_nolinear.h"

bool OptimizerPnP::SolvePnP(SE3 &world2camera, const std::vector<Point3d> &object_points,
                            const std::vector<Point2d> &image_points, const GSLAM::Camera &camera)
{
    assert(camera.isValid());
    assert(object_points.size() == image_points.size());

    int max_num = 50;
    double lambda = 0.01;
    double sigma_squared = 0.01;
    double lambda_factor = 10;

    GSLAM::SE3 w2c = world2camera;
    double cur_error = CacuSumError(w2c, object_points, image_points, camera, sigma_squared);

    for (int i = 0; i < max_num; ++i)
    {
        PnPHessian hessian = PnPHessian::Identity() * lambda;
        JacobianError jacerr = JacobianError::Zero();

        for (int j = 0; j < object_points.size(); ++j)
        {
            Point3d cam = w2c * object_points[i];
            Point2d err = image_points[i] - camera.Project(cam);
            double info = HuberWeight(err.x * err.x + err.y * err.y, sigma_squared);
            err = err * info;

            Eigen::Vector3d pc(cam.x, cam.y, cam.z);
            PnPJacobian jac;
            double inv_z = 1. / pc(2);
            double inv_z2 = inv_z * inv_z;
            const double &x = pc(0), &y = pc(1);

            jac << inv_z, 0, -x * inv_z2, -x * y * inv_z2, (1 + (x * x * inv_z2)), -y * inv_z,
                   0, inv_z, -y * inv_z2, -(1 + y * y * inv_z2), x * y * inv_z2, x * inv_z;
            hessian += jac.transpose() * info * jac;
            jacerr += jac.transpose() * Vector2d(err.x, err.y);
        }

        Vector6d inc = hessian.ldlt().solve(jacerr);
        SE3 w2c_new = SE3::exp(*(pi::Array_<double, 6>*)&inc) * w2c;
        double error_new = CacuSumError(w2c_new, object_points, image_points, camera, sigma_squared);
        double fac_error = error_new / cur_error;

        if (fac_error >= 1)
        {
            lambda *= lambda_factor;
        }
        else
        {
            w2c = w2c_new;
            cur_error = error_new;
            lambda /= lambda_factor;

            if (fac_error > 0.9999) break;
            if (cur_error < 1e-7) break;
        }
    }

    world2camera = w2c;
    return true;
}

double OptimizerPnP::CacuSumError(SE3 &world2camera, const std::vector<Point3d> &object_points,
                                  const std::vector<Point2d> &image_points, const GSLAM::Camera &camera,
                                  const double &sigma_squared)
{
    double result = 0.0;

    for (int i = 0; i < object_points.size(); ++i)
    {
        Point3d cam = world2camera * object_points[i];
        Point2d err = camera.Project(cam) - image_points[i];
        double info = HuberWeight(err.x * err.x + err.y * err.y, sigma_squared);
        err = err * info;
        result += err.x * err.x + err.y * err.y;
    }

    return result;
}
