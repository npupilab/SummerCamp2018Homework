#include <Geometry.h>

class GeometryImpl : public summercamp::Geometry
{
public:
    bool pointOnLine(const GSLAM::Point3d& point,
                                 const GSLAM::Point3d& line)
    {

        if(point.dot(line)<1e-6)
            return 1;
        else
            return 0;
    }
    GSLAM::Point3d intersect(const GSLAM::Point3d& line1,
                                         const GSLAM::Point3d& line2)
    {
        return line1.cross(line2);
    }
    GSLAM::Point3d line(const GSLAM::Point3d& pt1,
                                    const GSLAM::Point3d& pt2)
    {

        return pt1.cross(pt2);
    }
    double distance(const GSLAM::Point3d& point,const GSLAM::Point3d& line)
    {
        return 0;

    }

    // 2D similarity transform
    GSLAM::Point2d transform(double theta,double scale,
                                     const GSLAM::Point2d& translation,
                                     const GSLAM::Point2d& point)
    {
        return (GSLAM::Point2d)0;
    }

    // 2D homography transform, H is 3x3
    GSLAM::Point2d transform(double* H,const GSLAM::Point2d)
    {
        return ;
    }

    // 3D homography transform, H is 4x4
    GSLAM::Point3d transform(double* H,const GSLAM::Point3d)
    {
        return (GSLAM::Point3d)0;
    }

    // Compute the epipolarline as known both Pinhole Camera parameters and camera poses
    GSLAM::Point3d epipolarLine(GSLAM::Camera  cam1, GSLAM::SE3 pose1,
                                        GSLAM::Camera  cam2, GSLAM::SE3 pose2,
                                        GSLAM::Point2d point1)
    {
        return (GSLAM::Point3d)0;
    }

};
