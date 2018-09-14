#include <Geometry.h>
#include <stdlib.h>

class GeometryImpl : public summercamp::Geometry
{
public:
    bool pointOnLine(const GSLAM::Point3d& point,
                     const GSLAM::Point3d& line) const
    {
        return (point * line) < 1e-6;
    }

    GSLAM::Point3d intersect(const GSLAM::Point3d& line1,
                             const GSLAM::Point3d& line2) const
    {
        double a1 = line1.x, a2 = line1.y, a3 = line1.z;
        double b1 = line2.x, b2 = line2.y, b3 = line2.z;
        return GSLAM::Point3d((-a3 * b2 + a2 * b3), (a3 * b1 - a1 * b3),
                              (-a2 * b1 + a1 * b2));
    }

    GSLAM::Point3d line(const GSLAM::Point3d& pt1,
                        const GSLAM::Point3d& pt2) const
    {
        double a1 = pt1.x, a2 = pt1.y, a3 = pt1.z;
        double b1 = pt2.x, b2 = pt2.y, b3 = pt2.z;

        return GSLAM::Point3d((-a3 * b2 + a2 * b3), (a3 * b1 - a1 * b3),
                              (-a2 * b1 + a1 * b2));
    }

    double distance(const GSLAM::Point3d& point,const GSLAM::Point3d& line) const
    {
        double a1 = point.x, a2 = point.y, a3 = point.z;
        double b1 = line.x, b2 = line.y, b3 = line.z;

        return fabs((point.dot(line)) / sqrt(b1 * b1 + b2 * b2));
    }

    GSLAM::Point2d transform(double theta,double scale,
                                     const GSLAM::Point2d& translation,
                                     const GSLAM::Point2d& point) const
    {
        double a11 = scale * cos(theta), a12 = scale * -sin(theta), a13 = translation.x;
        double a21 = scale * sin(theta), a22 = scale *  cos(theta), a23 = translation.y;
        double a31 = 1,                  a32 = 1,                   a33 = 1            ;

        double x = (a11 * point.x + a12 * point.y + a13);
        double y = (a21 * point.x + a22 * point.y + a23);
        return GSLAM::Point2d(x, y);
    }

    GSLAM::Point2d transform(double* H,const GSLAM::Point2d pt) const
    {
        double x = H[0] * pt.x + H[1] * pt.y + H[2];
        double y = H[3] * pt.x + H[4] * pt.y + H[5];
        double z = H[6] * pt.x + H[7] * pt.y + H[8];

        return GSLAM::Point2d(x, y);
    }

    GSLAM::Point3d transform(double* H,const GSLAM::Point3d pt) const
    {
        double x = H[0] * pt.x + H[1] * pt.y + H[2] * pt.z + H[3];
        double y = H[4] * pt.x + H[5] * pt.y + H[6] * pt.z + H[7];
        double z = H[8] * pt.x + H[9] * pt.y + H[10] * pt.z + H[11];
        double w = H[12] * pt.x + H[13] * pt.y + H[14] * pt.z + H[15];

        return GSLAM::Point3d(x / w, y / w, z / w);
    }

    GSLAM::Point3d epipolarLine(GSLAM::Camera  cam1, GSLAM::SE3 pose1,
            GSLAM::Camera  cam2, GSLAM::SE3 pose2,
            GSLAM::Point2d point1) const
    {
        GSLAM::SE3 trans21 = pose2.inverse() * pose1;
        auto l = trans21.get_translation().cross(trans21.get_rotation() * cam1.UnProject(point1));

        std::vector<double> param = cam2.getParameters();
        double f_x = param[2], f_y = param[3], c_x = param[4], c_y = param[5];
        double l_x = l[0], l_y = l[1], l_z = l[2];
        double line_x = l_x / f_x;
        double line_y = l_y / f_y;
        double line_z = l_z - (c_x / f_x * l_x + c_y / f_y * l_y);
        return GSLAM::Point3d(line_x, line_y, line_z);
    }
};

summercamp::GeometryPtr summercamp::Geometry::create()
{
    return summercamp::GeometryPtr(new GeometryImpl());
}