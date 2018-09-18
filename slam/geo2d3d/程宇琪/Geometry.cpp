#include <Geometry.h>

class GeometryImpl : public summercamp::Geometry
{
public:
    GeometryImpl() : Geometry() {}

    bool pointOnLine(const GSLAM::Point3d& point,
                                 const GSLAM::Point3d& line) const
    {

        if(fabs(point.dot(line))<1e-6)
            return 1;
        else
            return 0;
    }
    GSLAM::Point3d intersect(const GSLAM::Point3d& line1,
                                         const GSLAM::Point3d& line2) const
    {
        return line1.cross(line2);
    }
    GSLAM::Point3d line(const GSLAM::Point3d& pt1,
                                    const GSLAM::Point3d& pt2) const
    {

        return pt1.cross(pt2);
    }
    double distance(const GSLAM::Point3d& point,const GSLAM::Point3d& line) const
    {
        return fabs(point.dot(line)/sqrt(line.x*line.x+line.y*line.y));;

    }

    // 2D similarity transform
    GSLAM::Point2d transform(double theta,double scale,
                                     const GSLAM::Point2d& translation,
                                     const GSLAM::Point2d& point) const
    {
        double x=scale*(cos(theta)*point.x-sin(theta)*point.y)+translation.x;
        double y=scale*(sin(theta)*point.x+cos(theta)*point.y)+translation.y;
        return GSLAM::Point2d(x,y);
    }

    // 2D homography transform, H is 3x3
    GSLAM::Point2d transform(double* H,const GSLAM::Point2d point) const
    {
        double x=H[0]*point.x+H[1]*point.y+H[2];
        double y=H[3]*point.x+H[4]*point.y+H[5];
        double w=H[6]*point.x+H[7]*point.y+H[8];
        return GSLAM::Point2d(x/w,y/w);
    }

    // 3D homography transform, H is 4x4
    GSLAM::Point3d transform(double* H,const GSLAM::Point3d point) const
    {
        double x=H[0]*point.x+H[1]*point.y+H[2]*point.z+H[3];
        double y=H[4]*point.x+H[5]*point.y+H[6]*point.z+H[7];
        double z=H[8]*point.x+H[9]*point.y+H[10]*point.z+H[11];
        double w=H[12]*point.x+H[13]*point.y+H[14]*point.z+H[15];
        return GSLAM::Point3d(x/w,y/w,z/w);
    }

    // Compute the epipolarline as known both Pinhole Camera parameters and camera poses
    GSLAM::Point3d epipolarLine(GSLAM::Camera  cam1, GSLAM::SE3 pose1,
                                        GSLAM::Camera  cam2, GSLAM::SE3 pose2,
                                        GSLAM::Point2d point1) const
    {
        GSLAM::Point2d pt1=cam2.Project(pose2.inverse()*pose1*cam1.UnProject(point1));
        GSLAM::Point2d pt2=cam2.Project(pose2.inverse()*pose1*(cam1.UnProject(point1)*10));
        return line(GSLAM::Point3d(pt1.x,pt1.y,1),GSLAM::Point3d(pt2.x,pt2.y,1));
    }

};
summercamp::GeometryPtr summercamp::Geometry::create()
{
    return summercamp::GeometryPtr(new GeometryImpl());
}
