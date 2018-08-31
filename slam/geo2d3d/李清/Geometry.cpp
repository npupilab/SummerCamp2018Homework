#include <Geometry.h>

class GeometryImpl : public summercamp::Geometry
{
public:

    virtual bool pointOnLine(const GSLAM::Point3d& point,
            const GSLAM::Point3d& line)const
    {
        return point.dot(line);
    }

    virtual GSLAM::Point3D intersect(const GSLAM::Point3d& line1,
                                     const GSLAM::Point3d& line2)const
    {
        return line1.cross(line2);
    }

    virtual GSLAM::Point3d line(const GSLAM::Point3d& pt1,
                                const GSLAM::Point3d& pt2)const
    {
        return pt1.cross(pt2);
    }

    virtual double         distance(const GSLAM::Point3d& point,
                                    const GSLAM::Point3d& line)const
    {
        return fabs(point.dot(line)/sqrt(line.x * line.x + line.y * line.y));
    }

    virtual GSLAM::Point2d transform(double theta,double scale,
                                     const GSLAM::Point2d& translation,
                                     const GSLAM::Point2d& point)const
    {
         double sint = sin(theta);
         double cost = cos(theta);
         double x = scale*cost*point.x-scale*sint*point.y+translation.x;
         double y = scale*sint*point.x+scale*cost*point.y+translation.y;
        return Point2d(x,y);
    }

    virtual GSLAM::Point2d transform(double* H,const GSLAM::Point2d)const
    {
        double x=H[0]*pt.x+H[1]*pt.y+H[2];
        double y=H[3]*pt.x+H[4]*pt.y+H[5];
        double w=H[6]*pt.x+H[7]*pt.y+H[8];
        return Point2d(x/w,y/w);
    }


    virtual GSLAM::Point3d transform(double* H,const GSLAM::Point3d pt)const
    {
        double x=H[0]*pt.x+H[1]*pt.y+H[2]*pt.z+H[3];
        double y=H[4]*pt.x+H[5]*pt.y+H[6]*pt.z+H[7];
        double z=H[8]*pt.x+H[9]*pt.y+H[10]*pt.z+H[11];
        double w=H[12]*pt.x+H[13]*pt.y+H[14]*pt.z+H[15];
        return Point3d(x/w,y/w,z/w);
    }

    virtual GSLAM::Point3d epipolarLine(GSLAM::Camera  cam1,GSLAM::SE3 pose1,
                                        GSLAM::Camera  cam2,GSLAM::SE3 pose2,
                                        GSLAM::Point2d point1)const
    {
        auto pt1=cam2.Project(pose2.inverse()*pose1*(cam1.UnProject(point1)*10));
        auto pt2=cam2.Project(pose2.inverse()*pose1*(cam1.UnProject(point1)*20));
        return line(Point3d(pt1.x,pt1.y,1),Point3d(pt2.x,pt2.y,1));
    }
};

summercamp::GeometryPtr summercamp::Geometry::create()
{
    return summercamp::GeometryPtr(new GeometryImpl());
}
