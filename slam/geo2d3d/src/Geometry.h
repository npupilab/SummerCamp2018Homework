#ifndef SUMMERCAMP_GEOMETRY_H
#define SUMMERCAMP_GEOMETRY_H
#include <memory>
#include "Point.h"

namespace summercamp {

class Geometry{
public:
    static std::shared_ptr<Geometry> create();

    virtual ~Geometry(){}

    virtual bool pointOnLine(const GSLAM::Point3d& point,
                             const GSLAM::Point3d& line)const=0;

    virtual GSLAM::Point3d intersect(const GSLAM::Point3d& line1,
                                     const GSLAM::Point3d& line2)const=0;

    virtual GSLAM::Point3d line(const GSLAM::Point3d& pt1,
                                const GSLAM::Point3d& pt2)const=0;

    virtual double         distance(const GSLAM::Point3d& point,
                                    const GSLAM::Point3d& line)const=0;

    // 2D similarity transform
    virtual GSLAM::Point2d transform(double theta,double scale,
                                            const GSLAM::Point2d& translation,
                                            const GSLAM::Point2d& point)const=0;

    // 2D homography transform, H is 3x3
    virtual GSLAM::Point2d transform(double* H,const GSLAM::Point2d)const=0;

    // 3D homography transform, H is 4x4
    virtual GSLAM::Point3d transform(double* H,const GSLAM::Point3d)const=0;
};

typedef std::shared_ptr<Geometry> GeometryPtr;
}

#endif
