#ifndef SUMMERCAMP_CAMERA_H
#define SUMMERCAMP_CAMERA_H
#include <GSLAM/core/Point.h>

namespace summercamp {

class CameraOpenCV {
 public:
  typedef GSLAM::Point2d Point2d;
  typedef GSLAM::Point3d Point3d;
  CameraOpenCV(double Fx, double Fy, double Cx, double Cy, double K1, double K2,
               double P1, double P2, double K3);

  virtual std::string CameraType() const { return "OpenCV"; }
  virtual bool isValid() const;
  virtual Point2d Project(const Point3d& p3d) const;
  virtual Point3d UnProject(const Point2d& p2d) const;
  virtual bool applyScale(double scale = 0.5);
  double fx, fy, cx, cy, k1, k2, p1, p2, k3;
};

}

#endif
