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

CameraOpenCV::CameraOpenCV(double Fx, double Fy, double Cx, double Cy,
                           double K1, double K2, double P1, double P2,
                           double K3)
    : fx(Fx), fy(Fy), cx(Cx), cy(Cy), k1(K1), k2(K2), p1(P1), p2(P2), k3(K3) {}

bool CameraOpenCV::isValid() const{
    return fx!=0&&fy!=0;
}

GSLAM::Point2d CameraOpenCV::Project(const GSLAM::Point3d& p3d) const {
  double X = p3d.x, Y = p3d.y;
  if (p3d.z != 1.) {
    double z_inv = 1. / p3d.z;
    X *= z_inv;
    Y *= z_inv;
  }

  double r2, r4, r6, X1, Y1, X2, Y2, xy2;
  X2 = X * X;
  Y2 = Y * Y;
  r2 = X2 + Y2;
  r4 = r2 * r2;
  r6 = r2 * r4;
  xy2 = X * Y * 2.0;
  X1 = X * (1. + k1 * r2 + k2 * r4 + k3 * r6) + xy2 * p1 + p2 * (r2 + 2.0 * X2);
  Y1 = Y * (1. + k1 * r2 + k2 * r4 + k3 * r6) + xy2 * p2 + p1 * (r2 + 2.0 * Y2);

  return Point2d(cx + fx * X1, cy + fy * Y1);
}

GSLAM::Point3d CameraOpenCV::UnProject(const GSLAM::Point2d& p2d) const {
  double x = (p2d.x - cx) / fx;
  double y = (p2d.y - cy) / fy;

  // compensate tilt distortion
  double x0 = x;
  double y0 = y;
  // compensate distortion iteratively
  for (int j = 0; j < 5; j++) {
    double r2 = x * x + y * y;
    double icdist = (1) / (1 + ((k3 * r2 + k2) * r2 + k1) * r2);
    double deltaX = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    double deltaY = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
    x = (x0 - deltaX) * icdist;
    y = (y0 - deltaY) * icdist;
  }
  return pi::Point3d(x, y, 1);
}

bool CameraOpenCV::applyScale(double scale) {
  fx = scale * fx;
  fy = scale * fy;
  cx = scale * cx;
  cy = scale * cy;
  return true;
}
}

#endif
