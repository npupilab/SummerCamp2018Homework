#include <Geometry.h>
#include <cmath>

class GeometryImpl : public summercamp::Geometry {
 public:
  GeometryImpl() : Geometry() {}
  static summercamp::GeometryPtr create();
  virtual bool pointOnLine(const GSLAM::Point3d &point,
                           const GSLAM::Point3d &line) const;
  virtual GSLAM::Point3d intersect(const GSLAM::Point3d &line1,
                                   const GSLAM::Point3d &line2) const;
  virtual GSLAM::Point3d line(const GSLAM::Point3d &pt1,
                              const GSLAM::Point3d &pt2) const;
  virtual double distance(const GSLAM::Point3d &point,
                          const GSLAM::Point3d &line) const;
  // 2D similarity transform
  virtual GSLAM::Point2d transform(double theta, double scale,
                                   const GSLAM::Point2d &translation,
                                   const GSLAM::Point2d &point) const;

  // 2D homography transform, H is 3x3
  virtual GSLAM::Point2d transform(double *H, const GSLAM::Point2d) const;

  // 3D homography transform, H is 4x4
  virtual GSLAM::Point3d transform(double *H, const GSLAM::Point3d) const;

  // Compute the epipolarline as known both Pinhole Camera parameters and camera
  // poses
  virtual GSLAM::Point3d epipolarLine(GSLAM::Camera cam1, GSLAM::SE3 pose1,
                                      GSLAM::Camera cam2, GSLAM::SE3 pose2,
                                      GSLAM::Point2d point1) const;
};

summercamp::GeometryPtr summercamp::Geometry::create() {
  return summercamp::GeometryPtr(new GeometryImpl());
}

summercamp::GeometryPtr GeometryImpl::create(){
  return summercamp::GeometryPtr(new GeometryImpl());
}

bool GeometryImpl::pointOnLine(const GSLAM::Point3d &point,
                               const GSLAM::Point3d &line) const {
  double dot_product =
      point[0] * line[0] + point[1] * line[1] + point[2] * line[2];
  return (dot_product < 1e-6);
}
GSLAM::Point3d GeometryImpl::intersect(const GSLAM::Point3d &line1,
                                       const GSLAM::Point3d &line2) const {
  return GSLAM::Point3d(line1[1] * line2[2] - line1[2] * line2[1],
                        line1[2] * line2[0] - line1[0] * line2[2],
                        line1[0] * line2[1] - line1[1] * line2[0]);
}
GSLAM::Point3d GeometryImpl::line(const GSLAM::Point3d &pt1,
                                  const GSLAM::Point3d &pt2) const {
  return GSLAM::Point3d(pt1[1] * pt2[2] - pt1[2] * pt2[1],
                        pt1[2] * pt2[0] - pt1[0] * pt2[2],
                        pt1[0] * pt2[1] - pt1[1] * pt2[0]);
}

double GeometryImpl::distance(const GSLAM::Point3d &point,
                              const GSLAM::Point3d &line) const {
  double dot_product =
      point[0] * line[0] + point[1] * line[1] + point[2] * line[2];
  double dis =
      std::abs(dot_product) / std::sqrt(line[0] * line[0] + line[1] * line[1]);
  return dis;
}

GSLAM::Point2d GeometryImpl::transform(double theta, double scale,
                                       const GSLAM::Point2d &translation,
                                       const GSLAM::Point2d &point) const {
  double a11 = scale * std::cos(theta), a12 = -scale * std::sin(theta),
         a21 = scale * std::sin(theta), a22 = scale * std::cos(theta);
  double t_x = translation[0];
  double t_y = translation[1];
  GSLAM::Point2d result;
  result[0] = a11 * point[0] + a12 * point[1] + t_x;
  result[1] = a21 * point[0] + a22 * point[1] + t_y;
  return result;
}
GSLAM::Point2d GeometryImpl::transform(double *H,
                                       const GSLAM::Point2d point) const {
  const double &h11 = H[0], &h12 = H[1], &h13 = H[2], &h21 = H[3], &h22 = H[4],
               &h23 = H[5], &h31 = H[6], &h32 = H[7], &h33 = H[8];
  GSLAM::Point3d result;
  result[0] = point[0] * h11 + point[1] * h12 + h13;
  result[1] = point[0] * h21 + point[1] * h22 + h23;
  result[2] = point[0] * h31 + point[1] * h32 + h33;
  return GSLAM::Point2d(result[0] / result[2], result[1] / result[2]);
}

GSLAM::Point3d GeometryImpl::transform(double *H,
                                       const GSLAM::Point3d point) const {
  const double &h11 = H[0], &h12 = H[1], &h13 = H[2], &h14 = H[3], &h21 = H[4],
               &h22 = H[5], &h23 = H[6], &h24 = H[7], &h31 = H[8], &h32 = H[9],
               &h33 = H[10], &h34 = H[11], &h41 = H[12], &h42 = H[13],
               &h43 = H[14], &h44 = H[15];
  GSLAM::Point3d result;
  double result4;
  result[0] = point[0] * h11 + point[1] * h12 + point[2] * h13 + h14;
  result[1] = point[0] * h21 + point[1] * h22 + point[2] * h23 + h24;
  result[2] = point[0] * h31 + point[1] * h32 + point[2] * h33 + h34;
  result4 = point[0] * h41 + point[1] * h42 + point[2] * h43 + h44;
  return result / result4;
}

GSLAM::Point3d GeometryImpl::epipolarLine(GSLAM::Camera cam1, GSLAM::SE3 pose1,
                                          GSLAM::Camera cam2, GSLAM::SE3 pose2,
                                          GSLAM::Point2d point1) const {
        //cam2 is Identity
        // [t]_x is skew-symmetric matrix 
        // (K_1^{-1}p_1)^TR^T[T]_xK_2^{-1}p_2 = 0
        //GSLAM::SE3 trans21=pose2.inverse()*pose1;
        //GSLAM::Point3d line=trans21.get_translation().cross(trans21.get_rotation()*cam1.UnProject(point1));
        //return line;
        GSLAM::SE3 trans21=pose2.inverse()*pose1;
        auto l=trans21.get_translation().cross(trans21.get_rotation()*cam1.UnProject(point1));
        return l;
        //FIXME
}
