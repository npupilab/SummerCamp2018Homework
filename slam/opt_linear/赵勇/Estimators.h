#include <Eigen/Dense>
#include <GSLAM/core/Glog.h>
#include <GSLAM/core/SIM3.h>

namespace Eigen {

typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;
typedef Eigen::Matrix<uint8_t, 3, 1> Vector3ub;
typedef Eigen::Matrix<uint8_t, 4, 1> Vector4ub;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

}  // namespace Eigen

namespace GSLAM {

// Direct linear transformation algorithm to compute the homography between
// point pairs. This algorithm computes the least squares estimate for
// the homography from at least 4 correspondences.
class HomographyMatrixEstimator {
 public:
  typedef Eigen::Vector2d X_t;
  typedef Eigen::Vector2d Y_t;
  typedef Eigen::Matrix3d M_t;

  // The minimum number of samples needed to estimate a model.
  static const int kMinNumSamples = 4;

  // Estimate the projective transformation (homography).
  //
  // The number of corresponding points must be at least 4.
  //
  // @param points1    First set of corresponding points.
  // @param points2    Second set of corresponding points.
  //
  // @return         3x3 homogeneous transformation matrix.
  static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
                                   const std::vector<Y_t>& points2);

  // Calculate the transformation error for each corresponding point pair.
  //
  // Residuals are defined as the squared transformation error when
  // transforming the source to the destination coordinates.
  //
  // @param points1    First set of corresponding points.
  // @param points2    Second set of corresponding points.
  // @param H          3x3 projective matrix.
  // @param residuals  Output vector of residuals.
  static void Residuals(const std::vector<X_t>& points1,
                        const std::vector<Y_t>& points2, const M_t& H,
                        std::vector<double>* residuals);
};

// Fundamental matrix estimator from corresponding point pairs.
//
// This algorithm solves the 7-Point problem and is based on the following
// paper:
//
//    Zhengyou Zhang and T. Kanade, Determining the Epipolar Geometry and its
//    Uncertainty: A Review, International Journal of Computer Vision, 1998.
//    http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.33.4540
class FundamentalMatrixSevenPointEstimator {
 public:
  typedef Eigen::Vector2d X_t;
  typedef Eigen::Vector2d Y_t;
  typedef Eigen::Matrix3d M_t;

  // The minimum number of samples needed to estimate a model.
  static const int kMinNumSamples = 7;

  // Estimate either 1 or 3 possible fundamental matrix solutions from a set of
  // corresponding points.
  //
  // The number of corresponding points must be exactly 7.
  //
  // @param points1  First set of corresponding points.
  // @param points2  Second set of corresponding points
  //
  // @return         Up to 4 solutions as a vector of 3x3 fundamental matrices.
  static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
                                   const std::vector<Y_t>& points2);

  // Calculate the residuals of a set of corresponding points and a given
  // fundamental matrix.
  //
  // Residuals are defined as the squared Sampson error.
  //
  // @param points1    First set of corresponding points as Nx2 matrix.
  // @param points2    Second set of corresponding points as Nx2 matrix.
  // @param F          3x3 fundamental matrix.
  // @param residuals  Output vector of residuals.
  static void Residuals(const std::vector<X_t>& points1,
                        const std::vector<Y_t>& points2, const M_t& F,
                        std::vector<double>* residuals);
};

// Fundamental matrix estimator from corresponding point pairs.
//
// This algorithm solves the 8-Point problem based on the following paper:
//
//    Hartley and Zisserman, Multiple View Geometry, algorithm 11.1, page 282.
class FundamentalMatrixEightPointEstimator {
 public:
  typedef Eigen::Vector2d X_t;
  typedef Eigen::Vector2d Y_t;
  typedef Eigen::Matrix3d M_t;

  // The minimum number of samples needed to estimate a model.
  static const int kMinNumSamples = 8;

  // Estimate fundamental matrix solutions from a set of corresponding points.
  //
  // The number of corresponding points must be at least 8.
  //
  // @param points1  First set of corresponding points.
  // @param points2  Second set of corresponding points
  //
  // @return         Single solution as a vector of 3x3 fundamental matrices.
  static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
                                   const std::vector<Y_t>& points2);

  // Calculate the residuals of a set of corresponding points and a given
  // fundamental matrix.
  //
  // Residuals are defined as the squared Sampson error.
  //
  // @param points1    First set of corresponding points as Nx2 matrix.
  // @param points2    Second set of corresponding points as Nx2 matrix.
  // @param F          3x3 fundamental matrix.
  // @param residuals  Output vector of residuals.
  static void Residuals(const std::vector<X_t>& points1,
                        const std::vector<Y_t>& points2, const M_t& F,
                        std::vector<double>* residuals);
};

class EssentialMatrixFivePointEstimator {
 public:
  typedef Eigen::Vector2d X_t;
  typedef Eigen::Vector2d Y_t;
  typedef Eigen::Matrix3d M_t;

  // The minimum number of samples needed to estimate a model.
  static const int kMinNumSamples = 5;

  // Estimate up to 10 possible essential matrix solutions from a set of
  // corresponding points.
  //
  //  The number of corresponding points must be at least 5.
  //
  // @param points1  First set of corresponding points.
  // @param points2  Second set of corresponding points.
  //
  // @return         Up to 10 solutions as a vector of 3x3 essential matrices.
  static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
                                   const std::vector<Y_t>& points2);

  // Calculate the residuals of a set of corresponding points and a given
  // essential matrix.
  //
  // Residuals are defined as the squared Sampson error.
  //
  // @param points1    First set of corresponding points.
  // @param points2    Second set of corresponding points.
  // @param E          3x3 essential matrix.
  // @param residuals  Output vector of residuals.
  static void Residuals(const std::vector<X_t>& points1,
                        const std::vector<Y_t>& points2, const M_t& E,
                        std::vector<double>* residuals);
};

// Essential matrix estimator from corresponding normalized point pairs.
//
// This algorithm solves the 8-Point problem based on the following paper:
//
//    Hartley and Zisserman, Multiple View Geometry, algorithm 11.1, page 282.
class EssentialMatrixEightPointEstimator {
 public:
  typedef Eigen::Vector2d X_t;
  typedef Eigen::Vector2d Y_t;
  typedef Eigen::Matrix3d M_t;

  // The minimum number of samples needed to estimate a model.
  static const int kMinNumSamples = 8;

  // Estimate essential matrix solutions from  set of corresponding points.
  //
  // The number of corresponding points must be at least 8.
  //
  // @param points1  First set of corresponding points.
  // @param points2  Second set of corresponding points.
  static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
                                   const std::vector<Y_t>& points2);

  // Calculate the residuals of a set of corresponding points and a given
  // essential matrix.
  //
  // Residuals are defined as the squared Sampson error.
  //
  // @param points1    First set of corresponding points.
  // @param points2    Second set of corresponding points.
  // @param E          3x3 essential matrix.
  // @param residuals  Output vector of residuals.
  static void Residuals(const std::vector<X_t>& points1,
                        const std::vector<Y_t>& points2, const M_t& E,
                        std::vector<double>* residuals);
};


// Analytic solver for the P3P (Perspective-Three-Point) problem.
//
// The algorithm is based on the following paper:
//
//    X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang. Complete Solution
//    Classification for the Perspective-Three-Point Problem.
//    http://www.mmrc.iss.ac.cn/~xgao/paper/ieee.pdf
class P3PEstimator {
 public:
  // The 2D image feature observations.
  typedef Eigen::Vector2d X_t;
  // The observed 3D features in the world frame.
  typedef Eigen::Vector3d Y_t;
  // The transformation from the world to the camera frame.
  typedef Eigen::Matrix3x4d M_t;

  // The minimum number of samples needed to estimate a model.
  static const int kMinNumSamples = 3;

  // Estimate the most probable solution of the P3P problem from a set of
  // three 2D-3D point correspondences.
  //
  // @param points2D   Normalized 2D image points as 3x2 matrix.
  // @param points3D   3D world points as 3x3 matrix.
  //
  // @return           Most probable pose as length-1 vector of a 3x4 matrix.
  static std::vector<M_t> Estimate(const std::vector<X_t>& points2D,
                                   const std::vector<Y_t>& points3D);

  // Calculate the squared reprojection error given a set of 2D-3D point
  // correspondences and a projection matrix.
  //
  // @param points2D     Normalized 2D image points as Nx2 matrix.
  // @param points3D     3D world points as Nx3 matrix.
  // @param proj_matrix  3x4 projection matrix.
  // @param residuals    Output vector of residuals.
  static void Residuals(const std::vector<X_t>& points2D,
                        const std::vector<Y_t>& points3D,
                        const M_t& proj_matrix, std::vector<double>* residuals);
};

// EPNP solver for the PNP (Perspective-N-Point) problem. The solver needs a
// minimum of 4 2D-3D correspondences.
//
// The algorithm is based on the following paper:
//
//    Lepetit, Vincent, Francesc Moreno-Noguer, and Pascal Fua.
//    "Epnp: An accurate o (n) solution to the pnp problem."
//    International journal of computer vision 81.2 (2009): 155-166.
//
// The implementation is based on their original open-source release, but is
// ported to Eigen and contains several improvements over the original code.
class EPNPEstimator {
 public:
  // The 2D image feature observations.
  typedef Eigen::Vector2d X_t;
  // The observed 3D features in the world frame.
  typedef Eigen::Vector3d Y_t;
  // The transformation from the world to the camera frame.
  typedef Eigen::Matrix3x4d M_t;

  // The minimum number of samples needed to estimate a model.
  static const int kMinNumSamples = 4;

  // Estimate the most probable solution of the P3P problem from a set of
  // three 2D-3D point correspondences.
  //
  // @param points2D   Normalized 2D image points as 3x2 matrix.
  // @param points3D   3D world points as 3x3 matrix.
  //
  // @return           Most probable pose as length-1 vector of a 3x4 matrix.
  static std::vector<M_t> Estimate(const std::vector<X_t>& points2D,
                                   const std::vector<Y_t>& points3D);

  // Calculate the squared reprojection error given a set of 2D-3D point
  // correspondences and a projection matrix.
  //
  // @param points2D     Normalized 2D image points as Nx2 matrix.
  // @param points3D     3D world points as Nx3 matrix.
  // @param proj_matrix  3x4 projection matrix.
  // @param residuals    Output vector of residuals.
  static void Residuals(const std::vector<X_t>& points2D,
                        const std::vector<Y_t>& points3D,
                        const M_t& proj_matrix, std::vector<double>* residuals);

 private:
  bool ComputePose(const std::vector<Eigen::Vector2d>& points2D,
                   const std::vector<Eigen::Vector3d>& points3D,
                   Eigen::Matrix3x4d* proj_matrix);

  void ChooseControlPoints();
  bool ComputeBarycentricCoordinates();

  Eigen::Matrix<double, Eigen::Dynamic, 12> ComputeM();
  Eigen::Matrix<double, 6, 10> ComputeL6x10(
      const Eigen::Matrix<double, 12, 12>& Ut);
  Eigen::Matrix<double, 6, 1> ComputeRho();

  void FindBetasApprox1(const Eigen::Matrix<double, 6, 10>& L_6x10,
                        const Eigen::Matrix<double, 6, 1>& rho,
                        Eigen::Vector4d* betas);
  void FindBetasApprox2(const Eigen::Matrix<double, 6, 10>& L_6x10,
                        const Eigen::Matrix<double, 6, 1>& rho,
                        Eigen::Vector4d* betas);
  void FindBetasApprox3(const Eigen::Matrix<double, 6, 10>& L_6x10,
                        const Eigen::Matrix<double, 6, 1>& rho,
                        Eigen::Vector4d* betas);

  void RunGaussNewton(const Eigen::Matrix<double, 6, 10>& L_6x10,
                      const Eigen::Matrix<double, 6, 1>& rho,
                      Eigen::Vector4d* betas);

  double ComputeRT(const Eigen::Matrix<double, 12, 12>& Ut,
                   const Eigen::Vector4d& betas, Eigen::Matrix3d* R,
                   Eigen::Vector3d* t);

  void ComputeCcs(const Eigen::Vector4d& betas,
                  const Eigen::Matrix<double, 12, 12>& Ut);
  void ComputePcs();

  void SolveForSign();

  void EstimateRT(Eigen::Matrix3d* R, Eigen::Vector3d* t);

  double ComputeTotalReprojectionError(const Eigen::Matrix3d& R,
                                       const Eigen::Vector3d& t);

  std::vector<Eigen::Vector2d> points2D_;
  std::vector<Eigen::Vector3d> points3D_;
  std::vector<Eigen::Vector3d> pcs_;
  std::vector<Eigen::Vector4d> alphas_;
  std::array<Eigen::Vector3d, 4> cws_;
  std::array<Eigen::Vector3d, 4> ccs_;
};


class SE3PlaneEstimator {
public:
    typedef pi::Point3d X_t;
    typedef pi::Point3d Y_t;
    typedef pi::SE3d    M_t;

    // The minimum number of samples needed to estimate a model.
    static const int kMinNumSamples = 3;

    // Estimate a plane from set of corresponding points.
    //
    // The number of corresponding points must be at least 3.
    //
    // @param points1  First set of corresponding points.
    static std::vector<M_t> Estimate(const std::vector<X_t>& vv3Inliers,const std::vector<X_t>& vv3Inliers2)
    {
        // With these inliers, calculate mean and cov
        pi::Point3d v3MeanOfInliers(0,0,0);
        for(unsigned int i=0; i<vv3Inliers.size(); i++)
            v3MeanOfInliers=v3MeanOfInliers+vv3Inliers[i];
        v3MeanOfInliers =v3MeanOfInliers*(1.0 / vv3Inliers.size());

        Eigen::Matrix3d A=Eigen::Matrix3d::Zero();
        for(unsigned int i=0; i<vv3Inliers.size(); i++)
        {
            pi::Point3d d = vv3Inliers[i] - v3MeanOfInliers;
            A(0,0)+=d.x*d.x; A(0,1)+=d.x*d.y; A(0,2)+=d.x*d.z;
            A(1,0)+=d.y*d.x; A(1,1)+=d.y*d.y; A(1,2)+=d.y*d.z;
            A(2,0)+=d.z*d.x; A(2,1)+=d.z*d.y; A(2,2)+=d.z*d.z;
        };

        // Solve for the nullspace of the constraint matrix.
        Eigen::JacobiSVD<Eigen::Matrix<double, 3,3> > svd(
            A, Eigen::ComputeFullV);

        const Eigen::Vector3d nullspace = svd.matrixV().col(2);
        auto v3BestNormal=pi::Point3d(nullspace[0],nullspace[1],nullspace[2]);

        pi::Point3d vx,vy,vz;
        if(v3BestNormal.z<0)
            vz=-v3BestNormal;
        else vz=v3BestNormal;
        vx=(vz^pi::Point3d(0,-1,0)).normalize();
        vy=(vz^vx);
        double r[9];
        r[0]=vx.x;r[1]=vy.x;r[2]=vz.x;
        r[3]=vx.y;r[4]=vy.y;r[5]=vz.y;
        r[6]=vx.z;r[7]=vy.z;r[8]=vz.z;
        pi::SE3d se3Aligner;
        se3Aligner.get_translation() = v3MeanOfInliers;
        se3Aligner.get_rotation().fromMatrix(r);
        return std::vector<M_t>({se3Aligner});
    }

    // Calculate the residuals of a set of corresponding points and a given
    // essential matrix.
    //
    // Residuals are defined as the squared Sampson error.
    //
    // @param points1    First set of corresponding points.
    // @param points2    Second set of corresponding points.
    // @param residuals  Output vector of residuals.
    static void Residuals(const std::vector<X_t>& points1,
                          const std::vector<Y_t>& points2, const M_t& plane,
                          std::vector<double>* residuals)
    {
        auto pt2plane=plane.inverse();
        residuals->resize(points1.size());
        for(int i=0;i<points1.size();++i){
            double e=(pt2plane*points1[i])[2];
            residuals->at(i)=e*e;
        }
    }
};

//class Sim3Estimator{
//public:
//    typedef pi::Point3d X_t;
//    typedef pi::Point3d Y_t;
//    typedef pi::SIM3d   M_t;

//    // The minimum number of samples needed to estimate a model.
//    static const int kMinNumSamples = 3;

//    // Estimate a plane from set of corresponding points.
//    //
//    // The number of corresponding points must be at least 3.
//    //
//    // @param points1  First set of corresponding points.
//    static std::vector<M_t> Estimate(const std::vector<X_t>& from,const std::vector<Y_t>& to)
//    {
//        // Custom implementation of:
//        // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions
//        // Y=M_t*X

//        /// 1. Compute the centre of two point set and translate points to centre

//        pi::Point3d centre_Track(0,0,0);
//        pi::Point3d centre_GPS(0,0,0);
//        size_t Num=from.size();

//        for(size_t i=0;i<Num;i++)
//        {
//            centre_Track=centre_Track+from[i];
//            centre_GPS  =centre_GPS+to[i];
//        }
//        centre_Track=centre_Track/(double)Num;
//        centre_GPS=centre_GPS/(double)Num;

//        Eigen::Matrix<double,3,Eigen::Dynamic> X(3,Num);
//        Eigen::Matrix<double,3,Eigen::Dynamic> Y(3,Num);
//        for(size_t i=0;i<Num;i++)
//        {
//            pi::Point3d pt1=from[i];
//            pi::Point3d pt2=to[i];
//            pt1=pt1-centre_Track;
//            pt2=pt2-centre_GPS;
//            X.block(i,0,1,3)=Eigen::Vector3d(pt1.x,pt1.y,pt1.z);
//            Y.block(i,0,1,3)=Eigen::Vector3d(pt2.x,pt2.y,pt2.z);
//        }

//        /// 2. Compute M ,N matrix

//        Eigen::Matrix3d M = Y*X.transpose();

//        double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

//        Eigen::Matrix4d N;

//        N11 = M(0,0)+M(1,1)+M(2,2);
//        N12 = M(1,2)-M(2,1);
//        N13 = M(2,0)-M(0,2);
//        N14 = M(0,1)-M(1,0);
//        N22 = M(0,0)-M(1,1)-M(2,2);
//        N23 = M(0,1)+M(1,0);
//        N24 = M(2,0)+M(0,2);
//        N33 = -M(0,0)+M(1,1)-M(2,2);
//        N34 = M(1,2)+M(2,1);
//        N44 = -M(0,0)-M(1,1)+M(2,2);

//        N  <<   N11, N12, N13, N14,
//                N12, N22, N23, N24,
//                N13, N23, N33, N34,
//                N14, N24, N34, N44;


//        /// 3. Get rotation from eigenvector of the highest eigenvalue
//        Eigen::EigenSolver<Eigen::Matrix4d > eigen(N);

//        const Eigen::Vector3d highestVec = eigen.pseudoEigenvectors().block(0,0,3,1);
//        pi::SO3d Rxy(highestVec[0],highestVec[1],highestVec[2],highestVec[3]);

//        /// 4: Rotate set 2 and compute scale
//        Eigen::Matrix3d mRxy;
//        Rxy.getMatrixUnsafe(mRxy);

//        auto Y_=mRxy*Y;
//        double XY=X.dot(Y_);
//        double YY=Y_.dot(Y_);

//        double scale = YY/XY;

////        /// 5. Compute translation and get SIM3

//        auto Ryx=Rxy.inv();
//        pi::Point3d translation = centre_GPS - (Ryx*centre_Track)*scale;

//        return {pi::SIM3d(Ryx,translation,scale)};
//    }

//    // Calculate the residuals of a set of corresponding points and a given
//    // essential matrix.
//    //
//    // Residuals are defined as the squared Sampson error.
//    //
//    // @param points1    First set of corresponding points.
//    // @param points2    Second set of corresponding points.
//    // @param residuals  Output vector of residuals.
//    static void Residuals(const std::vector<X_t>& points1,
//                          const std::vector<Y_t>& points2, const M_t& plane,
//                          std::vector<double>* residuals)
//    {
//        residuals->resize(points1.size());
//        for(int i=0;i<points1.size();++i){
//            double e=(plane.inverse()*points1[i])[2];
//            residuals->at(i)=e*e;
//        }
//    }
//};

void CenterAndNormalizeImagePoints(const std::vector<Eigen::Vector2d>& points,
                                   std::vector<Eigen::Vector2d>* normed_points,
                                   Eigen::Matrix3d* matrix) {
  // Calculate centroid
  Eigen::Vector2d centroid(0, 0);
  for (const auto point : points) {
    centroid += point;
  }
  centroid /= points.size();

  // Root mean square error to centroid of all points
  double rms_mean_dist = 0;
  for (const auto point : points) {
    rms_mean_dist += (point - centroid).squaredNorm();
  }
  rms_mean_dist = std::sqrt(rms_mean_dist / points.size());

  // Compose normalization matrix
  const double norm_factor = std::sqrt(2.0) / rms_mean_dist;
  *matrix << norm_factor, 0, -norm_factor * centroid(0), 0, norm_factor,
      -norm_factor * centroid(1), 0, 0, 1;

  // Apply normalization matrix
  normed_points->resize(points.size());

  const double M_00 = (*matrix)(0, 0);
  const double M_01 = (*matrix)(0, 1);
  const double M_02 = (*matrix)(0, 2);
  const double M_10 = (*matrix)(1, 0);
  const double M_11 = (*matrix)(1, 1);
  const double M_12 = (*matrix)(1, 2);
  const double M_20 = (*matrix)(2, 0);
  const double M_21 = (*matrix)(2, 1);
  const double M_22 = (*matrix)(2, 2);

  for (size_t i = 0; i < points.size(); ++i) {
    const double p_0 = points[i](0);
    const double p_1 = points[i](1);

    const double np_0 = M_00 * p_0 + M_01 * p_1 + M_02;
    const double np_1 = M_10 * p_0 + M_11 * p_1 + M_12;
    const double np_2 = M_20 * p_0 + M_21 * p_1 + M_22;

    const double inv_np_2 = 1.0 / np_2;
    (*normed_points)[i](0) = np_0 * inv_np_2;
    (*normed_points)[i](1) = np_1 * inv_np_2;
  }
}


// Remove leading zero coefficients.
Eigen::VectorXd RemoveLeadingZeros(const Eigen::VectorXd& coeffs) {
  Eigen::VectorXd::Index num_zeros = 0;
  for (; num_zeros < coeffs.size(); ++num_zeros) {
    if (coeffs(num_zeros) != 0) {
      break;
    }
  }
  return coeffs.tail(coeffs.size() - num_zeros);
}

// Remove trailing zero coefficients.
Eigen::VectorXd RemoveTrailingZeros(const Eigen::VectorXd& coeffs) {
  Eigen::VectorXd::Index num_zeros = 0;
  for (; num_zeros < coeffs.size(); ++num_zeros) {
    if (coeffs(coeffs.size() - 1 - num_zeros) != 0) {
      break;
    }
  }
  return coeffs.head(coeffs.size() - num_zeros);
}


bool FindLinearPolynomialRoots(const Eigen::VectorXd& coeffs,
                               Eigen::VectorXd* real, Eigen::VectorXd* imag) {
  CHECK_EQ(coeffs.size(), 2);

  if (coeffs(0) == 0) {
    return false;
  }

  if (real != nullptr) {
    real->resize(1);
    (*real)(0) = -coeffs(1) / coeffs(0);
  }

  if (imag != nullptr) {
    imag->resize(1);
    (*imag)(0) = 0;
  }

  return true;
}

bool FindQuadraticPolynomialRoots(const Eigen::VectorXd& coeffs,
                                  Eigen::VectorXd* real,
                                  Eigen::VectorXd* imag) {
  CHECK_EQ(coeffs.size(), 3);

  const double a = coeffs(0);
  if (a == 0) {
    return FindLinearPolynomialRoots(coeffs.tail(2), real, imag);
  }

  const double b = coeffs(1);
  const double c = coeffs(2);
  if (b == 0 && c == 0) {
    if (real != nullptr) {
      real->resize(1);
      (*real)(0) = 0;
    }
    if (imag != nullptr) {
      imag->resize(1);
      (*imag)(0) = 0;
    }
    return true;
  }

  const double d = b * b - 4 * a * c;

  if (d >= 0) {
    const double sqrt_d = std::sqrt(d);
    if (real != nullptr) {
      real->resize(2);
      if (b >= 0) {
        (*real)(0) = (-b - sqrt_d) / (2 * a);
        (*real)(1) = (2 * c) / (-b - sqrt_d);
      } else {
        (*real)(0) = (2 * c) / (-b + sqrt_d);
        (*real)(1) = (-b + sqrt_d) / (2 * a);
      }
    }
    if (imag != nullptr) {
      imag->resize(2);
      imag->setZero();
    }
  } else {
    if (real != nullptr) {
      real->resize(2);
      real->setConstant(-b / (2 * a));
    }
    if (imag != nullptr) {
      imag->resize(2);
      (*imag)(0) = std::sqrt(-d) / (2 * a);
      (*imag)(1) = -(*imag)(0);
    }
  }

  return true;
}

bool FindPolynomialRootsDurandKerner(const Eigen::VectorXd& coeffs_all,
                                     Eigen::VectorXd* real,
                                     Eigen::VectorXd* imag) {
  CHECK_GE(coeffs_all.size(), 2);

  const Eigen::VectorXd coeffs = RemoveLeadingZeros(coeffs_all);

  const int degree = coeffs.size() - 1;

  if (degree <= 0) {
    return false;
  } else if (degree == 1) {
    return FindLinearPolynomialRoots(coeffs, real, imag);
  } else if (degree == 2) {
    return FindQuadraticPolynomialRoots(coeffs, real, imag);
  }

  // Initialize roots.
  Eigen::VectorXcd roots(degree);
  roots(degree - 1) = std::complex<double>(1, 0);
  for (int i = degree - 2; i >= 0; --i) {
    roots(i) = roots(i + 1) * std::complex<double>(1, 1);
  }

  // Iterative solver.
  const int kMaxNumIterations = 100;
  const double kMaxRootChange = 1e-10;
  for (int iter = 0; iter < kMaxNumIterations; ++iter) {
    double max_root_change = 0.0;
    for (int i = 0; i < degree; ++i) {
      const std::complex<double> root_i = roots(i);
      std::complex<double> numerator = coeffs[0];
      std::complex<double> denominator = coeffs[0];
      for (int j = 0; j < degree; ++j) {
        numerator = numerator * root_i + coeffs[j + 1];
        if (i != j) {
          denominator = denominator * (root_i - roots(j));
        }
      }
      const std::complex<double> root_i_change = numerator / denominator;
      roots(i) = root_i - root_i_change;
      max_root_change =
          std::max(max_root_change, std::abs(root_i_change.real()));
      max_root_change =
          std::max(max_root_change, std::abs(root_i_change.imag()));
    }

    // Break, if roots do not change anymore.
    if (max_root_change < kMaxRootChange) {
      break;
    }
  }

  if (real != nullptr) {
    real->resize(degree);
    *real = roots.real();
  }
  if (imag != nullptr) {
    imag->resize(degree);
    *imag = roots.imag();
  }

  return true;
}

bool FindPolynomialRootsCompanionMatrix(const Eigen::VectorXd& coeffs_all,
                                        Eigen::VectorXd* real,
                                        Eigen::VectorXd* imag) {
  CHECK_GE(coeffs_all.size(), 2);

  Eigen::VectorXd coeffs = RemoveLeadingZeros(coeffs_all);

  const int degree = coeffs.size() - 1;

  if (degree <= 0) {
    return false;
  } else if (degree == 1) {
    return FindLinearPolynomialRoots(coeffs, real, imag);
  } else if (degree == 2) {
    return FindQuadraticPolynomialRoots(coeffs, real, imag);
  }

  // Remove the coefficients where zero is a solution.
  coeffs = RemoveTrailingZeros(coeffs);

  // Check if only zero is a solution.
  if (coeffs.size() == 1) {
    if (real != nullptr) {
      real->resize(1);
      (*real)(0) = 0;
    }
    if (imag != nullptr) {
      imag->resize(1);
      (*imag)(0) = 0;
    }
    return true;
  }

  // Fill the companion matrix.
  Eigen::MatrixXd C(coeffs.size() - 1, coeffs.size() - 1);
  C.setZero();
  for (Eigen::MatrixXd::Index i = 1; i < C.rows(); ++i) {
    C(i, i - 1) = 1;
  }
  C.row(0) = -coeffs.tail(coeffs.size() - 1) / coeffs(0);

  // Solve for the roots of the polynomial.
  Eigen::EigenSolver<Eigen::MatrixXd> solver(C, false);
  if (solver.info() != Eigen::Success) {
    return false;
  }

  // If there are trailing zeros, we must add zero as a solution.
  const int effective_degree =
      coeffs.size() - 1 < degree ? coeffs.size() : coeffs.size() - 1;

  if (real != nullptr) {
    real->resize(effective_degree);
    real->head(coeffs.size() - 1) = solver.eigenvalues().real();
    if (effective_degree > coeffs.size() - 1) {
      (*real)(real->size() - 1) = 0;
    }
  }
  if (imag != nullptr) {
    imag->resize(effective_degree);
    imag->head(coeffs.size() - 1) = solver.eigenvalues().imag();
    if (effective_degree > coeffs.size() - 1) {
      (*imag)(imag->size() - 1) = 0;
    }
  }

  return true;
}

void ComputeSquaredSampsonError(const std::vector<Eigen::Vector2d>& points1,
                                const std::vector<Eigen::Vector2d>& points2,
                                const Eigen::Matrix3d& E,
                                std::vector<double>* residuals) {
  CHECK_EQ(points1.size(), points2.size());

  residuals->resize(points1.size());

  // Note that this code might not be as nice as Eigen expressions,
  // but it is significantly faster in various tests

  const double E_00 = E(0, 0);
  const double E_01 = E(0, 1);
  const double E_02 = E(0, 2);
  const double E_10 = E(1, 0);
  const double E_11 = E(1, 1);
  const double E_12 = E(1, 2);
  const double E_20 = E(2, 0);
  const double E_21 = E(2, 1);
  const double E_22 = E(2, 2);

  for (size_t i = 0; i < points1.size(); ++i) {
    const double x1_0 = points1[i](0);
    const double x1_1 = points1[i](1);
    const double x2_0 = points2[i](0);
    const double x2_1 = points2[i](1);

    // Ex1 = E * points1[i].homogeneous();
    const double Ex1_0 = E_00 * x1_0 + E_01 * x1_1 + E_02;
    const double Ex1_1 = E_10 * x1_0 + E_11 * x1_1 + E_12;
    const double Ex1_2 = E_20 * x1_0 + E_21 * x1_1 + E_22;

    // Etx2 = E.transpose() * points2[i].homogeneous();
    const double Etx2_0 = E_00 * x2_0 + E_10 * x2_1 + E_20;
    const double Etx2_1 = E_01 * x2_0 + E_11 * x2_1 + E_21;

    // x2tEx1 = points2[i].homogeneous().transpose() * Ex1;
    const double x2tEx1 = x2_0 * Ex1_0 + x2_1 * Ex1_1 + Ex1_2;

    // Sampson distance
    (*residuals)[i] =
        x2tEx1 * x2tEx1 /
        (Ex1_0 * Ex1_0 + Ex1_1 * Ex1_1 + Etx2_0 * Etx2_0 + Etx2_1 * Etx2_1);
  }
}

void ComputeSquaredReprojectionError(
    const std::vector<Eigen::Vector2d>& points2D,
    const std::vector<Eigen::Vector3d>& points3D,
    const Eigen::Matrix3x4d& proj_matrix, std::vector<double>* residuals) {
  CHECK_EQ(points2D.size(), points3D.size());

  residuals->resize(points2D.size());

  // Note that this code might not be as nice as Eigen expressions,
  // but it is significantly faster in various tests.

  const double P_00 = proj_matrix(0, 0);
  const double P_01 = proj_matrix(0, 1);
  const double P_02 = proj_matrix(0, 2);
  const double P_03 = proj_matrix(0, 3);
  const double P_10 = proj_matrix(1, 0);
  const double P_11 = proj_matrix(1, 1);
  const double P_12 = proj_matrix(1, 2);
  const double P_13 = proj_matrix(1, 3);
  const double P_20 = proj_matrix(2, 0);
  const double P_21 = proj_matrix(2, 1);
  const double P_22 = proj_matrix(2, 2);
  const double P_23 = proj_matrix(2, 3);

  for (size_t i = 0; i < points2D.size(); ++i) {
    const double X_0 = points3D[i](0);
    const double X_1 = points3D[i](1);
    const double X_2 = points3D[i](2);

    // Project 3D point from world to camera.
    const double px_2 = P_20 * X_0 + P_21 * X_1 + P_22 * X_2 + P_23;

    // Check if 3D point is in front of camera.
    if (px_2 > std::numeric_limits<double>::epsilon()) {
      const double px_0 = P_00 * X_0 + P_01 * X_1 + P_02 * X_2 + P_03;
      const double px_1 = P_10 * X_0 + P_11 * X_1 + P_12 * X_2 + P_13;

      const double x_0 = points2D[i](0);
      const double x_1 = points2D[i](1);

      const double inv_px_2 = 1.0 / px_2;
      const double dx_0 = x_0 - px_0 * inv_px_2;
      const double dx_1 = x_1 - px_1 * inv_px_2;

      (*residuals)[i] = dx_0 * dx_0 + dx_1 * dx_1;
    } else {
      (*residuals)[i] = std::numeric_limits<double>::max();
    }
  }
}


std::vector<HomographyMatrixEstimator::M_t> HomographyMatrixEstimator::Estimate(
    const std::vector<X_t>& points1, const std::vector<Y_t>& points2) {
  CHECK_EQ(points1.size(), points2.size());

  const size_t N = points1.size();

  // Center and normalize image points for better numerical stability.
  std::vector<X_t> normed_points1;
  std::vector<Y_t> normed_points2;
  Eigen::Matrix3d points1_norm_matrix;
  Eigen::Matrix3d points2_norm_matrix;
  CenterAndNormalizeImagePoints(points1, &normed_points1, &points1_norm_matrix);
  CenterAndNormalizeImagePoints(points2, &normed_points2, &points2_norm_matrix);

  // Setup constraint matrix.
  Eigen::Matrix<double, Eigen::Dynamic, 9> A = Eigen::MatrixXd::Zero(2 * N, 9);

  for (size_t i = 0, j = N; i < points1.size(); ++i, ++j) {
    const double s_0 = normed_points1[i](0);
    const double s_1 = normed_points1[i](1);
    const double d_0 = normed_points2[i](0);
    const double d_1 = normed_points2[i](1);

    A(i, 0) = -s_0;
    A(i, 1) = -s_1;
    A(i, 2) = -1;
    A(i, 6) = s_0 * d_0;
    A(i, 7) = s_1 * d_0;
    A(i, 8) = d_0;

    A(j, 3) = -s_0;
    A(j, 4) = -s_1;
    A(j, 5) = -1;
    A(j, 6) = s_0 * d_1;
    A(j, 7) = s_1 * d_1;
    A(j, 8) = d_1;
  }

  // Solve for the nullspace of the constraint matrix.
  Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> svd(
      A, Eigen::ComputeFullV);

  const Eigen::VectorXd nullspace = svd.matrixV().col(8);
  Eigen::Map<const Eigen::Matrix3d> H_t(nullspace.data());

  const std::vector<M_t> models = {points2_norm_matrix.inverse() *
                                   H_t.transpose() * points1_norm_matrix};
  return models;
}

void HomographyMatrixEstimator::Residuals(const std::vector<X_t>& points1,
                                          const std::vector<Y_t>& points2,
                                          const M_t& H,
                                          std::vector<double>* residuals) {
  CHECK_EQ(points1.size(), points2.size());

  residuals->resize(points1.size());

  // Note that this code might not be as nice as Eigen expressions,
  // but it is significantly faster in various tests.

  const double H_00 = H(0, 0);
  const double H_01 = H(0, 1);
  const double H_02 = H(0, 2);
  const double H_10 = H(1, 0);
  const double H_11 = H(1, 1);
  const double H_12 = H(1, 2);
  const double H_20 = H(2, 0);
  const double H_21 = H(2, 1);
  const double H_22 = H(2, 2);

  for (size_t i = 0; i < points1.size(); ++i) {
    const double s_0 = points1[i](0);
    const double s_1 = points1[i](1);
    const double d_0 = points2[i](0);
    const double d_1 = points2[i](1);

    const double pd_0 = H_00 * s_0 + H_01 * s_1 + H_02;
    const double pd_1 = H_10 * s_0 + H_11 * s_1 + H_12;
    const double pd_2 = H_20 * s_0 + H_21 * s_1 + H_22;

    const double inv_pd_2 = 1.0 / pd_2;
    const double dd_0 = d_0 - pd_0 * inv_pd_2;
    const double dd_1 = d_1 - pd_1 * inv_pd_2;

    (*residuals)[i] = dd_0 * dd_0 + dd_1 * dd_1;
  }
}



std::vector<FundamentalMatrixSevenPointEstimator::M_t>
FundamentalMatrixSevenPointEstimator::Estimate(
    const std::vector<X_t>& points1, const std::vector<Y_t>& points2) {
  CHECK_EQ(points1.size(), 7);
  CHECK_EQ(points2.size(), 7);

  // Note that no normalization of the points is necessary here.

  // Setup system of equations: [points2(i,:), 1]' * F * [points1(i,:), 1]'.
  Eigen::Matrix<double, 7, 9> A;
  for (size_t i = 0; i < 7; ++i) {
    const double x0 = points1[i](0);
    const double y0 = points1[i](1);
    const double x1 = points2[i](0);
    const double y1 = points2[i](1);
    A(i, 0) = x1 * x0;
    A(i, 1) = x1 * y0;
    A(i, 2) = x1;
    A(i, 3) = y1 * x0;
    A(i, 4) = y1 * y0;
    A(i, 5) = y1;
    A(i, 6) = x0;
    A(i, 7) = y0;
    A(i, 8) = 1;
  }

  // 9 unknowns with 7 equations, so we have 2D null space.
  Eigen::JacobiSVD<Eigen::Matrix<double, 7, 9>> svd(A, Eigen::ComputeFullV);
  const Eigen::Matrix<double, 9, 9> f = svd.matrixV();
  Eigen::Matrix<double, 1, 9> f1 = f.col(7);
  Eigen::Matrix<double, 1, 9> f2 = f.col(8);

  f1 -= f2;

  // Normalize, such that lambda + mu = 1
  // and add constraint det(F) = det(lambda * f1 + (1 - lambda) * f2).

  const double t0 = f1(4) * f1(8) - f1(5) * f1(7);
  const double t1 = f1(3) * f1(8) - f1(5) * f1(6);
  const double t2 = f1(3) * f1(7) - f1(4) * f1(6);
  const double t3 = f2(4) * f2(8) - f2(5) * f2(7);
  const double t4 = f2(3) * f2(8) - f2(5) * f2(6);
  const double t5 = f2(3) * f2(7) - f2(4) * f2(6);

  Eigen::Vector4d coeffs;
  coeffs(0) = f1(0) * t0 - f1(1) * t1 + f1(2) * t2;
  coeffs(1) = f2(0) * t0 - f2(1) * t1 + f2(2) * t2 -
              f2(3) * (f1(1) * f1(8) - f1(2) * f1(7)) +
              f2(4) * (f1(0) * f1(8) - f1(2) * f1(6)) -
              f2(5) * (f1(0) * f1(7) - f1(1) * f1(6)) +
              f2(6) * (f1(1) * f1(5) - f1(2) * f1(4)) -
              f2(7) * (f1(0) * f1(5) - f1(2) * f1(3)) +
              f2(8) * (f1(0) * f1(4) - f1(1) * f1(3));
  coeffs(2) = f1(0) * t3 - f1(1) * t4 + f1(2) * t5 -
              f1(3) * (f2(1) * f2(8) - f2(2) * f2(7)) +
              f1(4) * (f2(0) * f2(8) - f2(2) * f2(6)) -
              f1(5) * (f2(0) * f2(7) - f2(1) * f2(6)) +
              f1(6) * (f2(1) * f2(5) - f2(2) * f2(4)) -
              f1(7) * (f2(0) * f2(5) - f2(2) * f2(3)) +
              f1(8) * (f2(0) * f2(4) - f2(1) * f2(3));
  coeffs(3) = f2(0) * t3 - f2(1) * t4 + f2(2) * t5;

  Eigen::VectorXd roots_real;
  Eigen::VectorXd roots_imag;
  if (!FindPolynomialRootsCompanionMatrix(coeffs, &roots_real, &roots_imag)) {
    return {};
  }

  std::vector<M_t> models;
  models.reserve(roots_real.size());

  for (Eigen::VectorXd::Index i = 0; i < roots_real.size(); ++i) {
    const double kMaxRootImag = 1e-10;
    if (std::abs(roots_imag(i)) > kMaxRootImag) {
      continue;
    }

    const double lambda = roots_real(i);
    const double mu = 1;

    Eigen::MatrixXd F = lambda * f1 + mu * f2;

    F.resize(3, 3);

    const double kEps = 1e-10;
    if (std::abs(F(2, 2)) < kEps) {
      continue;
    }

    F /= F(2, 2);

    models.push_back(F.transpose());
  }

  return models;
}


void FundamentalMatrixSevenPointEstimator::Residuals(
    const std::vector<X_t>& points1, const std::vector<Y_t>& points2,
    const M_t& F, std::vector<double>* residuals) {
  ComputeSquaredSampsonError(points1, points2, F, residuals);
}

std::vector<FundamentalMatrixEightPointEstimator::M_t>
FundamentalMatrixEightPointEstimator::Estimate(
    const std::vector<X_t>& points1, const std::vector<Y_t>& points2) {
  CHECK_EQ(points1.size(), points2.size());

  // Center and normalize image points for better numerical stability.
  std::vector<X_t> normed_points1;
  std::vector<Y_t> normed_points2;
  Eigen::Matrix3d points1_norm_matrix;
  Eigen::Matrix3d points2_norm_matrix;
  CenterAndNormalizeImagePoints(points1, &normed_points1, &points1_norm_matrix);
  CenterAndNormalizeImagePoints(points2, &normed_points2, &points2_norm_matrix);

  // Setup homogeneous linear equation as x2' * F * x1 = 0.
  Eigen::Matrix<double, Eigen::Dynamic, 9> cmatrix(points1.size(), 9);
  for (size_t i = 0; i < points1.size(); ++i) {
    cmatrix.block<1, 3>(i, 0) = normed_points1[i].homogeneous();
    cmatrix.block<1, 3>(i, 0) *= normed_points2[i].x();
    cmatrix.block<1, 3>(i, 3) = normed_points1[i].homogeneous();
    cmatrix.block<1, 3>(i, 3) *= normed_points2[i].y();
    cmatrix.block<1, 3>(i, 6) = normed_points1[i].homogeneous();
  }

  // Solve for the nullspace of the constraint matrix.
  Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> cmatrix_svd(
      cmatrix, Eigen::ComputeFullV);
  const Eigen::VectorXd cmatrix_nullspace = cmatrix_svd.matrixV().col(8);
  const Eigen::Map<const Eigen::Matrix3d> ematrix_t(cmatrix_nullspace.data());

  // Enforcing the internal constraint that two singular values must non-zero
  // and one must be zero.
  Eigen::JacobiSVD<Eigen::Matrix3d> fmatrix_svd(
      ematrix_t.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector3d singular_values = fmatrix_svd.singularValues();
  singular_values(2) = 0.0;
  const Eigen::Matrix3d F = fmatrix_svd.matrixU() *
                            singular_values.asDiagonal() *
                            fmatrix_svd.matrixV().transpose();

  const std::vector<M_t> models = {points2_norm_matrix.transpose() * F *
                                   points1_norm_matrix};
  return models;
}

void FundamentalMatrixEightPointEstimator::Residuals(
    const std::vector<X_t>& points1, const std::vector<Y_t>& points2,
    const M_t& E, std::vector<double>* residuals) {
  ComputeSquaredSampsonError(points1, points2, E, residuals);
}
std::vector<EssentialMatrixFivePointEstimator::M_t>
EssentialMatrixFivePointEstimator::Estimate(const std::vector<X_t>& points1,
                                            const std::vector<Y_t>& points2) {
  CHECK_EQ(points1.size(), points2.size());

  // Step 1: Extraction of the nullspace x, y, z, w.

  Eigen::Matrix<double, Eigen::Dynamic, 9> Q(points1.size(), 9);
  for (size_t i = 0; i < points1.size(); ++i) {
    const double x1_0 = points1[i](0);
    const double x1_1 = points1[i](1);
    const double x2_0 = points2[i](0);
    const double x2_1 = points2[i](1);
    Q(i, 0) = x1_0 * x2_0;
    Q(i, 1) = x1_1 * x2_0;
    Q(i, 2) = x2_0;
    Q(i, 3) = x1_0 * x2_1;
    Q(i, 4) = x1_1 * x2_1;
    Q(i, 5) = x2_1;
    Q(i, 6) = x1_0;
    Q(i, 7) = x1_1;
    Q(i, 8) = 1;
  }

  // Extract the 4 Eigen vectors corresponding to the smallest singular values.
  const Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> svd(
      Q, Eigen::ComputeFullV);
  const Eigen::Matrix<double, 9, 4> E = svd.matrixV().block<9, 4>(0, 5);

  // Step 3: Gauss-Jordan elimination with partial pivoting on A.

  Eigen::Matrix<double, 10, 20> A;
#include "essential_matrix_poly.h"
  Eigen::Matrix<double, 10, 10> AA =
      A.block<10, 10>(0, 0).partialPivLu().solve(A.block<10, 10>(0, 10));

  // Step 4: Expansion of the determinant polynomial of the 3x3 polynomial
  //         matrix B to obtain the tenth degree polynomial.

  Eigen::Matrix<double, 13, 3> B;
  for (size_t i = 0; i < 3; ++i) {
    B(0, i) = 0;
    B(4, i) = 0;
    B(8, i) = 0;
    B.block<3, 1>(1, i) = AA.block<1, 3>(i * 2 + 4, 0);
    B.block<3, 1>(5, i) = AA.block<1, 3>(i * 2 + 4, 3);
    B.block<4, 1>(9, i) = AA.block<1, 4>(i * 2 + 4, 6);
    B.block<3, 1>(0, i) -= AA.block<1, 3>(i * 2 + 5, 0);
    B.block<3, 1>(4, i) -= AA.block<1, 3>(i * 2 + 5, 3);
    B.block<4, 1>(8, i) -= AA.block<1, 4>(i * 2 + 5, 6);
  }

  // Step 5: Extraction of roots from the degree 10 polynomial.
  Eigen::Matrix<double, 11, 1> coeffs;
#include "essential_matrix_coeffs.h"

  Eigen::VectorXd roots_real;
  Eigen::VectorXd roots_imag;
  if (!FindPolynomialRootsCompanionMatrix(coeffs, &roots_real, &roots_imag)) {
    return {};
  }

  std::vector<M_t> models;
  models.reserve(roots_real.size());

  for (Eigen::VectorXd::Index i = 0; i < roots_imag.size(); ++i) {
    const double kMaxRootImag = 1e-10;
    if (std::abs(roots_imag(i)) > kMaxRootImag) {
      continue;
    }

    const double z1 = roots_real(i);
    const double z2 = z1 * z1;
    const double z3 = z2 * z1;
    const double z4 = z3 * z1;

    Eigen::Matrix3d Bz;
    for (size_t j = 0; j < 3; ++j) {
      Bz(j, 0) = B(0, j) * z3 + B(1, j) * z2 + B(2, j) * z1 + B(3, j);
      Bz(j, 1) = B(4, j) * z3 + B(5, j) * z2 + B(6, j) * z1 + B(7, j);
      Bz(j, 2) = B(8, j) * z4 + B(9, j) * z3 + B(10, j) * z2 + B(11, j) * z1 +
                 B(12, j);
    }

    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(Bz, Eigen::ComputeFullV);
    const Eigen::Vector3d X = svd.matrixV().block<3, 1>(0, 2);

    const double kMaxX3 = 1e-10;
    if (std::abs(X(2)) < kMaxX3) {
      continue;
    }

    Eigen::MatrixXd essential_vec = E.col(0) * (X(0) / X(2)) +
                                    E.col(1) * (X(1) / X(2)) + E.col(2) * z1 +
                                    E.col(3);
    essential_vec /= essential_vec.norm();

    const Eigen::Matrix3d essential_matrix =
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            essential_vec.data());
    models.push_back(essential_matrix);
  }

  return models;
}

void EssentialMatrixFivePointEstimator::Residuals(
    const std::vector<X_t>& points1, const std::vector<Y_t>& points2,
    const M_t& E, std::vector<double>* residuals) {
  ComputeSquaredSampsonError(points1, points2, E, residuals);
}

std::vector<EssentialMatrixEightPointEstimator::M_t>
EssentialMatrixEightPointEstimator::Estimate(const std::vector<X_t>& points1,
                                             const std::vector<Y_t>& points2) {
  CHECK_EQ(points1.size(), points2.size());

  // Center and normalize image points for better numerical stability.
  std::vector<X_t> normed_points1;
  std::vector<Y_t> normed_points2;
  Eigen::Matrix3d points1_norm_matrix;
  Eigen::Matrix3d points2_norm_matrix;
  CenterAndNormalizeImagePoints(points1, &normed_points1, &points1_norm_matrix);
  CenterAndNormalizeImagePoints(points2, &normed_points2, &points2_norm_matrix);

  // Setup homogeneous linear equation as x2' * F * x1 = 0.
  Eigen::Matrix<double, Eigen::Dynamic, 9> cmatrix(points1.size(), 9);
  for (size_t i = 0; i < points1.size(); ++i) {
    cmatrix.block<1, 3>(i, 0) = normed_points1[i].homogeneous();
    cmatrix.block<1, 3>(i, 0) *= normed_points2[i].x();
    cmatrix.block<1, 3>(i, 3) = normed_points1[i].homogeneous();
    cmatrix.block<1, 3>(i, 3) *= normed_points2[i].y();
    cmatrix.block<1, 3>(i, 6) = normed_points1[i].homogeneous();
  }

  // Solve for the nullspace of the constraint matrix.
  Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> cmatrix_svd(
      cmatrix, Eigen::ComputeFullV);
  const Eigen::VectorXd ematrix_nullspace = cmatrix_svd.matrixV().col(8);
  const Eigen::Map<const Eigen::Matrix3d> ematrix_t(ematrix_nullspace.data());

  // Enforcing the internal constraint that two singular values must be equal
  // and one must be zero.
  Eigen::JacobiSVD<Eigen::Matrix3d> ematrix_svd(
      ematrix_t.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector3d singular_values = ematrix_svd.singularValues();
  singular_values(0) = (singular_values(0) + singular_values(1)) / 2.0;
  singular_values(1) = singular_values(0);
  singular_values(2) = 0.0;
  const Eigen::Matrix3d E = ematrix_svd.matrixU() *
                            singular_values.asDiagonal() *
                            ematrix_svd.matrixV().transpose();

  const std::vector<M_t> models = {points2_norm_matrix.transpose() * E *
                                   points1_norm_matrix};
  return models;
}

void EssentialMatrixEightPointEstimator::Residuals(
    const std::vector<X_t>& points1, const std::vector<Y_t>& points2,
    const M_t& E, std::vector<double>* residuals) {
  ComputeSquaredSampsonError(points1, points2, E, residuals);
}

namespace {

Eigen::Vector3d LiftImagePoint(const Eigen::Vector2d& point) {
  return point.homogeneous() / std::sqrt(point.squaredNorm() + 1);
}

}  // namespace

std::vector<P3PEstimator::M_t> P3PEstimator::Estimate(
    const std::vector<X_t>& points2D, const std::vector<Y_t>& points3D) {
  CHECK_EQ(points2D.size(), 3);
  CHECK_EQ(points3D.size(), 3);

  Eigen::Matrix3d points3D_world;
  points3D_world.col(0) = points3D[0];
  points3D_world.col(1) = points3D[1];
  points3D_world.col(2) = points3D[2];

  const Eigen::Vector3d u = LiftImagePoint(points2D[0]);
  const Eigen::Vector3d v = LiftImagePoint(points2D[1]);
  const Eigen::Vector3d w = LiftImagePoint(points2D[2]);

  // Angles between 2D points.
  const double cos_uv = u.transpose() * v;
  const double cos_uw = u.transpose() * w;
  const double cos_vw = v.transpose() * w;

  // Distances between 2D points.
  const double dist_AB_2 = (points3D[0] - points3D[1]).squaredNorm();
  const double dist_AC_2 = (points3D[0] - points3D[2]).squaredNorm();
  const double dist_BC_2 = (points3D[1] - points3D[2]).squaredNorm();

  const double dist_AB = std::sqrt(dist_AB_2);

  const double a = dist_BC_2 / dist_AB_2;
  const double b = dist_AC_2 / dist_AB_2;

  // Helper variables for calculation of coefficients.
  const double a2 = a * a;
  const double b2 = b * b;
  const double p = 2 * cos_vw;
  const double q = 2 * cos_uw;
  const double r = 2 * cos_uv;
  const double p2 = p * p;
  const double p3 = p2 * p;
  const double q2 = q * q;
  const double r2 = r * r;
  const double r3 = r2 * r;
  const double r4 = r3 * r;
  const double r5 = r4 * r;

  // Build polynomial coefficients: a4*x^4 + a3*x^3 + a2*x^2 + a1*x + a0 = 0.
  Eigen::Matrix<double, 5, 1> coeffs;
  coeffs(0) = -2 * b + b2 + a2 + 1 + a * b * (2 - r2) - 2 * a;
  coeffs(1) = -2 * q * a2 - r * p * b2 + 4 * q * a + (2 * q + p * r) * b +
              (r2 * q - 2 * q + r * p) * a * b - 2 * q;
  coeffs(2) = (2 + q2) * a2 + (p2 + r2 - 2) * b2 - (4 + 2 * q2) * a -
              (p * q * r + p2) * b - (p * q * r + r2) * a * b + q2 + 2;
  coeffs(3) = -2 * q * a2 - r * p * b2 + 4 * q * a +
              (p * r + q * p2 - 2 * q) * b + (r * p + 2 * q) * a * b - 2 * q;
  coeffs(4) = a2 + b2 - 2 * a + (2 - p2) * b - 2 * a * b + 1;

  Eigen::VectorXd roots_real;
  Eigen::VectorXd roots_imag;
  if (!FindPolynomialRootsCompanionMatrix(coeffs, &roots_real, &roots_imag)) {
    return std::vector<P3PEstimator::M_t>({});
  }

  std::vector<M_t> models;
  models.reserve(roots_real.size());

  for (Eigen::VectorXd::Index i = 0; i < roots_real.size(); ++i) {
    const double kMaxRootImag = 1e-10;
    if (std::abs(roots_imag(i)) > kMaxRootImag) {
      continue;
    }

    const double x = roots_real(i);
    if (x < 0) {
      continue;
    }

    const double x2 = x * x;
    const double x3 = x2 * x;

    // Build polynomial coefficients: b1*y + b0 = 0.
    const double bb1 =
        (p2 - p * q * r + r2) * a + (p2 - r2) * b - p2 + p * q * r - r2;
    const double b1 = b * bb1 * bb1;
    const double b0 =
        ((1 - a - b) * x2 + (a - 1) * q * x - a + b + 1) *
        (r3 * (a2 + b2 - 2 * a - 2 * b + (2 - r2) * a * b + 1) * x3 +
         r2 *
             (p + p * a2 - 2 * r * q * a * b + 2 * r * q * b - 2 * r * q -
              2 * p * a - 2 * p * b + p * r2 * b + 4 * r * q * a +
              q * r3 * a * b - 2 * r * q * a2 + 2 * p * a * b + p * b2 -
              r2 * p * b2) *
             x2 +
         (r5 * (b2 - a * b) - r4 * p * q * b +
          r3 * (q2 - 4 * a - 2 * q2 * a + q2 * a2 + 2 * a2 - 2 * b2 + 2) +
          r2 * (4 * p * q * a - 2 * p * q * a * b + 2 * p * q * b - 2 * p * q -
                2 * p * q * a2) +
          r * (p2 * b2 - 2 * p2 * b + 2 * p2 * a * b - 2 * p2 * a + p2 +
               p2 * a2)) *
             x +
         (2 * p * r2 - 2 * r3 * q + p3 - 2 * p2 * q * r + p * q2 * r2) * a2 +
         (p3 - 2 * p * r2) * b2 +
         (4 * q * r3 - 4 * p * r2 - 2 * p3 + 4 * p2 * q * r - 2 * p * q2 * r2) *
             a +
         (-2 * q * r3 + p * r4 + 2 * p2 * q * r - 2 * p3) * b +
         (2 * p3 + 2 * q * r3 - 2 * p2 * q * r) * a * b + p * q2 * r2 -
         2 * p2 * q * r + 2 * p * r2 + p3 - 2 * r3 * q);

    // Solve for y.
    const double y = b0 / b1;
    const double y2 = y * y;

    const double nu = x2 + y2 - 2 * x * y * cos_uv;

    const double dist_PC = dist_AB / std::sqrt(nu);
    const double dist_PB = y * dist_PC;
    const double dist_PA = x * dist_PC;

    Eigen::Matrix3d points3D_camera;
    points3D_camera.col(0) = u * dist_PA;  // A'
    points3D_camera.col(1) = v * dist_PB;  // B'
    points3D_camera.col(2) = w * dist_PC;  // C'

    // Find transformation from the world to the camera system.
    const Eigen::Matrix4d transform =
        Eigen::umeyama(points3D_world, points3D_camera, false);
    models.push_back(transform.topLeftCorner<3, 4>());
  }

  return models;
}

void P3PEstimator::Residuals(const std::vector<X_t>& points2D,
                             const std::vector<Y_t>& points3D,
                             const M_t& proj_matrix,
                             std::vector<double>* residuals) {
  ComputeSquaredReprojectionError(points2D, points3D, proj_matrix, residuals);
}

std::vector<EPNPEstimator::M_t> EPNPEstimator::Estimate(
    const std::vector<X_t>& points2D, const std::vector<Y_t>& points3D) {
  CHECK_GE(points2D.size(), 4);
  CHECK_EQ(points2D.size(), points3D.size());

  EPNPEstimator epnp;
  M_t proj_matrix;
  if (!epnp.ComputePose(points2D, points3D, &proj_matrix)) {
    return std::vector<EPNPEstimator::M_t>({});
  }

  return std::vector<EPNPEstimator::M_t>({proj_matrix});
}

void EPNPEstimator::Residuals(const std::vector<X_t>& points2D,
                              const std::vector<Y_t>& points3D,
                              const M_t& proj_matrix,
                              std::vector<double>* residuals) {
  ComputeSquaredReprojectionError(points2D, points3D, proj_matrix, residuals);
}

bool EPNPEstimator::ComputePose(const std::vector<Eigen::Vector2d>& points2D,
                                const std::vector<Eigen::Vector3d>& points3D,
                                Eigen::Matrix3x4d* proj_matrix) {
  points2D_ = points2D;
  points3D_ = points3D;

  ChooseControlPoints();

  if (!ComputeBarycentricCoordinates()) {
    return false;
  }

  const Eigen::Matrix<double, Eigen::Dynamic, 12> M = ComputeM();
  const Eigen::Matrix<double, 12, 12> MtM = M.transpose() * M;

  Eigen::JacobiSVD<Eigen::Matrix<double, 12, 12>> svd(
      MtM, Eigen::ComputeFullV | Eigen::ComputeFullU);
  const Eigen::Matrix<double, 12, 12> Ut = svd.matrixU().transpose();

  const Eigen::Matrix<double, 6, 10> L6x10 = ComputeL6x10(Ut);
  const Eigen::Matrix<double, 6, 1> rho = ComputeRho();

  Eigen::Vector4d betas[4];
  std::array<double, 4> reproj_errors;
  std::array<Eigen::Matrix3d, 4> Rs;
  std::array<Eigen::Vector3d, 4> ts;

  FindBetasApprox1(L6x10, rho, &betas[1]);
  RunGaussNewton(L6x10, rho, &betas[1]);
  reproj_errors[1] = ComputeRT(Ut, betas[1], &Rs[1], &ts[1]);

  FindBetasApprox2(L6x10, rho, &betas[2]);
  RunGaussNewton(L6x10, rho, &betas[2]);
  reproj_errors[2] = ComputeRT(Ut, betas[2], &Rs[2], &ts[2]);

  FindBetasApprox3(L6x10, rho, &betas[3]);
  RunGaussNewton(L6x10, rho, &betas[3]);
  reproj_errors[3] = ComputeRT(Ut, betas[3], &Rs[3], &ts[3]);

  int best_idx = 1;
  if (reproj_errors[2] < reproj_errors[1]) {
    best_idx = 2;
  }
  if (reproj_errors[3] < reproj_errors[best_idx]) {
    best_idx = 3;
  }

  proj_matrix->leftCols<3>() = Rs[best_idx];
  proj_matrix->rightCols<1>() = ts[best_idx];

  return true;
}

void EPNPEstimator::ChooseControlPoints() {
  // Take C0 as the reference points centroid:
  cws_[0].setZero();
  for (size_t i = 0; i < points3D_.size(); ++i) {
    cws_[0] += points3D_[i];
  }
  cws_[0] /= points3D_.size();

  Eigen::Matrix<double, Eigen::Dynamic, 3> PW0(points3D_.size(), 3);
  for (size_t i = 0; i < points3D_.size(); ++i) {
    PW0.row(i) = points3D_[i] - cws_[0];
  }

  const Eigen::Matrix3d PW0tPW0 = PW0.transpose() * PW0;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      PW0tPW0, Eigen::ComputeFullV | Eigen::ComputeFullU);
  const Eigen::Vector3d D = svd.singularValues();
  const Eigen::Matrix3d Ut = svd.matrixU().transpose();

  for (int i = 1; i < 4; ++i) {
    const double k = std::sqrt(D(i - 1) / points3D_.size());
    cws_[i] = cws_[0] + k * Ut.row(i - 1).transpose();
  }
}

bool EPNPEstimator::ComputeBarycentricCoordinates() {
  Eigen::Matrix3d CC;
  for (int i = 0; i < 3; ++i) {
    for (int j = 1; j < 4; ++j) {
      CC(i, j - 1) = cws_[j][i] - cws_[0][i];
    }
  }

  if (CC.colPivHouseholderQr().rank() < 3) {
    return false;
  }

  const Eigen::Matrix3d CC_inv = CC.inverse();

  alphas_.resize(points2D_.size());
  for (size_t i = 0; i < points3D_.size(); ++i) {
    for (int j = 0; j < 3; ++j) {
      alphas_[i][1 + j] = CC_inv(j, 0) * (points3D_[i][0] - cws_[0][0]) +
                          CC_inv(j, 1) * (points3D_[i][1] - cws_[0][1]) +
                          CC_inv(j, 2) * (points3D_[i][2] - cws_[0][2]);
    }
    alphas_[i][0] = 1.0 - alphas_[i][1] - alphas_[i][2] - alphas_[i][3];
  }

  return true;
}

Eigen::Matrix<double, Eigen::Dynamic, 12> EPNPEstimator::ComputeM() {
  Eigen::Matrix<double, Eigen::Dynamic, 12> M(2 * points2D_.size(), 12);
  for (size_t i = 0; i < points3D_.size(); ++i) {
    for (size_t j = 0; j < 4; ++j) {
      M(2 * i, 3 * j) = alphas_[i][j];
      M(2 * i, 3 * j + 1) = 0.0;
      M(2 * i, 3 * j + 2) = -alphas_[i][j] * points2D_[i].x();

      M(2 * i + 1, 3 * j) = 0.0;
      M(2 * i + 1, 3 * j + 1) = alphas_[i][j];
      M(2 * i + 1, 3 * j + 2) = -alphas_[i][j] * points2D_[i].y();
    }
  }
  return M;
}

Eigen::Matrix<double, 6, 10> EPNPEstimator::ComputeL6x10(
    const Eigen::Matrix<double, 12, 12>& Ut) {
  Eigen::Matrix<double, 6, 10> L6x10;

  std::array<std::array<Eigen::Vector3d, 6>, 4> dv;
  for (int i = 0; i < 4; ++i) {
    int a = 0, b = 1;
    for (int j = 0; j < 6; ++j) {
      dv[i][j][0] = Ut(11 - i, 3 * a) - Ut(11 - i, 3 * b);
      dv[i][j][1] = Ut(11 - i, 3 * a + 1) - Ut(11 - i, 3 * b + 1);
      dv[i][j][2] = Ut(11 - i, 3 * a + 2) - Ut(11 - i, 3 * b + 2);

      b += 1;
      if (b > 3) {
        a += 1;
        b = a + 1;
      }
    }
  }

  for (int i = 0; i < 6; ++i) {
    L6x10(i, 0) = dv[0][i].transpose() * dv[0][i];
    L6x10(i, 1) = 2.0 * dv[0][i].transpose() * dv[1][i];
    L6x10(i, 2) = dv[1][i].transpose() * dv[1][i];
    L6x10(i, 3) = 2.0 * dv[0][i].transpose() * dv[2][i];
    L6x10(i, 4) = 2.0 * dv[1][i].transpose() * dv[2][i];
    L6x10(i, 5) = dv[2][i].transpose() * dv[2][i];
    L6x10(i, 6) = 2.0 * dv[0][i].transpose() * dv[3][i];
    L6x10(i, 7) = 2.0 * dv[1][i].transpose() * dv[3][i];
    L6x10(i, 8) = 2.0 * dv[2][i].transpose() * dv[3][i];
    L6x10(i, 9) = dv[3][i].transpose() * dv[3][i];
  }

  return L6x10;
}

Eigen::Matrix<double, 6, 1> EPNPEstimator::ComputeRho() {
  Eigen::Matrix<double, 6, 1> rho;
  rho[0] = (cws_[0] - cws_[1]).squaredNorm();
  rho[1] = (cws_[0] - cws_[2]).squaredNorm();
  rho[2] = (cws_[0] - cws_[3]).squaredNorm();
  rho[3] = (cws_[1] - cws_[2]).squaredNorm();
  rho[4] = (cws_[1] - cws_[3]).squaredNorm();
  rho[5] = (cws_[2] - cws_[3]).squaredNorm();
  return rho;
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_1 = [B11 B12     B13         B14]

void EPNPEstimator::FindBetasApprox1(const Eigen::Matrix<double, 6, 10>& L6x10,
                                     const Eigen::Matrix<double, 6, 1>& rho,
                                     Eigen::Vector4d* betas) {
  Eigen::Matrix<double, 6, 4> L_6x4;
  for (int i = 0; i < 6; ++i) {
    L_6x4(i, 0) = L6x10(i, 0);
    L_6x4(i, 1) = L6x10(i, 1);
    L_6x4(i, 2) = L6x10(i, 3);
    L_6x4(i, 3) = L6x10(i, 6);
  }

  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 4>> svd(
      L_6x4, Eigen::ComputeFullV | Eigen::ComputeFullU);
  Eigen::Matrix<double, 6, 1> Rho_temp = rho;
  const Eigen::Matrix<double, 4, 1> b4 = svd.solve(Rho_temp);

  if (b4[0] < 0) {
    (*betas)[0] = std::sqrt(-b4[0]);
    (*betas)[1] = -b4[1] / (*betas)[0];
    (*betas)[2] = -b4[2] / (*betas)[0];
    (*betas)[3] = -b4[3] / (*betas)[0];
  } else {
    (*betas)[0] = std::sqrt(b4[0]);
    (*betas)[1] = b4[1] / (*betas)[0];
    (*betas)[2] = b4[2] / (*betas)[0];
    (*betas)[3] = b4[3] / (*betas)[0];
  }
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_2 = [B11 B12 B22                            ]

void EPNPEstimator::FindBetasApprox2(const Eigen::Matrix<double, 6, 10>& L6x10,
                                     const Eigen::Matrix<double, 6, 1>& rho,
                                     Eigen::Vector4d* betas) {
  Eigen::Matrix<double, 6, 3> L_6x3(6, 3);

  for (int i = 0; i < 6; ++i) {
    L_6x3(i, 0) = L6x10(i, 0);
    L_6x3(i, 1) = L6x10(i, 1);
    L_6x3(i, 2) = L6x10(i, 2);
  }

  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 3>> svd(
      L_6x3, Eigen::ComputeFullV | Eigen::ComputeFullU);
  Eigen::Matrix<double, 6, 1> Rho_temp = rho;
  const Eigen::Matrix<double, 3, 1> b3 = svd.solve(Rho_temp);

  if (b3[0] < 0) {
    (*betas)[0] = std::sqrt(-b3[0]);
    (*betas)[1] = (b3[2] < 0) ? std::sqrt(-b3[2]) : 0.0;
  } else {
    (*betas)[0] = std::sqrt(b3[0]);
    (*betas)[1] = (b3[2] > 0) ? std::sqrt(b3[2]) : 0.0;
  }

  if (b3[1] < 0) {
    (*betas)[0] = -(*betas)[0];
  }

  (*betas)[2] = 0.0;
  (*betas)[3] = 0.0;
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_3 = [B11 B12 B22 B13 B23                    ]

void EPNPEstimator::FindBetasApprox3(const Eigen::Matrix<double, 6, 10>& L6x10,
                                     const Eigen::Matrix<double, 6, 1>& rho,
                                     Eigen::Vector4d* betas) {
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 5>> svd(
      L6x10.leftCols<5>(), Eigen::ComputeFullV | Eigen::ComputeFullU);
  Eigen::Matrix<double, 6, 1> Rho_temp = rho;
  const Eigen::Matrix<double, 5, 1> b5 = svd.solve(Rho_temp);

  if (b5[0] < 0) {
    (*betas)[0] = std::sqrt(-b5[0]);
    (*betas)[1] = (b5[2] < 0) ? std::sqrt(-b5[2]) : 0.0;
  } else {
    (*betas)[0] = std::sqrt(b5[0]);
    (*betas)[1] = (b5[2] > 0) ? std::sqrt(b5[2]) : 0.0;
  }
  if (b5[1] < 0) {
    (*betas)[0] = -(*betas)[0];
  }
  (*betas)[2] = b5[3] / (*betas)[0];
  (*betas)[3] = 0.0;
}

void EPNPEstimator::RunGaussNewton(const Eigen::Matrix<double, 6, 10>& L6x10,
                                   const Eigen::Matrix<double, 6, 1>& rho,
                                   Eigen::Vector4d* betas) {
  Eigen::Matrix<double, 6, 4> A;
  Eigen::Matrix<double, 6, 1> b;

  const int kNumIterations = 5;
  for (int k = 0; k < kNumIterations; ++k) {
    for (int i = 0; i < 6; ++i) {
      A(i, 0) = 2 * L6x10(i, 0) * (*betas)[0] + L6x10(i, 1) * (*betas)[1] +
                L6x10(i, 3) * (*betas)[2] + L6x10(i, 6) * (*betas)[3];
      A(i, 1) = L6x10(i, 1) * (*betas)[0] + 2 * L6x10(i, 2) * (*betas)[1] +
                L6x10(i, 4) * (*betas)[2] + L6x10(i, 7) * (*betas)[3];
      A(i, 2) = L6x10(i, 3) * (*betas)[0] + L6x10(i, 4) * (*betas)[1] +
                2 * L6x10(i, 5) * (*betas)[2] + L6x10(i, 8) * (*betas)[3];
      A(i, 3) = L6x10(i, 6) * (*betas)[0] + L6x10(i, 7) * (*betas)[1] +
                L6x10(i, 8) * (*betas)[2] + 2 * L6x10(i, 9) * (*betas)[3];

      b(i) = rho[i] - (L6x10(i, 0) * (*betas)[0] * (*betas)[0] +
                       L6x10(i, 1) * (*betas)[0] * (*betas)[1] +
                       L6x10(i, 2) * (*betas)[1] * (*betas)[1] +
                       L6x10(i, 3) * (*betas)[0] * (*betas)[2] +
                       L6x10(i, 4) * (*betas)[1] * (*betas)[2] +
                       L6x10(i, 5) * (*betas)[2] * (*betas)[2] +
                       L6x10(i, 6) * (*betas)[0] * (*betas)[3] +
                       L6x10(i, 7) * (*betas)[1] * (*betas)[3] +
                       L6x10(i, 8) * (*betas)[2] * (*betas)[3] +
                       L6x10(i, 9) * (*betas)[3] * (*betas)[3]);
    }

    const Eigen::Vector4d x = A.colPivHouseholderQr().solve(b);

    (*betas) += x;
  }
}

double EPNPEstimator::ComputeRT(const Eigen::Matrix<double, 12, 12>& Ut,
                                const Eigen::Vector4d& betas,
                                Eigen::Matrix3d* R, Eigen::Vector3d* t) {
  ComputeCcs(betas, Ut);
  ComputePcs();

  SolveForSign();

  EstimateRT(R, t);

  return ComputeTotalReprojectionError(*R, *t);
}

void EPNPEstimator::ComputeCcs(const Eigen::Vector4d& betas,
                               const Eigen::Matrix<double, 12, 12>& Ut) {
  for (int i = 0; i < 4; ++i) {
    ccs_[i][0] = ccs_[i][1] = ccs_[i][2] = 0.0;
  }

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      for (int k = 0; k < 3; ++k) {
        ccs_[j][k] += betas[i] * Ut(11 - i, 3 * j + k);
      }
    }
  }
}

void EPNPEstimator::ComputePcs() {
  pcs_.resize(points2D_.size());
  for (size_t i = 0; i < points3D_.size(); ++i) {
    for (int j = 0; j < 3; ++j) {
      pcs_[i][j] = alphas_[i][0] * ccs_[0][j] + alphas_[i][1] * ccs_[1][j] +
                   alphas_[i][2] * ccs_[2][j] + alphas_[i][3] * ccs_[3][j];
    }
  }
}

void EPNPEstimator::SolveForSign() {
  if (pcs_[0][2] < 0.0 || pcs_[0][2] > 0.0) {
    for (int i = 0; i < 4; ++i) {
      ccs_[i] = -ccs_[i];
    }
    for (size_t i = 0; i < points3D_.size(); ++i) {
      pcs_[i] = -pcs_[i];
    }
  }
}

void EPNPEstimator::EstimateRT(Eigen::Matrix3d* R, Eigen::Vector3d* t) {
  Eigen::Vector3d pc0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d pw0 = Eigen::Vector3d::Zero();

  for (size_t i = 0; i < points3D_.size(); ++i) {
    pc0 += pcs_[i];
    pw0 += points3D_[i];
  }
  pc0 /= points3D_.size();
  pw0 /= points3D_.size();

  Eigen::Matrix3d abt = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < points3D_.size(); ++i) {
    for (int j = 0; j < 3; ++j) {
      abt(j, 0) += (pcs_[i][j] - pc0[j]) * (points3D_[i][0] - pw0[0]);
      abt(j, 1) += (pcs_[i][j] - pc0[j]) * (points3D_[i][1] - pw0[1]);
      abt(j, 2) += (pcs_[i][j] - pc0[j]) * (points3D_[i][2] - pw0[2]);
    }
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      abt, Eigen::ComputeFullV | Eigen::ComputeFullU);
  const Eigen::Matrix3d abt_U = svd.matrixU();
  const Eigen::Matrix3d abt_V = svd.matrixV();

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      (*R)(i, j) = abt_U.row(i) * abt_V.row(j).transpose();
    }
  }

  if (R->determinant() < 0) {
    Eigen::Matrix3d Abt_v_prime = abt_V;
    Abt_v_prime.col(2) = -abt_V.col(2);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        (*R)(i, j) = abt_U.row(i) * Abt_v_prime.row(j).transpose();
      }
    }
  }

  *t = pc0 - *R * pw0;
}

double EPNPEstimator::ComputeTotalReprojectionError(const Eigen::Matrix3d& R,
                                                    const Eigen::Vector3d& t) {
  Eigen::Matrix3x4d proj_matrix;
  proj_matrix.leftCols<3>() = R;
  proj_matrix.rightCols<1>() = t;

  std::vector<double> residuals;
  ComputeSquaredReprojectionError(points2D_, points3D_, proj_matrix,
                                  &residuals);

  double reproj_error = 0.0;
  for (const double residual : residuals) {
    reproj_error += std::sqrt(residual);
  }

  return reproj_error;
}

}
