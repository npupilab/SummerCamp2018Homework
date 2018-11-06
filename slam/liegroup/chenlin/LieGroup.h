#ifndef SUMMERCAMP_LIEGROUP_H
#define SUMMERCAMP_LIEGROUP_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
namespace summercamp {

class LieGroup{
public:
    typedef Eigen::Vector3d Vector3d;
    typedef Eigen::Matrix<double,6,1> Vector6d;
    typedef Eigen::Matrix3d Matrix3d;
    typedef Eigen::Matrix<double,3,4> Matrix34d;
    typedef Eigen::Matrix<double,4,4> Matrix4d;
    typedef Eigen::Matrix<double,3,6> Matrix36d;

    // Please return the camera to world transformation T_wc
    // P_w=T_wc*P_c
    static Matrix4d lookAt(Vector3d translation, // this means the position of the camera
                           Vector3d lookAtPoint, // this means the z axis direction
                           Vector3d up);         // this means the -y axis direction

    // Coordinates: IMU, Camera, World
    // T_cw : world to left camera
    // T_ic : left camera to imu
    // Please return the IMU to world transformation T_wi
    static Matrix4d transfomMulti(Matrix4d Tcw,Matrix4d Tic);

    // Given a rotation matrix R, please return the lie algebra log(R)
    static Vector3d log(Matrix3d R);

    // Given the rotation lie algebra so3, please return the rotation matrix R
    static Matrix3d exp(Vector3d so3);

    // Given a transform matrix T, please return the lie algebra log(T)
    static Vector6d log(Eigen::Matrix4d T);

    // Given the transform lie algebra se3, please return the transform matrix T
    static Matrix4d exp(Vector6d se3);

    // Given rotation lie algebra \xi , 3d point p,
    // please return \frac{\partial{exp(\xi)*p}}{\partial{p}}
    static Matrix3d jacobianP(Vector3d xi,Vector3d p);

    // Given rotation R, 3d point p, sub to: R=exp(\xi)
    // please return \frac{\partial{R*p}}{\partial{\xi}}
    static Matrix3d jacobianR(Matrix3d R,Vector3d p);

    // Given SE3 T, 3d point p, sub to : T=exp(\xi)
    // please return \frac{\partial{T*p}}{\partial{\xi}}
    static Matrix36d jacobianT(Matrix4d T,Vector3d p);
};

LieGroup::Matrix4d LieGroup::lookAt(Vector3d translation, Vector3d lookAtPoint, Vector3d up)
{
    Matrix4d T_wc = Matrix4d::Zero(4,4);
    Matrix3d rotation;

    Vector3d x = (-up).cross(lookAtPoint);
    rotation.col(0) = x.normalized();
    rotation.col(1) = (-up).normalized();
    rotation.col(2) = lookAtPoint.normalized();
    T_wc.block(0,0,3,3) = rotation;
    T_wc.block(0,3,3,1) = translation;
    T_wc(3,3) = 1;
    return T_wc;
}

LieGroup::Matrix4d LieGroup::transfomMulti(Matrix4d Tcw, Matrix4d Tic)
{
    return (Tic*Tcw).inverse();
}

LieGroup::Vector3d LieGroup::log(Matrix3d R)
{
    Eigen::AngleAxisd angleAxis;
    angleAxis.fromRotationMatrix(R);
    return angleAxis.axis();
}

LieGroup::Matrix3d LieGroup::exp(Vector3d so3)
{
    Eigen::AngleAxisd angleAxis(so3.norm(),so3.normalized());
    return angleAxis.toRotationMatrix();
}

LieGroup::Vector6d LieGroup::log(Eigen::Matrix4d T)
{
    LieGroup::Matrix3d rotationM(T.block(0,0,3,3));
    Eigen::Vector3d t(T.block(0,3,3,1));

    Eigen::Vector3d angleAxis = LieGroup::log(rotationM);
    double theta = angleAxis.norm();
    Eigen::Vector3d a = angleAxis.normalized();
    LieGroup::Matrix3d a_hat = LieGroup::Matrix3d::Zero();
    a_hat(0,1) = -a[2];
    a_hat(0,2) = a[1];
    a_hat(1,0) = a[2];
    a_hat(1,2) = -a[0];
    a_hat(2,0) = -a[1];
    a_hat(2,1) = a[0];

    LieGroup::Matrix3d J = sin(theta)/theta*Matrix3d::Identity() + (1 - sin(theta)/theta)*a*a.transpose() + (1-cos(theta))/theta*a_hat;

    LieGroup::Vector6d result;
    result.block(0,0,3,1) = J.inverse()*t;
    result.block(3,0,3,1) = angleAxis;
    return result;
}

LieGroup::Matrix4d LieGroup::exp(Vector6d se3)
{
    LieGroup::Vector3d rou = se3.block(0,0,3,1);
    Eigen::AngleAxisd fi(se3.block(3,0,3,1).norm(),se3.block(3,0,3,1).normalized());
    Matrix3d a_hat = Matrix3d::Zero();
    a_hat(0,1) = -fi.axis()[2];
    a_hat(0,2) = fi.axis()[1];
    a_hat(1,0) = fi.axis()[2];
    a_hat(1,2) = -fi.axis()[0];
    a_hat(2,0) = -fi.axis()[1];
    a_hat(2,1) = fi.axis()[0];
    LieGroup::Matrix3d J = sin(fi.angle())/fi.angle()*Matrix3d::Identity() + (1 - sin(fi.angle())/fi.angle())*fi.axis()*fi.axis().transpose() + (1-cos(fi.angle()))/fi.angle()*a_hat;
    LieGroup::Matrix4d result = LieGroup::Matrix4d::Zero();
    result.block(0,0,3,3) = fi.matrix();
    result.block(0,3,3,1) = J*rou;
    result(3,3) = 1;
    return result;
}

LieGroup::Matrix3d LieGroup::jacobianP(Vector3d xi, Vector3d p)
{
    return LieGroup::exp(xi);
}

LieGroup::Matrix3d LieGroup::jacobianR(Matrix3d R, Vector3d p)
{
    Eigen::Vector3d v = R*p;
    LieGroup::Matrix3d result = LieGroup::Matrix3d::Zero();
    result(0,1) = v[2];
    result(0,2) = -v[1];
    result(1,0) = -v[2];
    result(1,2) = v[0];
    result(2,0) = v[1];
    result(2,1) = -v[0];
    return result;
}

LieGroup::Matrix36d LieGroup::jacobianT(Matrix4d T, Vector3d p)
{
    Matrix3d R = T.block(0,0,3,3);
    Vector3d t = T.block(0,3,3,1);
    Matrix36d result = Matrix36d::Zero();
    result.block(0,0,3,3) = Matrix3d::Identity();
    Vector3d v = -(R*p+t);
    Matrix3d b = Matrix3d::Zero();
    b(0,0) = -v[2];
    b(0,2) = v[1];
    b(1,0) = v[2];
    b(1,2) = -v[0];
    b(2,0) = -v[1];
    b(2,1) = v[0];
    result.block(0,3,3,3) = b;
    return result;
}

}
#endif
