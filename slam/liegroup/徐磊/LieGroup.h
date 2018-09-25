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
    Matrix4d t_cw = Matrix4d::Zero(4,4);
    Matrix3d rotation;
    Vector3d x = (-up).cross(lookAtPoint);
    x.normalize();
    rotation.col(0) = x;
    Vector3d y = (-up).normalized();
    y.normalize();
    rotation.col(1) = y;
    lookAtPoint.normalize();
    rotation.col(2) = lookAtPoint;
    t_cw.block(0,0,3,3) = rotation;
    t_cw.block(0,3,3,1) = translation;
    t_cw(3,3) = 1;
    return t_cw;
}

LieGroup::Matrix4d LieGroup::transfomMulti(LieGroup::Matrix4d Tcw, LieGroup::Matrix4d Tic)
{
    //默认的inverse() 速度会不会特别慢
    return Tcw.inverse()*Tic.inverse();
}

LieGroup::Vector3d LieGroup::log(LieGroup::Matrix3d R)
{
    Eigen::AngleAxisd a;
    a.fromRotationMatrix(R);
    LieGroup::Vector3d v = a.axis();
    v.normalize();
    return v*a.angle();
    //    //assume no singularities
    //    double angle,x,y,z;
    //    double s = sqrt((m(2,1) - m(1,2))*(m(2,1) - m(1,2))
    //            +(m(0,2) - m(2,0))*(m(0,2) - m(2,0))
    //            +(m(1,0) - m(0,1))*(m(1,0) - m(0,1))); // used to normalise
    //    if (abs(s) < 0.001) s=1;
    //    angle = acos(( m(0,0) + m(1,1) + m(2,2) - 1)/2);
    //    x = (m(2,1) - m(1,2))/s;
    //    y = (m(0,2) - m(2,0))/s;
    //    z = (m(1,0) - m(0,1))/s;
    //    return Vector3d(x*angle,y*angle,z*angle);
}


LieGroup::Matrix3d LieGroup::exp(Vector3d so3)
{
    double angle = so3.norm();
    so3.normalize();
    Vector3d a = so3;
    Matrix3d a_hat = Matrix3d::Zero();
    a_hat(0,1) = -so3[2];
    a_hat(0,2) = so3[1];
    a_hat(1,0) = so3[2];
    a_hat(1,2) = -so3[0];
    a_hat(2,0) = -so3[1];
    a_hat(2,1) = so3[0];

    Matrix3d result = cos(angle)*Eigen::Matrix3d::Identity()+(1-cos(angle))*a*a.transpose() + sin(angle)*a_hat;
    return result;
}

LieGroup::Vector6d LieGroup::log(Matrix4d T)
{
    Matrix3d R = T.block(0,0,3,3);
    Vector3d r = LieGroup::log(R);
    Vector3d trans  = T.block(0,3,3,1);
    double angle = r.norm();
    r.normalize();
    Vector3d so3 = r;
    Matrix3d a_hat = Matrix3d::Zero();
    a_hat(0,1) = -so3[2];
    a_hat(0,2) = so3[1];
    a_hat(1,0) = so3[2];
    a_hat(1,2) = -so3[0];
    a_hat(2,0) = -so3[1];
    a_hat(2,1) = so3[0];
    
    Matrix3d J = sin(angle)/angle*Matrix3d::Identity() + (1 - sin(angle)/angle)*r*r.transpose() + (1-cos(angle))/angle*a_hat;
    Vector3d rou = J.lu().solve(trans);
    Vector6d result;
    result.block(0,0,3,1) = rou;
    result.block(3,0,3,1) = r*angle;
    return result;
}

LieGroup::Matrix4d  LieGroup::exp(Vector6d se3)
{
    Vector3d r = se3.block(3,0,3,1);
    Vector3d rou = se3.block(0,0,3,1);
    Matrix3d R = LieGroup::exp(r);
    double angle = r.norm();
    r.normalize();
    Matrix3d a_hat = Matrix3d::Zero();
    a_hat(0,1) = -r[2];
    a_hat(0,2) = r[1];
    a_hat(1,0) = r[2];
    a_hat(1,2) = -r[0];
    a_hat(2,0) = -r[1];
    a_hat(2,1) = r[0];
    Matrix3d  J = sin(angle)/angle*Matrix3d::Identity() + (1- sin(angle)/angle)*r*r.transpose() + (1-cos(angle))/angle*a_hat;
    Vector3d translation = J*rou;
    Matrix4d result = Matrix4d::Zero();
    result.block(0,0,3,3) = R;
    result.block(0,3,3,1) = translation;
    result(3,3) = 1;
    return result;
}

LieGroup::Matrix3d LieGroup::jacobianP(LieGroup::Vector3d xi,LieGroup::Vector3d p)
{
    return(LieGroup::exp(xi));
}

LieGroup::Matrix3d LieGroup::jacobianR(Matrix3d R,Vector3d p)
{
   //左扰动模型
   Vector3d v = -R*p;
   Matrix3d result = Matrix3d::Zero();
   result(0,0) = -v[2];
   result(0,2) = v[1];
   result(1,0) = v[2];
   result(1,2) = -v[0];
   result(2,0) = -v[1];
   result(2,1) = v[0];
   return result;

}

LieGroup::Matrix36d LieGroup::jacobianT(Matrix4d T,Vector3d p)
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
