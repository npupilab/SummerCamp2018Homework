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


CameraOpenCV::CameraOpenCV(double Fx, double Fy, double Cx, double Cy, double K1, double K2, double P1, double P2, double K3)
    :fx(Fx),fy(Fy),cx(Cx),cy(Cy),k1(K1),k2(K2),p1(P1),p2(P2),k3(K3)
{
}

bool CameraOpenCV::isValid()const
{
    return fx!=0&&fy!=0&&cx!=0&&cy!=0;
}

CameraOpenCV::Point2d CameraOpenCV::Project(const Point3d &p3d) const
{
    double inv_z = 1.0/p3d.z;
    double x = p3d.x * inv_z;
    double y = p3d.y * inv_z;

    double x2 = x*x;
    double y2 = y*y;
    double xy = x*y;
    double r2 = x*x + y*y;
    double r4 = r2*r2;
    double r6 = r4*r2;

    double x_d = x*(1+k1*r2+k2*r4+k3*r6) + 2*p1*xy + p2*(r2+2*x2);
    double y_d = y*(1+k1*r2+k2*r4+k3*r6) + p1*(r2+2*y2) + 2*p2*xy;

    double u = x_d*fx + cx;
    double v = y_d*fy + cy;

    return Point2d(u,v);
}

CameraOpenCV::Point3d CameraOpenCV::UnProject(const Point2d &p2d) const
{
   double x_d = (p2d.x-cx)/fx;
   double y_d = (p2d.y-cy)/fy;

   //简单迭代法x_n=f(x_{n-1})
   //取畸变后的二维点坐标作为初始值，求畸变前的二维点坐标
   //详细推导见https://github.com/JiaoYanMoGu/research_note/blob/master/OpenCV%E7%9B%B8%E6%9C%BA%E7%95%B8%E5%8F%98%E4%B8%8E%E5%8E%BB%E7%95%B8%E5%8F%98.md
   double x_n = x_d;
   double y_n = y_d;
   double x_n_new,y_n_new;
   for(int j=0 ; j< 5; j++)
   {
       double r2 = x_n*x_n + y_n*y_n;
       double disRadical = 1/(1.+k1*r2 + k2*r2*r2 + k3*r2*r2*r2);

       double deltX = 2*p1*x_n*y_n + p2*(r2 + 2*x_n*x_n);
       double deltY = p1*(r2 + 2*y_n*y_n) + 2*p2*x_n*y_n;
       x_n_new = (x_d - deltX)*disRadical;
       y_n_new = (y_d - deltY)*disRadical;

       x_n = x_n_new;
       y_n = y_n_new;
   }
   return CameraOpenCV::Point3d(x_n,y_n,1);
}

bool CameraOpenCV::applyScale(double scale)
{
    //why...
    fx = scale * fx;
    fy = scale * fy;
    cx = scale * cx;
    cy = scale * cy;
    return true;
}

}

#endif
