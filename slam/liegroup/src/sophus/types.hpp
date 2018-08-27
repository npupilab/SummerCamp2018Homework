#pragma once

#include "../g2o/core/base_vertex.h"
#include "../g2o/core/base_binary_edge.h"
#include "../g2o/core/base_unary_edge.h"
#include "sim3.hpp"
#include "se3.hpp"
#include <GSLAM/core/SE3.h>
#include <GSLAM/core/GPS.h>

using namespace Eigen;
using namespace g2o;

namespace Sophus {

typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 7, 7> Matrix7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 7, 1> Vector7d;

#define NO_READ_WRITE \
    bool read(std::istream& is){return false;}\
    bool write(std::ostream& os) const{return false;}

class VertexSim3 : public BaseVertex<7, Sim3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NO_READ_WRITE

    VertexSim3(): BaseVertex<7, Sim3d>()
    {
        _marginalized=false;
        _fix_scale = false;
    }

    virtual void setToOriginImpl() {
        _estimate = Sim3d();
    }

    virtual void oplusImpl(const double* update_)
    {
        Eigen::Map<Vector7d> update(const_cast<double*>(update_));

        if (_fix_scale)
            update[6] = 0;

        setEstimate(Sim3d::exp(update)*estimate());
    }

    bool _fix_scale;
};

class  VertexSE3 : public BaseVertex<6, SE3d>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NO_READ_WRITE

    virtual void setToOriginImpl() {
        _estimate = SE3d();
    }

    virtual void oplusImpl(const double* update_)  {
        Eigen::Map<const Vector6d> update(update_);
        setEstimate(SE3d::exp(update)*estimate());
    }
};

class VertexXYZ : public BaseVertex<3, Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NO_READ_WRITE

    virtual void setToOriginImpl() {
        _estimate.fill(0.);
    }

    virtual void oplusImpl(const double* update)
    {
        Eigen::Map<const Vector3d> v(update);
        _estimate += v;
    }
};

class VertexIdepth : public BaseVertex<1, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NO_READ_WRITE

    virtual void setToOriginImpl() {
        _estimate=0;
    }

    virtual void oplusImpl(const double* update)
    {
        _estimate += *update;
    }
};

class  EdgeSE3PinHoleXYZ: public  BaseBinaryEdge<2, Vector2d, VertexXYZ, VertexSE3>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NO_READ_WRITE

    EdgeSE3PinHoleXYZ(): fx(1),fy(1),cx(0),cy(0) {
    }

    void computeError()  {
        const VertexSE3* v1 = static_cast<const VertexSE3*>(_vertices[1]);
        const VertexXYZ* v2 = static_cast<const VertexXYZ*>(_vertices[0]);
        Vector2d obs(_measurement);
        _error = cam_project(v1->estimate()*v2->estimate())-obs;
    }

    virtual void linearizeOplus(){
        VertexXYZ* vi = static_cast<VertexXYZ*>(_vertices[0]);
        VertexSE3* vj = static_cast<VertexSE3*>(_vertices[1]);
        SE3d T(vj->estimate());
        Vector3d xyz = vi->estimate();
        Vector3d xyz_trans = T*xyz;

        const double& x = xyz_trans[0];
        const double& y = xyz_trans[1];
        const double& invz = 1./xyz_trans[2];
        const double& invz_2 = invz*invz;

        Matrix<double,2,3> proj;
        proj<<fx*invz,0,-x*invz_2*fx,
                0,fy*invz,-y*invz_2*fy;

        _jacobianOplusXi = proj * T.rotationMatrix();

        _jacobianOplusXj.block<2,3>(0,0)=proj;
        _jacobianOplusXj.block<2,3>(0,3)=-proj* Sophus::SO3d::hat(xyz_trans);
    }

    Vector2d cam_project(const Vector3d & ptCam) const{
        Vector2d proj (ptCam[0]/ptCam[2],ptCam[1]/ptCam[2]);
        return Vector2d(proj[0]*fx + cx, proj[1]*fy + cy);
    }

    bool isDepthPositive()
    {
        const VertexSE3* v1 = static_cast<const VertexSE3*>(_vertices[1]);
        const VertexXYZ* v2 = static_cast<const VertexXYZ*>(_vertices[0]);
        return (v1->estimate()*v2->estimate())[2]>0;
    }

    double fx, fy, cx, cy;
};

class EdgeSE3 : public BaseBinaryEdge<6, SE3d, VertexSE3, VertexSE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NO_READ_WRITE

    void computeError()
    {
        const VertexSE3* _from = static_cast<const VertexSE3*>(_vertices[0]);
        const VertexSE3* _to = static_cast<const VertexSE3*>(_vertices[1]);

        Sophus::SE3d error_= _from->estimate().inverse() * _to->estimate() * _inverseMeasurement;
        _error = error_.log();

    }

    void linearizeOplus()
    {
        const VertexSE3* _from = static_cast<const VertexSE3*>(_vertices[0]);

        _jacobianOplusXj = _from->estimate().inverse().Adj();
        _jacobianOplusXi = -_jacobianOplusXj;
    }


    virtual void setMeasurement(const Sophus::SE3d& m)
    {
        _measurement = m;
        _inverseMeasurement = m.inverse();
    }

    virtual bool setMeasurementData(const double* m)
    {
        Eigen::Map<const Vector6d> v(m);
        setMeasurement(Sophus::SE3d::exp(v));
        return true;
    }

    virtual bool setMeasurementFromState()
    {
        const VertexSE3* from = static_cast<const VertexSE3*>(_vertices[0]);
        const VertexSE3* to   = static_cast<const VertexSE3*>(_vertices[1]);
        Sophus::SE3d delta = from->estimate().inverse() * to->estimate();
        setMeasurement(delta);
        return true;
    }

    virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& , g2o::OptimizableGraph::Vertex* ) { return 1.;}

    virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* /*to*/)
    {
        VertexSE3 *_from = static_cast<VertexSE3*>(_vertices[0]);
        VertexSE3 *_to   = static_cast<VertexSE3*>(_vertices[1]);

        if (from.count(_from) > 0)
            _to->setEstimate(_from->estimate() * _measurement);
        else
            _from->setEstimate(_to->estimate() * _inverseMeasurement);
    }

protected:
    Sophus::SE3d _inverseMeasurement;
};

class EdgeSim3 : public BaseBinaryEdge<7, Sim3d, VertexSim3, VertexSim3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NO_READ_WRITE

    void computeError()// measurement= _from->estimate().inverse() * _to->estimate()
    {
        const VertexSim3* _from = static_cast<const VertexSim3*>(_vertices[0]);
        const VertexSim3* _to = static_cast<const VertexSim3*>(_vertices[1]);

        Sophus::Sim3d error_= _from->estimate().inverse() * _to->estimate() * _inverseMeasurement;
        _error = error_.log();

    }

    void linearizeOplus()
    {
        const VertexSim3* _from = static_cast<const VertexSim3*>(_vertices[0]);

        _jacobianOplusXj = _from->estimate().inverse().Adj();
        _jacobianOplusXi = -_jacobianOplusXj;
    }


    virtual void setMeasurement(const Sophus::Sim3d& m)
    {
        _measurement = m;
        _inverseMeasurement = m.inverse();
    }

    virtual bool setMeasurementData(const double* m)
    {
        Eigen::Map<const Vector7d> v(m);
        setMeasurement(Sophus::Sim3d::exp(v));
        return true;
    }

    virtual bool setMeasurementFromState()
    {
        const VertexSim3* from = static_cast<const VertexSim3*>(_vertices[0]);
        const VertexSim3* to   = static_cast<const VertexSim3*>(_vertices[1]);
        Sophus::Sim3d delta = from->estimate().inverse() * to->estimate();
        setMeasurement(delta);
        return true;
    }

    virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& , g2o::OptimizableGraph::Vertex* ) { return 1.;}

    virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* /*to*/)
    {
        VertexSim3 *_from = static_cast<VertexSim3*>(_vertices[0]);
        VertexSim3 *_to   = static_cast<VertexSim3*>(_vertices[1]);

        if (from.count(_from) > 0)
            _to->setEstimate(_from->estimate() * _measurement);
        else
            _from->setEstimate(_to->estimate() * _inverseMeasurement);
    }

protected:
    Sophus::Sim3d _inverseMeasurement;
};

class EdgeSE3GPS : public  BaseUnaryEdge<6, SE3d, VertexSE3>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NO_READ_WRITE

    void computeError()  {
        const VertexSE3* fr = static_cast<const VertexSE3*>(_vertices[0]);
        _error=(fr->estimate()*_measurement).log();
    }

    //    virtual void linearizeOplus(){
    //      _jacobianOplusXi = _measurement.Adj();
    //    }
};


class EdgeSE3GPSPRY : public  BaseUnaryEdge<6, SE3d, VertexSE3>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NO_READ_WRITE

    void computeError()  {
        const VertexSE3* fr = static_cast<const VertexSE3*>(_vertices[0]);
        _error=getTransPyr(fr->estimate().inverse())-transpyr;
    }

    Vector6d getTransPyr(SE3d ecef){
        SE3d localCamera=ecef2local*ecef;

//        SO3d camera2IMU(Eigen::Quaterniond(0.5,-0.5,0.5,-0.5));
//        SO3d imu2world=localCamera.so3()*camera2IMU.inverse();
//        auto t=localCamera.translation();
//        pi::SO3d r(imu2world);
//        Vector6d result;
//        result<<t[0],t[1],t[2],0,0,0;//r.getPitch(),r.getYaw(),r.getRoll();
        return localCamera.log();
    }

    void setMeasurement(const Measurement &m){
        pi::SE3d ecef=m;
        GSLAM::Point3d lla=GSLAM::GPS<>::XYZ2GPS(ecef.get_translation());
        double D2R=3.1415925/180.;
        double lon=lla.y*D2R;
        double lat=lla.x*D2R;
        GSLAM::Point3d up(cos(lon)*cos(lat), sin(lon)*cos(lat), sin(lat));
        GSLAM::Point3d east(-sin(lon), cos(lon), 0);
        GSLAM::Point3d north=up.cross(east);
        double R[9]={east.x, north.x, up.x,
                     east.y, north.y, up.y,
                     east.z, north.z, up.z};
        pi::SE3d   local2ecef;
        local2ecef.get_rotation().fromMatrix(R);
        local2ecef.get_translation()=GSLAM::GPS<>::GPS2XYZ(lla);

        ecef2local=local2ecef.inverse();
        transpyr=getTransPyr(m);
    }

    SE3d     ecef2local;
    Vector6d transpyr;
};

class EdgeSE3Epipolar : public BaseUnaryEdge<1, Vector2d, VertexSE3>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NO_READ_WRITE

    void computeError()  {
        const VertexSE3* fr = static_cast<const VertexSE3*>(_vertices[0]);
        const Vector3d&  t  = fr->estimate().translation();
        if(t.dot(t)<1e-8) {
            _error=ErrorVector(1e-3);return;
        }
        const Vector2d&  p2d=measurement();
        Vector3d tcrosspl(t[1]-t[2]*p2d[1],
                t[2]*p2d[0]-t[0],
                t[0]*p2d[1]-t[1]*p2d[0]);
        Vector3d line=fr->estimate().so3().inverse()*tcrosspl;
        _error=ErrorVector((line[0]*p[0]+line[1]*p[1]+line[2])/sqrt(line[0]*line[0]+line[1]*line[1]+1e-10));
    }

    virtual void linearizeOplus(){
        const VertexSE3* fr = static_cast<const VertexSE3*>(_vertices[0]);
        const Vector3d&  t  = fr->estimate().translation();
        if(t.dot(t)<1e-8){
            _jacobianOplusXi = Vector6d();
        }
        else
            BaseUnaryEdge<1, Vector2d, VertexSE3>::linearizeOplus();
    }

    Vector3d  p;
};

class EdgeSE3InvDepth : public BaseBinaryEdge<2, double*, VertexSE3,VertexIdepth>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NO_READ_WRITE

    void computeError()  {
        const VertexSE3*    fr = static_cast<const VertexSE3*>(_vertices[0]);
        const VertexIdepth* idepth= static_cast<const VertexIdepth*>(_vertices[1]);
        Measurement match=measurement();
        const Vector3d   p1(match[0],match[1],match[2]);
        auto p=fr->estimate()*(p1/idepth->estimate());

        const double invW=match[5]/p[2];
        _error=Vector2d(invW*p[0]-match[3],invW*p[1]-match[4]);
    }
};

}
