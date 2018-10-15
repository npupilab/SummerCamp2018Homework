#include <GSLAM/core/Optimizer.h>
#include <GSLAM/core/Timer.h>
#include <eigen3/Eigen/Cholesky>

#define RESIDUAL_SIZE 2
#define PARA_SIZE 6

typedef Eigen::Matrix<double,RESIDUAL_SIZE,PARA_SIZE> PnPJacobian;
typedef Eigen::Matrix<double,PARA_SIZE,PARA_SIZE>     PnPHessian;
typedef Eigen::Matrix<double,PARA_SIZE,1>             JacobianError;
typedef Eigen::Matrix<double,2,3>                     Matrix23;

typedef Eigen::Matrix<double,6,1>                     Vector6d;

using   Eigen::Vector2d;
using   GSLAM::SE3;
using   GSLAM::Point3d;
using   GSLAM::Point2d;


class OptimizerPnP: public GSLAM::Optimizer
{
public:

    virtual bool optimizePnP(const std::vector<std::pair<GSLAM::Point3d, GSLAM::CameraAnchor> > &matches,
                             GSLAM::SE3 &pose, GSLAM::KeyFrameEstimzationDOF dof, double *information)
    {
        GSLAM::ScopedTimer tm("OptimizerPnP::optimizePnP");
        std::vector<Point3d> objectPoints;
        std::vector<Point2d> imagePoints;
        objectPoints.reserve(matches.size());
        imagePoints.reserve(matches.size());

        for(const auto& m:matches){
            objectPoints.push_back(m.first);
            imagePoints.push_back(GSLAM::Point2d(m.second.x,m.second.y));
        }
        GSLAM::SE3 world2camera=pose.inverse();
        bool ret= solvePnP(world2camera,objectPoints,imagePoints,GSLAM::Camera({1,1}));
        pose=world2camera.inverse();
        return ret;
    }

    inline double HuberWeight(double dErrorSquared, double dSigmaSquared)
    {
        GSLAM::ScopedTimer tm("OptimizerPnP::HuberWeight");
      if(dErrorSquared < dSigmaSquared)
        return 1;
      else
        return sqrt(dSigmaSquared / dErrorSquared);
    }

    double cacuSumError(  SE3& world2camera,
                    const std::vector<Point3d>& objectPoints,
                    const std::vector<Point2d>& imagePoints,
                    const GSLAM::Camera&        camera,
                    const double&               dSigmaSquared)
    {
        GSLAM::ScopedTimer tm("OptimizerPnP::cacuSumError");
        double result=0;
        for(size_t i=0;i<objectPoints.size();i++)
        {
            Point3d pcam = world2camera*objectPoints[i];
            Point2d err  = camera.Project(pcam)-imagePoints[i];
            double  info = HuberWeight(err.x*err.x+err.y*err.y,dSigmaSquared);
            err = err*info;
            result+=err.x*err.x+err.y*err.y;
        }
        return result;
    }

    bool solvePnP(  SE3& world2camera,
                    const std::vector<Point3d>& objectPoints,
                    const std::vector<Point2d>& imagePoints,
                    const GSLAM::Camera&        camera)
    {
        GSLAM::ScopedTimer tm("OptimizerPnP::solvePnP");
        assert(camera.isValid());
        assert(objectPoints.size()==imagePoints.size());

        int    maxItNum=50;
        double lambda=0.01;
        double dSigmaSquared=0.01;
        double lambdaFactor=10;

        GSLAM::SE3 w2c=world2camera;
        double curError=cacuSumError(w2c,objectPoints,imagePoints,camera,dSigmaSquared);

        for(int it=0;it<maxItNum;it++)
        {
            PnPHessian Hessian=PnPHessian::Identity()*lambda;
            JacobianError jacerr=JacobianError::Zero();
            for(size_t i=0;i<objectPoints.size();i++)
            {
                GSLAM::ScopedTimer tm("OptimizerPnP::hessian");
                // estimate error and weight
                Point3d pcam = w2c*objectPoints[i];
                Point2d err  = imagePoints[i]-camera.Project(pcam);// WARNNING: The error should be y-f(x) and not f(x)-y
                double  info = HuberWeight(err.x*err.x+err.y*err.y,dSigmaSquared);
                err = err*info;

                // estimate jacobians, see slam/lie_group about how to compute the Jacobian on manifold
                Eigen::Vector3d pc(pcam.x,pcam.y,pcam.z);
                PnPJacobian     jac;
                double invZ =1./pc(2);
                double invZ2=invZ*invZ;
                const double& x=pc(0),&y=pc(1);
                jac<<invZ , 0    , -x*invZ2 , -x*y*invZ2      , (1+(x*x*invZ2))  , -y*invZ ,
                     0    , invZ , -y*invZ2 , -(1+y*y*invZ2)  , x*y*invZ2        , x*invZ ;

                // estimate Hessian and JacobianError
                Hessian += jac.transpose()*info*jac;
                jacerr  += jac.transpose()*Vector2d(err.x,err.y);
            }

            // caculate update, solve Hessian*inc=jacerr
            timer.enter("LDLT");
            Vector6d inc=Hessian.ldlt().solve(jacerr);// use Cholesky ldlt, also can use llt or QR decompose
            timer.leave("LDLT");

            SE3 w2c_new=SE3::exp(*(pi::Array_<double,6>*)&inc)*w2c;
            double errorNew=cacuSumError(w2c_new,objectPoints,imagePoints,camera,dSigmaSquared);

            double facError=errorNew/curError;
            if(facError>=1)
            {
                lambda*=lambdaFactor;
            }
            else
            {
                w2c=w2c_new;
                curError=errorNew;
                lambda/=lambdaFactor;

                printf("iter:%d, curError:%lf, facError:%lf, ",it,curError,facError);
                std::cout<<"inc:"<<inc.transpose()<<std::endl;

                if(facError>0.99) break;// abord when small update
                if(curError<1e-7)   break;// abord when small residual
            }

        }

        world2camera=w2c;
        return true;
    }
};


USE_OPTIMIZER_PLUGIN(OptimizerPnP);

