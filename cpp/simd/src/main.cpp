#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
#include "SO3SIMD.h"
#include <fstream>
#include "Random.h"
#include "Timer.h"

int Return(int argc,char** argv,int i){
    std::string topic="cpp/simd";
    std::string name ="demo";
    std::string scoreFile="score.txt";
    if(argc>1) topic=argv[1];
    if(argc>2) name =argv[2];
    if(argc>3) scoreFile=argv[3];

    std::vector<std::string> outputInfos={
        "[S]("+topic+"/"+name+"/SO3SIMD.cpp)",
        "[C]("+topic+"/evaluation/wrong_f.md)",
        "[C]("+topic+"/evaluation/wrong_d.md)",
        "[C]("+topic+"/evaluation/wrong_log.md)",
        "[C]("+topic+"/evaluation/wrong_exp.md)",
        "[B]("+topic+"/evaluation/slow_f.md)",
        "[B]("+topic+"/evaluation/slow_d.md)",
        "[B]("+topic+"/evaluation/slow_exp.md)",
        "[B]("+topic+"/evaluation/slow_ln.md)"
    };

    if(argc>3){
        std::ofstream ofs(scoreFile);
        ofs<<outputInfos[i];
    }
    else std::cout<<outputInfos[i];
    return 0;
}

int main(int argc,char** argv)
{
    int num=1e6;
    std::vector<pi::SO3d> ld,rd;
    std::vector<pi::SO3f> lf,rf;
    std::vector<Sophus::SO3f> lfs,rfs;
    std::vector<Sophus::SO3d> lds,rds;

    for(int i=0;i<num;i++){
        GSLAM::Point3d so3(GSLAM::Random::RandomValue(-3.15,3.15),
                           GSLAM::Random::RandomValue(-3.15,3.15),
                           GSLAM::Random::RandomValue(-3.15,3.15));
        GSLAM::Point3d so31(GSLAM::Random::RandomValue(-3.15,3.15),
                           GSLAM::Random::RandomValue(-3.15,3.15),
                           GSLAM::Random::RandomValue(-3.15,3.15));
        ld.push_back(pi::SO3d::exp(so3));
        rd.push_back(pi::SO3d::exp(so31));
        lf.push_back(pi::SO3f::exp(so3));
        rf.push_back(pi::SO3f::exp(so31));
        lfs.push_back(pi::SO3f::exp(so3));
        rfs.push_back(pi::SO3f::exp(so31));
        lds.push_back(pi::SO3d::exp(so3));
        rds.push_back(pi::SO3d::exp(so31));
    }

    {
        pi::SO3d so3;
        for(int i=0;i<num;i++)
        {
            so3=so3*ld[i];
        }
        std::cerr<<"SO3.Double:"<<so3<<std::endl;
        std::cerr<<"SquareNorm.Double:"<<so3.x*so3.x+so3.y*so3.y+so3.z*so3.z+so3.w*so3.w<<std::endl;
    }

    {
        pi::SO3f so3;
        for(int i=0;i<num;i++)
        {
            so3=so3*lf[i];
        }
        std::cerr<<"SO3.Float:"<<so3<<std::endl;
        std::cerr<<"SquareNorm.Float:"<<so3.x*so3.x+so3.y*so3.y+so3.z*so3.z+so3.w*so3.w<<std::endl;
    }

    // check mul float
    for(int i=0;i<num;i++)
    {
        pi::SO3f r=mul(lf[i],rf[i]);
        r=r.inv()*lf[i]*rf[i];
        if(r.ln().norm()>1e-6) return Return(argc,argv,1);
    }

    // check mul double
    for(int i=0;i<num;i++)
    {
        pi::SO3d r=mul(ld[i],rd[i]);
        r=r.inv()*ld[i]*rd[i];
        if(r.ln().norm()>1e-6)
            return Return(argc,argv,2);

        auto s=lds[i]*rds[i];
        if((((pi::SO3d)s).inv()*mul(ld[i],rd[i])).ln().norm()>1e-10)
            return Return(argc,argv,2);
    }

    // speed of mul float normal
    {
        GSLAM::ScopedTimer tm("mul_float_gslam");
        for(int i=0;i<num;i++){
            lf[i]=lf[i]*rf[i];
        }
    }

    {
        GSLAM::ScopedTimer tm("mul_float_simd");
        for(int i=0;i<num;i++){
            lf[i]=mul(lf[i],rf[i]);
        }
    }

    {
        GSLAM::ScopedTimer tm("mul_double_gslam");
        for(int i=0;i<num;i++){
            ld[i]=ld[i]*rd[i];
        }
    }

    {
        GSLAM::ScopedTimer tm("mul_double_simd");
        for(int i=0;i<num;i++){
            ld[i]=mul(ld[i],rd[i]);
        }
    }

    {
        GSLAM::ScopedTimer tm("mul_float_sophus");
        for(int i=0;i<num;i++){
            lfs[i]=lfs[i]*rfs[i];
        }
    }

    {
        GSLAM::ScopedTimer tm("mul_double_sophus");
        for(int i=0;i<num;i++){
            lds[i]=lds[i]*rds[i];
        }
    }

    // check of log
    for(int i=0;i<num;i++){
        GSLAM::Point3f  pt =rf[i].ln();
        Eigen::Vector3f pt1=rfs[i].log();
        GSLAM::Point3d  e=pt-pt1;
        if(e.norm()>1e-4)
        {
            std::cerr<<"Float: ln:"<<pt<<", sophus:"<<pt1;
            return Return(argc,argv,3);
        }
    }


    for(int i=0;i<num;i++){
        GSLAM::Point3d  pt =rd[i].ln();
        Eigen::Vector3d pt1=rds[i].log();
        GSLAM::Point3d  e=pt-pt1;
        if(e.norm()>1e-5)
        {
            std::cerr<<"ln:"<<pt<<", sophus:"<<pt1;
            return Return(argc,argv,3);
        }
    }

    // check of exp
    for(int i=0;i<num;i++){
        Eigen::Vector3f pt1=rfs[i].log();

        Sophus::SO3f  sophus=Sophus::SO3f::exp(pt1);
        pi::SO3f      gslam=exp((GSLAM::Point3f)pt1);

        pi::SO3f e=gslam.inv()*sophus;

        if(e.ln().norm()>1e-2)
        {
            std::cerr<<"Float: exp:"<<gslam<<", sophus:"<<(pi::SO3f)sophus;
            return Return(argc,argv,4);
        }
    }


    for(int i=0;i<num;i++){
        Eigen::Vector3d pt1=rds[i].log();

        Sophus::SO3d  sophus=Sophus::SO3d::exp(pt1);
        pi::SO3d      gslam=exp((GSLAM::Point3d)pt1);

        pi::SO3d e=gslam.inv()*sophus;

        if(e.ln().norm()>1e-2)
        {
            std::cerr<<"Double: exp:"<<gslam<<", sophus:"<<sophus.data();
            return Return(argc,argv,4);
        }
    }

    // speed of log
    {
        std::vector<Eigen::Vector3f> r;
        r.reserve(rfs.size());
        {
            GSLAM::ScopedTimer tm("log_float_sophus");
            for(int i=0;i<num;i++)
            {
                r.push_back(rfs[i].log());
            }
        }

        {
            GSLAM::ScopedTimer tm("exp_float_sophus");
            for(int i=0;i<num;i++)
            {
                rfs[i]=Sophus::SO3f::exp(r[i]);
            }
        }
    }


    {
        std::vector<Eigen::Vector3d> r;
        r.reserve(rds.size());
        {
            GSLAM::ScopedTimer tm("log_double_sophus");
            for(int i=0;i<num;i++)
            {
                r.push_back(rds[i].log());
            }
        }

        {
            GSLAM::ScopedTimer tm("exp_double_sophus");
            for(int i=0;i<num;i++)
            {
                rds[i]=Sophus::SO3d::exp(r[i]);
            }
        }
    }


    {
        std::vector<GSLAM::Point3f> r;
        r.reserve(rf.size());
        {
            GSLAM::ScopedTimer tm("log_float_gslam");
            for(int i=0;i<num;i++)
            {
                r.push_back(rf[i].ln());
            }
        }

        {
            GSLAM::ScopedTimer tm("exp_float_gslam");
            for(int i=0;i<num;i++)
            {
                rf[i]=pi::SO3f::exp(r[i]);
            }
        }
    }

    {
        std::vector<GSLAM::Point3d> r;
        r.reserve(rd.size());
        {
            GSLAM::ScopedTimer tm("log_double_gslam");
            for(int i=0;i<num;i++)
            {
                r.push_back(rd[i].ln());
            }
        }

        {
            GSLAM::ScopedTimer tm("exp_double_gslam");
            for(int i=0;i<num;i++)
            {
                rd[i]=pi::SO3d::exp(r[i]);
            }
        }
    }

    {
        std::vector<GSLAM::Point3f> r;
        r.reserve(rf.size());
        {
            GSLAM::ScopedTimer tm("log_float_simd");
            for(int i=0;i<num;i++)
            {
                r.push_back(log(rf[i]));
            }
        }

        {
            GSLAM::ScopedTimer tm("exp_float_simd");
            for(int i=0;i<num;i++)
            {
                rf[i]=exp(r[i]);
            }
        }
    }

    {
        std::vector<GSLAM::Point3d> r;
        r.reserve(rd.size());
        {
            GSLAM::ScopedTimer tm("log_double_simd");
            for(int i=0;i<num;i++)
            {
                r.push_back(log(rd[i]));
            }
        }

        {
            GSLAM::ScopedTimer tm("exp_double_simd");
            for(int i=0;i<num;i++)
            {
                rd[i]=exp(r[i]);
            }
        }
    }



    return Return(argc,argv,0);
}
