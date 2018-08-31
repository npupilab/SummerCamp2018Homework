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
        "[C]("+topic+"/evaluation/wrong_exp.md)",
        "[C]("+topic+"/evaluation/wrong_ln.md)",
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

    for(int i=0;i<num;i++){
        GSLAM::Point3d so3(GSLAM::Random::RandomValue(-3.15,3.15),
                           GSLAM::Random::RandomValue(-3.15,3.15),
                           GSLAM::Random::RandomValue(-3.15,3.15));
        GSLAM::Point3d so31(GSLAM::Random::RandomValue(-3.15,3.15),
                           GSLAM::Random::RandomValue(-3.15,3.15),
                           GSLAM::Random::RandomValue(-3.15,3.15));
        ld.push_back(pi::SO3d::exp(so3));
        lf.push_back(pi::SO3f::exp(so3));
        rd.push_back(pi::SO3d::exp(so31));
        rf.push_back(pi::SO3f::exp(so31));
    }

    // check mul float
    for(int i=0;i<num;i++)
    {
        pi::SO3f r=mul(lf[i],rf[i]);
        r=r.inv()*lf[i]*rf[i];
        if(r.ln().norm()>1e-5) return Return(argc,argv,2);
    }

    // check mul double
    for(int i=0;i<num;i++)
    {
        pi::SO3d r=mul(ld[i],rd[i]);
        r=r.inv()*ld[i]*rd[i];
        if(r.ln().norm()>1e-6) return Return(argc,argv,2);
    }

    // speed of mul float normal
    {
        GSLAM::ScopedTimer tm("mul_float_O3");
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
    timer.getMeanTime("mul_float_O3");
    return Return(argc,argv,0);
}
