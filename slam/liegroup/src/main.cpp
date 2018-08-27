#include "sophus/sim3.hpp"
#include <fstream>
#include <LieGroup.h>
#include <GSLAM/core/Random.h>

using namespace summercamp;

bool LookAtTest(){
    Eigen::Vector3d t;
    t<<     GSLAM::Random::RandomValue(0.,1.),
            GSLAM::Random::RandomValue(0.,1.),
            GSLAM::Random::RandomValue(0.,1.);

    auto trans=LieGroup::lookAt(t,Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,1));
    return true;
}

bool transformMultiTest()
{
    return true;
}

bool logRTest(){

    return true;
}

bool expRTest(){

    return true;
}

bool logTTest(){

    return true;
}

bool expTTest(){

    return true;
}
bool jacobianPTest(){
    return true;
}

bool jacobianRTest(){
    return true;
}

bool jacobianTTest(){
    return true;
}

int Return(int argc,char** argv,int i){
    std::string topic="slam/liegroup";
    std::string name ="demo";
    std::string scoreFile="score.txt";
    if(argc>1) topic=argv[1];
    if(argc>2) name =argv[2];
    if(argc>3) scoreFile=argv[3];

    std::vector<std::string> outputInfos={
        "[S]("+topic+"/"+name+"/LieGroup.h)",
        "[C]("+topic+"/evaluation/lookat.md)",
        "[C]("+topic+"/evaluation/transform.md)",
        "[C]("+topic+"/evaluation/logR.md)",
        "[B]("+topic+"/evaluation/expR.md)",
        "[B]("+topic+"/evaluation/logT.md)",
        "[B]("+topic+"/evaluation/expT.md)",
        "[A]("+topic+"/evaluation/jacobianP.md)",
        "[A]("+topic+"/evaluation/jacobianR.md)",
        "[A]("+topic+"/evaluation/jacobianT.md)"
    };

    if(argc>3){
        std::ofstream ofs(scoreFile);
        ofs<<outputInfos[i];
    }
    else std::cout<<outputInfos[i];
}

int main(int argc,char** argv){
    if(!LookAtTest())
        return Return(argc,argv,1);

    if(!transformMultiTest())
        return Return(argc,argv,2);

    if(!logRTest())
        return Return(argc,argv,3);

    if(!expRTest())
        return Return(argc,argv,4);

    if(!logTTest())
        return Return(argc,argv,5);

    if(!expTTest())
        return Return(argc,argv,6);

    if(!jacobianPTest())
        return Return(argc,argv,7);

    if(!jacobianRTest()){
        return Return(argc,argv,8);
    }

    if(!jacobianTTest()){
        return Return(argc,argv,9);
    }

    return Return(argc,argv,0);
}

