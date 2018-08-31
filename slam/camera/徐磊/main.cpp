#include <fstream>
#include <GSLAM/core/Camera.h>
#include <GSLAM/core/Random.h>
#include <Camera.h>

using namespace std;
using namespace GSLAM;

bool operator ==(GSLAM::Point2d left,GSLAM::Point2d right){
    return (left-right).norm()<1e-8;
}

bool operator !=(GSLAM::Point2d left,GSLAM::Point2d right){
    return !(left==right);
}


bool operator ==(GSLAM::Point3d left,GSLAM::Point3d right){
    return (left-right).norm()<1e-8;
}

bool operator !=(GSLAM::Point3d left,GSLAM::Point3d right){
    return !(left==right);
}

GSLAM::Point2d to2d(GSLAM::Point3d pt){return Point2d(pt.x/pt.z,pt.y/pt.z);}


int Return(int argc,char** argv,int i){
    std::string topic="slam/camera";
    std::string name ="demo";
    std::string scoreFile="score.txt";
    if(argc>1) topic=argv[1];
    if(argc>2) name =argv[2];
    if(argc>3) scoreFile=argv[3];

    std::vector<std::string> outputInfos={
        "[S]("+topic+"/"+name+"/Camera.h)",
        "[C]("+topic+"/evaluation/project.md)",
        "[B]("+topic+"/evaluation/unproject.md)",
        "[A]("+topic+"/evaluation/scale.md)"
    };

    if(argc>3){
        std::ofstream ofs(scoreFile);
        ofs<<outputInfos[i];
    }
    else std::cout<<outputInfos[i];
    return 0;
}

int main(int argc,char** argv){
    std::vector<double> k={
        1920,1080,1184.51770,1183.63810,978.30778,533.85598,
        -0.01581,0.01052,-0.00075,0.00245,0.00000};

    summercamp::CameraOpenCV camera(
                k[2],k[3],k[4],k[5],
            k[6],k[7],k[8],k[9],k[10]);

    GSLAM::Camera ground(k);

    double scale=0.25;
    summercamp::CameraOpenCV scaledCamera=camera;
    if(!scaledCamera.applyScale(scale)) return Return(argc,argv,3);

    for(int i=0;i<100;i++){
        Point3d r1(Random::RandomValue(0.,1.),
                   Random::RandomValue(0.,1.),
                   2);
        Point2d pt(Random::RandomValue(0.,k[0]),
                   Random::RandomValue(0.,k[1]));

        if(camera.Project(r1)!=ground.Project(r1))
            return Return(argc,argv,1);

        if(camera.UnProject(pt)!=ground.UnProject(pt))
            return Return(argc,argv,2);

        if(camera.Project(r1)*scale!=scaledCamera.Project(r1))
            return Return(argc,argv,3);
    }

    return Return(argc,argv,0);
}
