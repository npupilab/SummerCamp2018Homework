#include "Initializer.h"
#include <GSLAM/core/Random.h>
#include <GSLAM/core/Timer.h>

using namespace  GSLAM;
using namespace std;

int simulateInitializer()
{
    Point3d so3(Random::RandomValue(-0.1,0.1),
                Random::RandomValue(-0.1,0.1),
                Random::RandomValue(-0.1,0.1));
    Point3d t(Random::RandomValue(-1.,1.),
              Random::RandomValue(-1.,1.),
              Random::RandomValue(-0.01,0.01));
    SE3 t21ground(SO3::exp(so3),t.normalize());

    GSLAM::Camera camera({640,480,500,500,320,240});

    std::vector<std::pair<GSLAM::Point2f,GSLAM::Point2f> > matches,noisedMatches;
    std::vector<Point3d>                  groundPoints;

    int N=200;
    int w=camera.width(),h=camera.height();
    for(int i=0;i<N;i++){
        GSLAM::Point2f p_img1(Random::RandomValue<float>(0,w),
                              Random::RandomValue<float>(0,h));
        double depth1=Random::RandomGaussianValue(10.,2.);
        Point3d        pt=camera.UnProject(p_img1)*depth1;
        GSLAM::Point2f p_img2=camera.Project(t21ground.inverse()*pt);
        groundPoints.push_back(pt);

        matches.push_back(make_pair(p_img1,p_img2));
        GSLAM::Point2f noise(Random::RandomGaussianValue(0.,2.),
                             Random::RandomGaussianValue(0.,2.));
        if(i%5==0){// outlier noise
            noise=Point2f(Random::RandomValue(-200.,200.),
                   Random::RandomValue(-200.,200.));
        }
        noisedMatches.push_back(make_pair(p_img1,p_img2+noise));
    }

    std::vector<std::pair<int,GSLAM::Point3d> > mpts;
    SE3 t21;

    InitializerPtr initializer=Initializer::create();

    {
        if(!initializer->initialize(matches,camera,t21,mpts))
        {
            DLOG(ERROR)<<"Failed to initialize.";
            return 2;
        }
        SE3 error=t21.inverse()*t21ground;
        if(error.get_translation().norm()>1e-3||error.get_rotation().ln().norm()>1e-3)
        {
            DLOG(ERROR)<<"Pose error too large.";
            return 2;
        }
        if(mpts.size()<N*0.6)
        {
            DLOG(ERROR)<<"Mappoints number "<<mpts.size()<<" too few.";
            return 2;
        }

        int goodCount=0;
        for(auto m:mpts){
            if((groundPoints[m.first]-m.second).norm()<1)
                goodCount++;
        }
        if(goodCount<N*0.5)
        {
            DLOG(ERROR)<<"Good mappoints number "<<goodCount<<" too few.";
            return 2;
        }
    }


    {
        if(!initializer->initialize(noisedMatches,camera,t21,mpts))
        {
            DLOG(ERROR)<<"Failed to initialize.";
            return 3;
        }
        SE3 error=t21.inverse()*t21ground;
        if(error.get_translation().norm()>1||error.get_rotation().ln().norm()>0.1)
        {
            DLOG(ERROR)<<"Pose error too large.";
            return 3;
        }
        if(mpts.size()<N*0.5) {

            DLOG(ERROR)<<"Mappoints number "<<mpts.size()<<" too few.";
            return 3;

        }
    }
    return 0;
}

int Return(int argc,char** argv,int ret){
    std::string topic="slam/initialize";
    std::string name ="demo";
    std::string scoreFile="score.txt";
    if(argc>1) topic=argv[1];
    if(argc>2) name =argv[2];
    if(argc>3) scoreFile=argv[3];

    std::vector<std::string> outputInfos={
        "[S]("+topic+"/"+name+")",
        "[D]("+topic+"/evaluation/none.md)",
        "[C]("+topic+"/evaluation/ground.md)",
        "[B]("+topic+"/evaluation/noise.md)",
        "[A]("+topic+"/evaluation/slow.md)"
    };

    std::string score;
    score=outputInfos[ret];
    if(argc>3){
        std::ofstream ofs(scoreFile);
        ofs<<score;
    }
    else std::cout<<score;
    return 0;
}

int main(int argc,char** argv)
{
   InitializerPtr initializer=Initializer::create();
   if(!initializer) return Return(argc,argv,1);

   GSLAM::TicToc tic;
   for(int i=0;i<10;i++)
   {
       int ret=simulateInitializer();
       if(ret!=0) return Return(argc,argv,ret);
   }
   if(tic.Toc()>10)
       return 4;

   return Return(argc,argv,0);
}
