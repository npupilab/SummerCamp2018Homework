#include <GSLAM/core/Optimizer.h>
#include <GSLAM/core/Random.h>
#include <GSLAM/core/Timer.h>

using namespace GSLAM;

int TestOptimizePnPSimulate()
{
    // Prepare simulation data
    int N=svar.GetInt("SimulateNumber",2000);//number of matches
    std::vector<std::pair<pi::Point3d, pi::Point3d> > groundMatches,estMatches;
    GSLAM::SE3                                        groundPose,estPose;
    pi::Array_<double,6> array;
    for(int i=0;i<array.size();i++)
        array.data[i]=GSLAM::Random::RandomValue<double>();
    groundPose=GSLAM::SE3::exp(array);
    estPose=groundPose*GSLAM::SE3(pi::SO3d::exp(Point3d(0.1,0.1,0.1)),pi::Point3d(1,2,3));
    for(int i=0;i<N;i++)
    {
        pi::Point3d pCamera(GSLAM::Random::RandomValue(-1.,1.),GSLAM::Random::RandomValue(-1.,1.),1);
        pi::Point3d pWorld(groundPose*(pCamera*GSLAM::Random::RandomGaussianValue(10.,1.)));

        groundMatches.push_back(std::make_pair(pWorld,pCamera));
        // add noise
        if(true)
        {
            pi::Point3d projectNoise(GSLAM::Random::RandomGaussianValue(0.,i<(0.5*N)?0.1:0.01),
                                     GSLAM::Random::RandomGaussianValue(0.,i<(0.5*N)?0.1:0.01),0);
            pCamera=pCamera+projectNoise;
        }
        estMatches.push_back(std::make_pair(pWorld,pCamera));
    }


    SPtr<Optimizer> opt=Optimizer::create();
    if(!opt)
    {
        LOG(ERROR)<<"No valid optimizer plugin!";
        return 1;
    }
    opt->_config.verbose=svar.GetInt("Optimizer.Verbose",0);
    GSLAM::TicToc ticToc;
    if(!opt->optimizePnP(estMatches,estPose)) return 2;

    LOG_IF(INFO,opt->_config.verbose)<<"GroundPose:"<<groundPose
                                    <<",EstPose:"<<estPose
                                   <<",Time:"<<ticToc.Tac()*1000<<"ms"<<std::endl;

    GSLAM::SE3 error=groundPose.inverse()*estPose;
    if(error.get_translation().norm()>0.2) return 3;
    if(error.get_rotation().ln().norm()>0.1) return 3;
    return 0;
}

int Return(int argc,char** argv,int ret){
    std::string topic="slam/opt_nolinear";
    std::string name ="demo";
    std::string scoreFile="score.txt";
    if(argc>1) topic=argv[1];
    if(argc>2) name =argv[2];
    if(argc>3) scoreFile=argv[3];

    std::vector<std::string> outputInfos={
        "[S]("+topic+"/"+name+")",
        "[D]("+topic+"/evaluation/none.md)",
        "[C]("+topic+"/evaluation/return.md)",
        "[B]("+topic+"/evaluation/wrong.md)"
    };

    std::string score=outputInfos[ret];
    if(argc>3){
        std::ofstream ofs(scoreFile);
        ofs<<score;
    }
    else std::cout<<score;
    return 0;
}


int main(int argc,char** argv){
    GSLAM::OptimizerPtr optimizer=GSLAM::Optimizer::create();
    if(!optimizer) return Return(argc,argv,1);

    return Return(argc,argv,TestOptimizePnPSimulate());
}
