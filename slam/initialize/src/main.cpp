#include "Initializer.h"

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
        "[C]("+topic+"/evaluation/error.md)",
        "[B]("+topic+"/evaluation/less.md)"
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

   return Return(argc,argv,0);
}
