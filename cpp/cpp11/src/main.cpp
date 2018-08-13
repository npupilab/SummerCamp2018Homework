#include "sum_multithread.h"
#include <fstream>
#include "Random.h"
#include "Timer.h"

int sum_singlethread(const std::vector<int>& nums)
{
    int sum=0;
    for(auto i:nums) sum+=i;
    return sum;
}

int main(int argc,char** argv)
{
    std::vector<int> vec(1e8+1,0);
    for(int& i:vec)
        i=GSLAM::Random::RandomInt(0,10);

    GSLAM::TicToc tictoc;
    tictoc.Tic();
    int sum1=sum_multithread(vec);
    double time1=tictoc.Toc();

    tictoc.Tic();
    int sum2=sum_singlethread(vec);
    double time2=tictoc.Toc();

    if(argc<4){
        std::cerr<<"sigle_thread:"<<sum2<<", time "<<time2<<"s"
                <<", multi_thread:"<<sum1<<", time "<<time1<<"s\n";
    return 0;
    }
    
    std::ofstream ofs(argv[3]);
    if(!ofs.is_open()) return 0;
    if(sum1!=sum2){ofs<<"[C](cpp/cpp11/evaluation/wrongsum.md";return 0;}
    if(time2<sum1){ofs<<"[B](cpp/cpp11/evaluation/slow.md";return 0;}

    ofs<<"["<<int(time1/1000)<<"](cpp/cpp11/"<<argv[2]<<"/sum_multithread.cpp)";
    return 0;
}
