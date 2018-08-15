#include <GSLAM/core/Estimator.h>
#include <GSLAM/core/Random.h>
#include <GSLAM/core/Array.h>

#define EXPECT_LE(a,b) if(a>b) return -1;
#define EXPECT_TRUE(a) if(!(a)) return -1;
#define EXPECT_FALSE(a) if(a) return -1;

int testHomographyRANSAC(){
    auto estimator=GSLAM::Estimator::create();
    if(!estimator.get())
    {
        LOG(WARNING)<<"Test aborded since estimator not created.";
        return -2;
    }

    double H[9]={1,2,3,
                 4,5,6,
                 0,0,1};
    std::vector<GSLAM::Point2d> src,dst;
    int n=svar.GetInt("HomographyTest.Num",100);
    src.reserve(n);
    dst.reserve(n);
    std::vector<double> noises;
    for(int i=0;i<n;i++){
        GSLAM::Point2d pt(GSLAM::Random::RandomValue(0.0,100.),
                          GSLAM::Random::RandomValue(0.0,100.));
        GSLAM::Point2d pt1(H[0]*pt.x+H[1]*pt.y+H[2],
                           H[3]*pt.x+H[4]*pt.y+H[5]);
        GSLAM::Point2d noise(GSLAM::Random::RandomGaussianValue(0.,0.1),
                             GSLAM::Random::RandomGaussianValue(0.,0.1));
        src.push_back(pt);
        dst.push_back(pt1+noise);
        noises.push_back(noise.norm());
    }

    double H_est[9];
    std::vector<uchar> mask;
    if(!estimator->findHomography(H_est,src,dst,GSLAM::RANSAC,1,&mask)) return 0;

    std::cout<<"H:";
    for(int i=0;i<9;i++) std::cout<<","<<H_est[i];
    std::cout<<"\n";

    // compute average error
    std::vector<double> errors;
    for(int i=0;i<n;i++){
        GSLAM::Point2d pt(src[i]);
        GSLAM::Point2d pt1(H_est[0]*pt.x+H_est[1]*pt.y+H_est[2],
                           H_est[3]*pt.x+H_est[4]*pt.y+H_est[5]);
        pt=pt1-dst[i];
        errors.push_back(pt.norm());
    }
    std::sort(errors.begin(),errors.end());
    std::sort(noises.begin(),noises.end());
    std::cout<<"MidNoise:"<<noises[noises.size()/2]<<",MidError:"<<errors[errors.size()/2]<<"\n";
    if(errors[errors.size()/2]<3*noises[noises.size()/2]) return 1;

    return -1;
}


int testFundamentalSevenPoint()
{
    auto estimator=GSLAM::Estimator::create();
    if(!estimator.get())
    {
        LOG(WARNING)<<"Test aborded since estimator not created.";
        return -2;
    }

    const double points1_raw[] = {0.4964, 1.0577,  0.3650,  -0.0919, -0.5412,
                                  0.0159, -0.5239, 0.9467,  0.3467,  0.5301,
                                  0.2797, 0.0012,  -0.1986, 0.0460};

    const double points2_raw[] = {0.7570, 2.7340,  0.3961,  0.6981, -0.6014,
                                  0.7110, -0.7385, 2.2712,  0.4177, 1.2132,
                                  0.3052, 0.4835,  -0.2171, 0.5057};

    const size_t kNumPoints = 7;

    std::vector<GSLAM::Point2d> points1(kNumPoints);
    std::vector<GSLAM::Point2d> points2(kNumPoints);
    for (size_t i = 0; i < kNumPoints; ++i) {
      points1[i] = GSLAM::Point2d(points1_raw[2 * i], points1_raw[2 * i + 1]);
      points2[i] = GSLAM::Point2d(points2_raw[2 * i], points2_raw[2 * i + 1]);
    }

    double F[9];
    if(!estimator->findFundamental(F,points1,points2,GSLAM::FM_7POINT)) return 0;

    // Reference values obtained from Matlab.
    EXPECT_LE(fabs(F[0]-4.81441976), 1e-6);
    EXPECT_LE(fabs(F[1]+8.16978909), 1e-6);
    EXPECT_LE(fabs(F[2]-6.73133404), 1e-6);
    EXPECT_LE(fabs(5.16247992-F[3]), 1e-6);
    EXPECT_LE(fabs(0.19325606-F[4]), 1e-6);
    EXPECT_LE(fabs(-2.87239381-F[5]), 1e-6);
    EXPECT_LE(fabs(-9.92570126-F[6]), 1e-6);
    EXPECT_LE(fabs(3.64159554-F[7]), 1e-6);
    EXPECT_LE(fabs(1.-F[8]), 1e-6);
    return 1;
}


int testFundamentalEightPoint()
{
    auto estimator=GSLAM::Estimator::create();
    if(!estimator.get())
    {
        LOG(WARNING)<<"Test aborded since estimator not created.";
        return -2;
    }

    const double points1_raw[] = {1.839035, 1.924743, 0.543582,  0.375221,
                                  0.473240, 0.142522, 0.964910,  0.598376,
                                  0.102388, 0.140092, 15.994343, 9.622164,
                                  0.285901, 0.430055, 0.091150,  0.254594};

    const double points2_raw[] = {
        1.002114, 1.129644, 1.521742, 1.846002, 1.084332, 0.275134,
        0.293328, 0.588992, 0.839509, 0.087290, 1.779735, 1.116857,
        0.878616, 0.602447, 0.642616, 1.028681,
    };

    const size_t kNumPoints = 8;

    std::vector<GSLAM::Point2d> points1(kNumPoints);
    std::vector<GSLAM::Point2d> points2(kNumPoints);
    for (size_t i = 0; i < kNumPoints; ++i) {
      points1[i] = GSLAM::Point2d(points1_raw[2 * i], points1_raw[2 * i + 1]);
      points2[i] = GSLAM::Point2d(points2_raw[2 * i], points2_raw[2 * i + 1]);
    }

    double F[9];
    EXPECT_TRUE(estimator->findFundamental(F,points1,points2,GSLAM::FM_8POINT));

    // Reference values obtained from Matlab.
    for(int i=0;i<9;i++) F[i]*=0.0221019;
    EXPECT_LE(fabs(-0.217859-F[0]), 2e-2);
    EXPECT_LE(fabs(0.419282-F[1]), 2e-2);// TODO : why opencv impementation not pass <1-5
    EXPECT_LE(fabs(-0.0343075-F[2]), 1e-2);
    EXPECT_LE(fabs(-0.0717941-F[3]), 1e-2);
    EXPECT_LE(fabs(0.0451643-F[4]), 1e-2);
    EXPECT_LE(fabs(0.0216073-F[5]), 1e-2);
    EXPECT_LE(fabs(0.248062-F[6]), 2e-2);
    EXPECT_LE(fabs(-0.429478-F[7]), 2e-2);
    EXPECT_LE(fabs(0.0221019-F[8]), 1e-2);
    return testFundamentalSevenPoint();
}

int testEssentialFivePoint() {
    auto estimator=GSLAM::Estimator::create();
    if(!estimator.get())
    {
        LOG(WARNING)<<"Test aborded since estimator not created.";
        return -2;
    }

    const double points1_raw[] = {
        0.4964, 1.0577, 0.3650,  -0.0919, -0.5412, 0.0159, -0.5239, 0.9467,
        0.3467, 0.5301, 0.2797,  0.0012,  -0.1986, 0.0460, -0.1622, 0.5347,
        0.0796, 0.2379, -0.3946, 0.7969,  0.2,     0.7,    0.6,     0.3};

    const double points2_raw[] = {
        0.7570, 2.7340, 0.3961,  0.6981, -0.6014, 0.7110, -0.7385, 2.2712,
        0.4177, 1.2132, 0.3052,  0.4835, -0.2171, 0.5057, -0.2059, 1.1583,
        0.0946, 0.7013, -0.6236, 3.0253, 0.5,     0.9,    0.9,     0.2};

    const size_t kNumPoints = 12;

    std::vector<GSLAM::Point2d> points1(kNumPoints);
    std::vector<GSLAM::Point2d> points2(kNumPoints);
    for (size_t i = 0; i < kNumPoints; ++i) {
        points1[i] = GSLAM::Point2d(points1_raw[2 * i], points1_raw[2 * i + 1]);
        points2[i] = GSLAM::Point2d(points2_raw[2 * i], points2_raw[2 * i + 1]);
    }

    double F[9];
    std::vector<uchar> inlier_mask;
    if(!estimator->findEssentialMatrix(F,points1,points2,GSLAM::RANSAC,0.02,0.9999,&inlier_mask)) return 0;


    EXPECT_FALSE(inlier_mask[10]);
    EXPECT_FALSE(inlier_mask[11]);
    return 1;
}


int testSE3PlaneRansac() {
    auto estimator=GSLAM::Estimator::create();
    if(!estimator.get())
    {
        LOG(WARNING)<<"Test aborded since estimator not created.";
        return -2;
    }
    const double points_raw[] = {
        0,0,1,
        0,1,1,
        1,0,1,
        1,1,1,
        7,7,1,
        0,9,1,
        9,0,1,
        9,9,1,
        0,0,0,
        0,1,0};

    const size_t kNumPoints = 10;

    std::vector<GSLAM::Point3d> points(kNumPoints);
    for (size_t i = 0; i < kNumPoints; ++i) {
        points[i] = GSLAM::Point3d(points_raw[3 * i], points_raw[3 * i + 1], points_raw[3*i+2]);
    }

    GSLAM::SE3 plane;
    std::vector<uchar> inlier_mask;
    EXPECT_TRUE(estimator->findPlane(plane,points,GSLAM::RANSAC,0.02,&inlier_mask));

    EXPECT_FALSE(inlier_mask[8]);
    EXPECT_FALSE(inlier_mask[9]);

    for(int i=0;i<8;i++)
    {
        EXPECT_TRUE(fabs((plane.inverse()*points[i]).z)<0.01);
    }

    return 1;

}

int testTriangulate(){
    auto estimator=GSLAM::Estimator::create();
    if(!estimator.get())
    {
        LOG(WARNING)<<"Test aborded since estimator not created.";
        return -2;
    }

    GSLAM::SE3 ref2cur(GSLAM::SO3::exp(GSLAM::Point3d(0.1,0.1,0.1)),
                       GSLAM::Point3d(1,1,0));
    GSLAM::Point3d refDirection(0,0,1);
    GSLAM::Point3d ptGround=refDirection*svar.GetDouble("Estimator.Triangulate.Depth",5.);
    GSLAM::Point3d curDirection=(ref2cur*ptGround);
    curDirection=curDirection/curDirection.z;

    GSLAM::Point3d refPtEst;
    estimator->trianglate(ref2cur,refDirection,curDirection,refPtEst);
    EXPECT_LE((refPtEst-ptGround).norm(),1e-6);
    return 1;
}

int Return(int argc,char** argv,int ret){
    std::string topic="slam/opt_linear";
    std::string name ="demo";
    std::string scoreFile="score.txt";
    if(argc>1) topic=argv[1];
    if(argc>2) name =argv[2];
    if(argc>3) scoreFile=argv[3];

    std::vector<std::string> outputInfos={
        "[D]("+topic+"/evaluation/none.md)",
        "[C]("+topic+"/evaluation/error.md)",
        "[B]("+topic+"/evaluation/less.md)",
        "[S]("+topic+"/"+name+")"
    };

    std::string score;
    if(ret<=2) score=outputInfos[ret];
    else score="[S,"+std::to_string(ret-1)+"]("+topic+"/"+name+")";

    if(argc>3){
        std::ofstream ofs(scoreFile);
        ofs<<score;
    }
    else std::cout<<score;
}

int main(int argc,char** argv){
    std::vector< int(*)()> funcs={
            testHomographyRANSAC,
            testFundamentalEightPoint,
            testEssentialFivePoint,
            testSE3PlaneRansac,
            testTriangulate};

    int successCount=0;
    for(auto func:funcs){
        int ret=func();
        if(ret<-1) return Return(argc,argv,0);
        else if(ret==-1) return Return(argc,argv,1);

        successCount+=ret;
    }

    if(successCount<2) return Return(argc,argv,2);
    return Return(argc,argv,successCount+1);
}
