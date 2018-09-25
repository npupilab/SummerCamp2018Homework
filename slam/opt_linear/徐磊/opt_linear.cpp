#include <cstdlib>
#include <ctime>
#include <cmath>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include "opt_linear.h"

#define random(a,b) (rand()%(b-a+1)+a)


bool GSLAM::EstimatorEigen::findHomography(double *H, const std::vector<GSLAM::Point2d> &srcPoints,
                                           const std::vector<GSLAM::Point2d> &dstPoints, int method,
                                           double ransacReprojThreshold, std::vector<uchar> *mask) const
{
    srand((unsigned) time(NULL));

    const int number_matches = srcPoints.size();

    vector<Point2d> npoints1,npoints2;

    Matrix3d T1,T2;
    Normalize(srcPoints,npoints1,T1);
    Normalize(dstPoints,npoints2,T2);

    Matrix3d T2_inv = T2.inverse();

    double score = -1.0;

    vector<Point2d> src_points8(8);
    vector<Point2d> dst_points8(8);

    Matrix3d h21,h12;

    double current_score;

    for(int i = 0; i<200; i++)
    {
        for(int j=0; j < src_points8.size() ; j++)
        {
            int idx = random(0,number_matches-1);
            src_points8[j] = npoints1[idx];
            dst_points8[j] = npoints2[idx];
        }

        Matrix3d currentH = ComputeH21(src_points8,dst_points8);
        h21 = T2_inv*currentH*T1;
        h12 = h21.inverse();
        current_score = CheckHomography(h21,h12,srcPoints,dstPoints);

        if(current_score > score)
        {
            score = current_score;
            H[0] = h21(0,0);
            H[1] = h21(0,1);
            H[2] = h21(0,2);
            H[3] = h21(1,0);
            H[4] = h21(1,1);
            H[5] = h21(1,2);
            H[6] = h21(2,0);
            H[7] = h21(2,1);
            H[8] = h21(2,2);
        }
    }
    return true;
}

bool EstimatorEigen::findFundamental(double *F, const std::vector<Point2d> &points1, const std::vector<Point2d> &points2, int method, double param1, double param2, std::vector<uchar> *mask) const
{
    const int N = points1.size();

    vector<Point2d> npoints1,npoints2;
    Matrix3d T1,T2;

    Normalize(points1,npoints1,T1);
    Normalize(points2,npoints2,T2);

    Matrix3d T2_t = T2.transpose();
    Matrix3d preF;

    preF = ComputeF21(npoints1,npoints2);
    preF  = T2_t*preF*T1;

    preF = preF/preF(2,2);

    F[0] = preF(0,0);
    F[1] = preF(0,1);
    F[2] = preF(0,2);
    F[3] = preF(1,0);
    F[4] = preF(1,1);
    F[5] = preF(1,2);
    F[6] = preF(2,0);
    F[7] = preF(2,1);
    F[8] = preF(2,2);

    return true;
}

bool EstimatorEigen::findEssentialMatrix(double *E, const std::vector<Point2d> &points1, const std::vector<Point2d> &points2, int method, double param1, double param2, std::vector<uchar> *mask) const
{
    //TODO
}

void EstimatorEigen::Normalize(const std::vector<Point2d> &input_points, std::vector<Point2d> &output_points, Matrix3d &translation) const
{
    double mean_x = 0.0;
    double mean_y = 0.0;
    const int N = input_points.size();

    output_points.resize(N);

    for(int i= 0; i<N ;i++)
    {
        mean_x +=input_points[i].x;
        mean_y +=input_points[i].y;
    }
    mean_x = mean_x /N;
    mean_y = mean_y /N;

    double  mean_devx = 0.0;
    double  mean_devy = 0.0;

    for(int i=0; i< N;i++)
    {
        output_points[i].x = input_points[i].x - mean_x;
        output_points[i].y = input_points[i].y - mean_y;
        mean_devx += fabs(output_points[i].x);
        mean_devy += fabs(output_points[i].y);
    }
    mean_devx  = mean_devx / N;
    mean_devy  = mean_devy / N;

    double sx = 1.0 / mean_devx;
    double sy = 1.0 / mean_devy;

    for(int i=0; i< N ;i++)
    {
        output_points[i].x  = sx * output_points[i].x;
        output_points[i].y  = sy * output_points[i].y;
    }
    translation = Eigen::Matrix3d::Identity();
    translation(0,0) = sx;
    translation(1,1) = sy;
    translation(0,2) = -mean_x * sx;
    translation(1,2) = -mean_y * sy;
}

Matrix3d EstimatorEigen::ComputeH21(std::vector<Point2d> &src_points, std::vector<Point2d> &dst_points) const
{
    const int N = src_points.size();
    MatrixXd cof = MatrixXd::Zero(N*2,9);

    for(int i=0 ; i< N ;i++)
    {
        const double u1 = src_points[i].x;
        const double v1 = src_points[i].y;
        const double u2 = dst_points[i].x;
        const double v2 = dst_points[i].y;

        cof(2 * i,0) = 0.0;
        cof(2 * i,1) = 0.0;
        cof(2 * i,2) = 0.0;
        cof(2 * i,3) = -u1;
        cof(2 * i,4) = -v1;
        cof(2 * i,5) = -1;
        cof(2 * i,6) = v2 * u1;
        cof(2 * i,7) = v2 * v1;
        cof(2 * i,8) = v2;

        cof(2 * i +1 ,0) = u1;
        cof(2 * i +1 ,1) = v1;
        cof(2 * i +1 ,2) = 1;
        cof(2 * i +1 ,3) = 0.0;
        cof(2 * i +1 ,4) = 0.0;
        cof(2 * i +1 ,5) = 0.0;
        cof(2 * i +1 ,6) = -u2 * u1;
        cof(2 * i +1 ,7) = -u2 * v1;
        cof(2 * i +1 ,8) = -u2;
    }
    JacobiSVD<Eigen::MatrixXd> svd(cof,ComputeFullU|ComputeFullV);
    MatrixXd vt = svd.matrixV();
    Matrix3d result;

    result<<
            vt(0,8), vt(1,8), vt(2,8),
            vt(3,8), vt(4,8), vt(5,8),
            vt(6,8), vt(7,8), vt(8,8);
    result = result / result(2,2);
    return result;
}

double EstimatorEigen::CheckHomography(const Matrix3d &homography21, const Matrix3d &homography12, const std::vector<Point2d> &src_points, const std::vector<Point2d> &dst_points) const
{
    const int N = src_points.size();

    const double h11 = homography21(0,0);
    const double h12 = homography21(0,1);
    const double h13 = homography21(0,2);
    const double h21 = homography21(1,0);
    const double h22 = homography21(1,1);
    const double h23 = homography21(1,2);
    const double h31 = homography21(2,0);
    const double h32 = homography21(2,1);
    const double h33 = homography21(2,2);

    const double h11_inv = homography12(0,0);
    const double h12_inv = homography12(0,1);
    const double h13_inv = homography12(0,2);
    const double h21_inv = homography12(1,0);
    const double h22_inv = homography12(1,1);
    const double h23_inv = homography12(1,2);
    const double h31_inv = homography12(2,0);
    const double h32_inv = homography12(2,1);
    const double h33_inv = homography12(2,2);

    double score  = 0.0;
    const double th = 5.991;

    for(int i=0 ; i < N; i++)
    {
        bool in = true;

        const double u1 = src_points[i].x;
        const double v1 = src_points[i].y;
        const double u2 = dst_points[i].x;
        const double v2 = dst_points[i].y;

        const double w2_in1_inv = 1.0 / (h31_inv * u2 + h32_inv * v2 + h33_inv);
        const double u2_in1 =(h11_inv * u2 + h12_inv * v2 + h13_inv) * w2_in1_inv;
        const double v2_in1 =(h21_inv * u2 + h22_inv * v2 + h23_inv) * w2_in1_inv;

        const double square_dist1 = (u1 - u2_in1)*(u2- u2_in1) + (v1 - v2_in1)*(v1-v2_in1);

        if(square_dist1 > th)
        {
            in = false;
        }
        else score +=th - square_dist1;
    }
    return score;
}

Matrix3d EstimatorEigen::ComputeF21(std::vector<Point2d> &src_points, std::vector<Point2d> &dst_points) const
{
    const int N = src_points.size();
    MatrixXd cof = MatrixXd::Zero(N,9);

    for(int i=0; i < N; i++)
    {
        const double u1 = src_points[i].x;
        const double v1 = src_points[i].y;
        const double u2 = dst_points[i].x;
        const double v2 = dst_points[i].y;


        cof(i,0) = u1 * u2;
        cof(i,1) = v1 * u2;
        cof(i,2) = u2;
        cof(i,3) = u1 * v2;
        cof(i,4) = v1 * v2;
        cof(i,5) = v2;
        cof(i,6) = u1;
        cof(i,7) = v1;
        cof(i,8) = 1.0;
    }

    JacobiSVD<MatrixXd> svd(cof,ComputeFullU | ComputeFullV);
    Eigen::MatrixXd vt = svd.matrixV();

    Matrix3d result;
    result <<
              vt(0,8), vt(1,8), vt(2,8),
              vt(3,8), vt(4,8), vt(5,8),
              vt(6,8), vt(7,8), vt(8,8);

    svd.compute(result,ComputeFullU | ComputeFullV);

    VectorXd w = svd.singularValues();
    Matrix3d w_pre = Matrix3d::Zero();
    w_pre(0,0) = w(0);
    w_pre(1,1) = w(1);

    result = svd.matrixU() * w_pre * svd.matrixV().transpose();

    return result;
}

double EstimatorEigen::CheckFundamental(const Matrix3d &fundamental21, const std::vector<Point2d> &src_points, const std::vector<Point2d> &dst_points) const
{
    const int N = src_points.size();

    const double f11 = fundamental21(0,0);
    const double f12 = fundamental21(0,1);
    const double f13 = fundamental21(0,2);
    const double f21 = fundamental21(1,0);
    const double f22 = fundamental21(1,1);
    const double f23 = fundamental21(1,2);
    const double f31 = fundamental21(2,0);
    const double f32 = fundamental21(2,1);
    const double f33 = fundamental21(2,2);

    const double th = 3.841;
    const double th_score = 5.991;

    double score = 0.0;

    for(int i=0; i< N ;i++)
    {
        const double u1 = src_points[i].x;
        const double v1 = src_points[i].y;
        const double u2 = src_points[i].x;
        const double v2 = src_points[i].y;

        const double a2 = f11 * u1 + f12 * v1 + f13;
        const double b2 = f21 * u1 + f22 * v1 + f23;
        const double c2 = f31 * u1 + f31 * v1 + f33;

        const double num2 = a2 * u2 + b2 * v2 + c2;

        const double square_dist1 = num2 * num2 / (a2 * a2 + b2 * b2);

        if(square_dist1 <= th)
        {
            score += th_score - square_dist1;
        }
    }
    return score;
}

Matrix3d EstimatorEigen::ComputeE21(const std::vector<Point2d> &src_points, const std::vector<Point2d> &dst_points) const
{
    const int N = src_points.size();
    Eigen::Matrix<double,Eigen::Dynamic,9> cof(N,9);

    for( int i=0 ; i< N ;i++)
    {
        const double u1 = src_points[i].x;
        const double v1 = src_points[i].y;
        const double u2 = dst_points[i].x;
        const double v2 = dst_points[i].y;

        cof(i,0) = u1 * u2;
        cof(i,1) = v1 * u2;
        cof(i,2) = u2;
        cof(i,3) = u1 * v2;
        cof(i,4) = v1 * v2;
        cof(i,5) = v2;
        cof(i,6) = u1;
        cof(i,7) = v1;
        cof(i,8) = 1.0;
    }

    const Eigen::JacobiSVD<Eigen::Matrix<double,Eigen::Dynamic,9>> svd(cof,ComputeFullU | ComputeFullV);
    const Eigen::Matrix<double,9,4> E = svd.matrixV().block<9,4>(0,5);//FIXME
}







