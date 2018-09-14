#include <cstdlib>
#include <ctime>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include "opt_linear.h"

#define random(a,b) (rand()%(b-a+1)+a)

namespace GSLAM
{
    bool EstimatorEigen::findHomography(double *H, const std::vector<Point2d> &srcPoints,
                                        const std::vector<Point2d> &dstPoints,
                                        int method, double ransacReprojThreshold, vector<uchar> *mask) const
    {
        srand((unsigned) time(NULL));
        const int number_matches = srcPoints.size();
        vector<Point2d> normalize_src_points, normalize_dst_points;
        Matrix3d translation1, translation2;

        Normalize(srcPoints, normalize_src_points, translation1);
        Normalize(dstPoints, normalize_dst_points, translation2);

        Matrix3d translation2_inv = translation2.inverse();
        double score = -1.0;

        vector<Point2d> src_points_8(8);
        vector<Point2d> dst_points_8(8);
        Matrix3d homography21, homography12;
        double current_score;

        for (int i = 0; i < 200; ++i)
        {
            for (int j = 0; j < src_points_8.size(); ++j)
            {
                int idx = random(0, number_matches - 1);

                src_points_8[j] = normalize_src_points[idx];
                dst_points_8[j] = normalize_dst_points[idx];
            }

            Matrix3d current_homography = ComputeH21(src_points_8, dst_points_8);
            homography21 = translation2_inv * current_homography * translation1;
            homography12 = homography21.inverse();
            current_score = CheckHomography(homography21, homography12, srcPoints, dstPoints);

            if (current_score > score)
            {
                score = current_score;
                H[0] = homography21(0, 0);
                H[1] = homography21(0, 1);
                H[2] = homography21(0, 2);
                H[3] = homography21(1, 0);
                H[4] = homography21(1, 1);
                H[5] = homography21(1, 2);
                H[6] = homography21(2, 0);
                H[7] = homography21(2, 1);
                H[8] = homography21(2, 2);
            }
        }

        return 1;
    }

    bool EstimatorEigen::findEssentialMatrix(double *E, const std::vector<Point2d> &points1,
                                             const std::vector<Point2d> &points2, int method, double param1,
                                             double param2, vector<uchar> *mask) const
    {
        const vector<Point2d> src_points(points1.begin(), points1.begin() + 5);
        const vector<Point2d> dst_points(points2.begin(), points2.begin() + 5);

        Matrix3d essential = ComputeE21(src_points, dst_points);
    }

    bool EstimatorEigen::findFundamental(double *F, const std::vector<Point2d> &points1,
                                         const std::vector<Point2d> &points2, int method, double param1, double param2,
                                         vector<uchar> *mask) const
    {
        const int number_matches = points1.size();
        vector<Point2d> normalize_src_points, normalize_dst_points;
        Matrix3d translation1, translation2;

        Normalize(points1, normalize_src_points, translation1);
        Normalize(points2, normalize_dst_points, translation2);

        Matrix3d translation2_trans = translation2.transpose();
        Matrix3d fundamental;

        fundamental = ComputeF21(normalize_src_points, normalize_dst_points);
        fundamental = translation2_trans * fundamental * translation1;

        fundamental = fundamental / fundamental(2, 2);
        F[0] = fundamental(0, 0);
        F[1] = fundamental(0, 1);
        F[2] = fundamental(0, 2);
        F[3] = fundamental(1, 0);
        F[4] = fundamental(1, 1);
        F[5] = fundamental(1, 2);
        F[6] = fundamental(2, 0);
        F[7] = fundamental(2, 1);
        F[8] = fundamental(2, 2);

        return 1;
    }

    void EstimatorEigen::Normalize(const std::vector<Point2d> &input_points, std::vector<Point2d> &output_points, Matrix3d &translation) const
    {
        double mean_x = 0;
        double mean_y = 0;
        const int number_point = input_points.size();

        output_points.resize(number_point);

        for (int i = 0; i < number_point; ++i)
        {
            mean_x += input_points[i].x;
            mean_y += input_points[i].y;
        }

        mean_x = mean_x / number_point;
        mean_y = mean_y / number_point;

        double mean_devx = 0;
        double mean_devy = 0;

        for (int i = 0; i < number_point; ++i)
        {
            output_points[i].x = input_points[i].x - mean_x;
            output_points[i].y = input_points[i].y - mean_y;
            mean_devx += fabs(output_points[i].x);
            mean_devy += fabs(output_points[i].y);
        }

        mean_devx = mean_devx / number_point;
        mean_devy = mean_devy / number_point;

        double sx = 1.0 / mean_devx;
        double sy = 1.0 / mean_devy;

        for (int i = 0; i < number_point; ++i)
        {
            output_points[i].x = sx * output_points[i].x;
            output_points[i].y = sy * output_points[i].y;
        }

        translation = Eigen::Matrix3d::Identity(3, 3);
        translation(0, 0) = sx;
        translation(1, 1) = sy;
        translation(0, 2) = -mean_x * sx;
        translation(1, 2) = -mean_y * sy;
    }

    Matrix3d EstimatorEigen::ComputeH21(std::vector<Point2d> &src_points, std::vector<Point2d> &dst_points) const
    {
        const int number_points = src_points.size();
        MatrixXd coefficient = MatrixXd::Zero(number_points * 2, 9);

        for (int i = 0; i < number_points; ++i)
        {
            const double u1 = src_points[i].x;
            const double v1 = src_points[i].y;
            const double u2 = dst_points[i].x;
            const double v2 = dst_points[i].y;

            coefficient(2 * i, 0) = 0.0;
            coefficient(2 * i, 1) = 0.0;
            coefficient(2 * i, 2) = 0.0;
            coefficient(2 * i, 3) = -u1;
            coefficient(2 * i, 4) = -v1;
            coefficient(2 * i, 5) = -1;
            coefficient(2 * i, 6) = v2 * u1;
            coefficient(2 * i, 7) = v2 * v1;
            coefficient(2 * i, 8) = v2;

            coefficient(2 * i + 1, 0) = u1;
            coefficient(2 * i + 1, 1) = v1;
            coefficient(2 * i + 1, 2) = 1;
            coefficient(2 * i + 1, 3) = 0.0;
            coefficient(2 * i + 1, 4) = 0.0;
            coefficient(2 * i + 1, 5) = 0.0;
            coefficient(2 * i + 1, 6) = -u2 * u1;
            coefficient(2 * i + 1, 7) = -u2 * v1;
            coefficient(2 * i + 1, 8) = -u2;
        }

        JacobiSVD<Eigen::MatrixXd> svd(coefficient, ComputeFullU | ComputeFullV);
        MatrixXd vt = svd.matrixV();
        Matrix3d result;

        result <<
            vt(0, 8), vt(1, 8), vt(2, 8),
            vt(3, 8), vt(4, 8), vt(5, 8),
            vt(6, 8), vt(7, 8), vt(8, 8);

        result = result / result(2, 2); // 需要归一化

        return result;
    }

    double EstimatorEigen::CheckHomography(const Matrix3d &homography21, const Matrix3d &homography12,
                                           const std::vector<Point2d> &src_points,
                                           const std::vector<Point2d> &dst_points) const
    {
        const int number_matches = src_points.size();

        const double h11 = homography21(0, 0);
        const double h12 = homography21(0, 1);
        const double h13 = homography21(0, 2);
        const double h21 = homography21(1, 0);
        const double h22 = homography21(1, 1);
        const double h23 = homography21(1, 2);
        const double h31 = homography21(2, 0);
        const double h32 = homography21(2, 1);
        const double h33 = homography21(2, 2);

        const double h11_inv = homography12(0, 0);
        const double h12_inv = homography12(0, 1);
        const double h13_inv = homography12(0, 2);
        const double h21_inv = homography12(1, 0);
        const double h22_inv = homography12(1, 1);
        const double h23_inv = homography12(1, 2);
        const double h31_inv = homography12(2, 0);
        const double h32_inv = homography12(2, 1);
        const double h33_inv = homography12(2, 2);

        double score = 0.0;
        const double th = 5.991;

        for (int i = 0; i < number_matches; ++i)
        {
            bool in = true;

            const double u1 = src_points[i].x;
            const double v1 = src_points[i].y;
            const double u2 = dst_points[i].x;
            const double v2 = dst_points[i].y;

            const double w2_in1_inv = 1.0 / (h31_inv * u2 + h32_inv * v2 + h33_inv);
            const double u2_in1 = (h11_inv * u2 + h12_inv * v2 + h13_inv) * w2_in1_inv;
            const double v2_in1 = (h21_inv * u2 + h22_inv * v2 + h23_inv) * w2_in1_inv;

            const double square_dist1 = (u1 - u2_in1) * (u1 - u2_in1) + (v1 - v2_in1) * (v1 - v2_in1);

            if(square_dist1 > th)
                in = false;
            else
                score += th - square_dist1;

            const double w1_in2_inv = 1.0 / (h31 * u1 + h32 * v1 + h33);
            const double u1_in2 = (h11 * u1 + h12 * v1 + h13) * w1_in2_inv;
            const double v1_in2 = (h21 * u1 + h22 * v1 + h23) * w1_in2_inv;

            const double square_dist2 = (u2 - u1_in2) * (u2 - u1_in2) + (v2 - v1_in2) * (v2 - v1_in2);

            if(square_dist2 > th)
                in = false;
            else
                score += th - square_dist2;
        }

        return score;
    }

    Matrix3d EstimatorEigen::ComputeF21(std::vector<Point2d> &src_points, std::vector<Point2d> &dst_points) const
    {
        const int number_size = src_points.size();
        MatrixXd coefficient = MatrixXd::Zero(number_size, 9);


        //OpenCV版本

//        cv::Mat coefficient(number_size, 9, CV_32F);
//        for (int i = 0; i < number_size; ++i)
//        {
//            const double u1 = src_points[i].x;
//            const double v1 = src_points[i].y;
//            const double u2 = dst_points[i].x;
//            const double v2 = dst_points[i].y;
//
//            coefficient.at<float>(i, 0) = u1 * u2;
//            coefficient.at<float>(i, 1) = v1 * u2;
//            coefficient.at<float>(i, 2) = u2;
//            coefficient.at<float>(i, 3) = u1 * v2;
//            coefficient.at<float>(i, 4) = v1 * v2;
//            coefficient.at<float>(i, 5) = v2;
//            coefficient.at<float>(i, 6) = u1;
//            coefficient.at<float>(i, 7) = v1;
//            coefficient.at<float>(i, 8) = 1;
//        }
//
//        cv::Mat u, w, vt;
//        cv::SVDecomp(coefficient, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
//        cv::Mat Fpre = vt.row(8).reshape(0, 3);
//        cv::SVDecomp(Fpre, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
//        w.at<float>(2) = 0;
//        cv::Mat rst = u * cv::Mat::diag(w) * vt;
//
//        Matrix3d result;
//        result <<
//        rst.at<float>(0, 0), rst.at<float>(0, 1), rst.at<float>(0, 2),
//        rst.at<float>(1, 0), rst.at<float>(1, 1), rst.at<float>(1, 2),
//        rst.at<float>(2, 0), rst.at<float>(2, 1), rst.at<float>(2, 2);

        for (int i = 0; i < number_size; ++i)
        {
            const double u1 = src_points[i].x;
            const double v1 = src_points[i].y;
            const double u2 = dst_points[i].x;
            const double v2 = dst_points[i].y;

            coefficient(i, 0) = u1 * u2;
            coefficient(i, 1) = v1 * u2;
            coefficient(i, 2) = u2;
            coefficient(i, 3) = u1 * v2;
            coefficient(i, 4) = v1 * v2;
            coefficient(i, 5) = v2;
            coefficient(i, 6) = u1;
            coefficient(i, 7) = v1;
            coefficient(i, 8) = 1.0;
        }


        // DIYSLAM版本

//        JacobiSVD<MatrixXd> svd(coefficient, ComputeFullU | ComputeFullV);
//        const Eigen::VectorXd vt = svd.matrixV().col(8);
//        const Eigen::Map<const Eigen::Matrix3d> ematrix_t(vt.data());
//        Eigen::JacobiSVD<Eigen::Matrix3d> fmatrix_svd(ematrix_t.transpose(), ComputeFullV | ComputeFullU);
//        Eigen::Vector3d singular_values = fmatrix_svd.singularValues();
//
//        singular_values(2) = 0.0;
//
//        const Eigen::Matrix3d result = fmatrix_svd.matrixU() * singular_values.asDiagonal() * fmatrix_svd.matrixV().transpose();

        JacobiSVD<MatrixXd> svd(coefficient, ComputeFullU | ComputeFullV);
        Eigen::MatrixXd vt = svd.matrixV();
        Matrix3d result;
        result <<
        vt(0, 8), vt(1, 8), vt(2, 8),
        vt(3, 8), vt(4, 8), vt(5, 8),
        vt(6, 8), vt(7, 8), vt(8, 8);

        svd.compute(result, ComputeFullU | ComputeFullV);

        VectorXd w = svd.singularValues();
        Matrix3d w_pre = Matrix3d::Zero();

        w_pre(0, 0) = w(0);
        w_pre(1, 1) = w(1);

        result = svd.matrixU() * w_pre * svd.matrixV().transpose();

        return result;
    }

    double EstimatorEigen::CheckFundamental(const Matrix3d &fundamental21,
                                            const std::vector<Point2d> &src_points,
                                            const std::vector<Point2d> &dst_points) const
    {
        const int number_matches = src_points.size();

        const double f11 = fundamental21(0, 0);
        const double f12 = fundamental21(0, 1);
        const double f13 = fundamental21(0, 2);
        const double f21 = fundamental21(1, 0);
        const double f22 = fundamental21(1, 1);
        const double f23 = fundamental21(1, 2);
        const double f31 = fundamental21(2, 0);
        const double f32 = fundamental21(2, 1);
        const double f33 = fundamental21(2, 2);

        const double th = 3.841;
        const double th_score = 5.991;
        double score = 0.0;

        for (int i = 0; i < number_matches; ++i)
        {
            const double u1 = src_points[i].x;
            const double v1 = src_points[i].y;
            const double u2 = dst_points[i].x;
            const double v2 = dst_points[i].y;

            const double a2 = f11 * u1 + f12 * v1 + f13;
            const double b2 = f21 * u1 + f22 * v1 + f23;
            const double c2 = f31 * u1 + f32 * v1 + f33;

            const double num2 = a2 * u2 + b2 * v2 + c2;
            const double square_dist1 = num2 * num2 / (a2 * a2 + b2 * b2);

            if (square_dist1 <= th)
            {
                score += th_score - square_dist1;
            }
        }

        return score;
    }

    Matrix3d EstimatorEigen::ComputeE21(const std::vector<Point2d> &src_points,
                                        const std::vector<Point2d> &dst_points) const
    {
        Eigen::Matrix<double, Eigen::Dynamic, 9> coefficient(src_points.size(), 9);

        for (int i = 0; i < src_points.size(); ++i)
        {
            const double u1 = src_points[i].x;
            const double v1 = src_points[i].y;
            const double u2 = dst_points[i].x;
            const double v2 = dst_points[i].y;

            coefficient(i, 0) = u1 * u2;
            coefficient(i, 1) = v1 * u2;
            coefficient(i, 2) = u2;
            coefficient(i, 3) = u1 * v2;
            coefficient(i, 4) = v1 * v2;
            coefficient(i, 5) = v2;
            coefficient(i, 6) = u1;
            coefficient(i, 7) = v1;
            coefficient(i, 8) = 1.0;
        }

        const Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9> > svd(coefficient, Eigen::ComputeFullV);
        cout << svd.singularValues() << endl;
        cout << svd.matrixV() << endl;
        const Eigen::Matrix<double, 9, 4> essential = svd.matrixV().block<9, 4>(0, 5);

        cout << endl;
        cout << essential << endl;
    }
}
