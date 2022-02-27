// changed by wz
#pragma once

#include <vector>
#include "../parameters.h"
using namespace std;

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>
using namespace Eigen;
#include <ros/console.h>
#include "random_array.h"

/* This class help you to calibrate extrinsic rotation between imu and camera when your totally don't konw the extrinsic parameter */
class InitialEXRotation
{
public:
    InitialEXRotation();
    bool CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result);

private:
    Eigen::Matrix3d compute_E_21(vector<Eigen::Vector3d> &bearings_1, vector<Eigen::Vector3d> &bearings_2);
    double check_inliers(Eigen::Matrix3d &E_21, vector<uchar> &is_inlier_match, vector<Eigen::Vector3d> corr1, vector<Eigen::Vector3d> corr2);
    Eigen::Matrix3d myfindFundamentalMat(vector<Eigen::Vector3d> corr1, vector<Eigen::Vector3d> corr2);

    Matrix3d solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres);

    double testTriangulation(const vector<Vector3d> &l,
                             const vector<Vector3d> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                          const Vector3d &point0, const Vector3d &point1, Vector3d &point_3d);
    void triangulatePoints(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                           const vector<Vector3d> &points0, const vector<Vector3d> &points1, vector<Vector3d> &points_3d);
    int frame_count;

    vector<Matrix3d> Rc;
    vector<Matrix3d> Rimu;
    vector<Matrix3d> Rc_g;
    Matrix3d ric;
};
