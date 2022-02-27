#pragma once

#include <vector>
using namespace std;

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "random_array.h"
using namespace Eigen;

#include <ros/console.h>

class MotionEstimator
{
public:
  Eigen::Matrix3d myfindFundamentalMat(vector<Eigen::Vector3d> corr1, vector<Eigen::Vector3d> corr2, int method, double theshold, double unknown, vector<uchar> &status);
  Eigen::Matrix3d compute_E_21(vector<Eigen::Vector3d> &bearings_1, vector<Eigen::Vector3d> &bearings_2);
  double check_inliers(Eigen::Matrix3d &E_21, vector<uchar> &is_inlier_match,
                       vector<Eigen::Vector3d> corr1, vector<Eigen::Vector3d> corr2);
  bool solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &R, Vector3d &T);

private:
  double testTriangulation(const vector<cv::Point2f> &l,
                           const vector<cv::Point2f> &r,
                           cv::Mat_<double> R, cv::Mat_<double> t);
  void decomposeE(cv::Mat E,
                  cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                  cv::Mat_<double> &t1, cv::Mat_<double> &t2);
  void decomposeEssentialMat(Eigen::Matrix3d _E, Eigen::Matrix3d &_R1, Eigen::Matrix3d &_R2, Eigen::Vector3d &_t);
  void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        const Vector3d &point0, const Vector3d &point1, Vector3d &point_3d);
  void triangulatePoints(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                         const vector<Vector3d> &points0, const vector<Vector3d> &points1, vector<Vector3d> &points_3d);
  int recoverPose(Eigen::Matrix3d E, vector<Eigen::Vector3d> _points1, vector<Eigen::Vector3d> _points2,
                  Matrix3d _R, Vector3d _t, vector<uchar> _mask);
};
