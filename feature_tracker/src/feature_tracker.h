#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"
#include "random_array.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);


//added by wz
Eigen::Matrix3d myfindFundamentalMat(vector<Eigen::Vector3d> corr1,vector<Eigen::Vector3d> corr2,int method , double theshold , double unknown ,vector<uchar> &status);
Eigen::Matrix3d compute_E_21(vector<Eigen::Vector3d> & bearings_1,vector<Eigen::Vector3d> & bearings_2);
double check_inliers(Eigen::Matrix3d &E_21, vector<uchar> &is_inlier_match,vector<Eigen::Vector3d> corr1,vector<Eigen::Vector3d> corr2);
class FeatureTracker
{
  public:
    FeatureTracker();

    void readImage(const cv::Mat &_img,double _cur_time);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();

    void undistortedPoints();

    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    //changed by wz
    vector<cv::Point2f> pts_velocity;
    vector<cv::Point3f> pts_velocity_3d;

    //added by wz
    vector<cv::Point3f> cur_un_pts_3d;

    vector<int> ids;
    vector<int> track_cnt;
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
    //added by wz
    map<int, cv::Point3f> cur_un_pts_map_3d;
    map<int, cv::Point3f> prev_un_pts_map_3d;
    camodocal::CameraPtr m_camera;
    double cur_time;
    double prev_time;

    static int n_id;
};
