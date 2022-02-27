#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"


#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class Pointcloud_image_fusion
{
  public:
    Pointcloud_image_fusion();

    cv::Mat euqual_rectangle_expansion(cv::Mat &img,int rows,int cols);

    void GetremapMat(cv::Mat &remap_x,cv::Mat &remap_y,int rows,int cols);

    void readImage(const cv::Mat &_img,double _cur_time);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();

    void undistortedPoints();

    void mygoodFeaturesToTrack( cv::InputArray _image, cv::OutputArray _corners,
                              int maxCorners, double qualityLevel, double minDistance,
                              cv::InputArray _mask, int blockSize=3,
                              bool useHarrisDetector=false, double harrisK=0.04 );

    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    //added by wz
    vector<cv::Point3f> cur_un_pts_3d;
    vector<cv::Point2f> pts_velocity;
    vector<cv::Point3f> pts_velocity_3d;
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
