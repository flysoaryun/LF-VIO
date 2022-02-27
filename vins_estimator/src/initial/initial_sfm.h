#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
using namespace Eigen;
using namespace std;

struct SFMFeature
{
	bool state;
	int id;
	//changed by wz
	vector<pair<int, Vector3d>> observation;

	double position[3];
	double depth;
};

struct ReprojectionError3D
{
	ReprojectionError3D(double observed_u, double observed_v, double observed_z)
		: observed_u(observed_u), observed_v(observed_v), observed_z(observed_z)
	{
	}

	//changed by wz
	template <typename T>
	bool operator()(const T *const camera_R, const T *const camera_T, const T *point, T *residuals) const
	{
		T p[3];
		ceres::QuaternionRotatePoint(camera_R, point, p);
		p[0] += camera_T[0];
		p[1] += camera_T[1];
		p[2] += camera_T[2];
		T xp = p[0] / sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
		T yp = p[1] / sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
		T zp = p[2] / sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
		residuals[0] = xp - T(observed_u);
		residuals[1] = yp - T(observed_v);
		residuals[2] = zp - T(observed_z);
		return true;
	}

	//changed by wz
	static ceres::CostFunction *Create(const double observed_x,
									   const double observed_y,
									   const double observed_z)
	{
		return (new ceres::AutoDiffCostFunction<
				ReprojectionError3D, 3, 4, 3, 3>(
			new ReprojectionError3D(observed_x, observed_y, observed_z)));
	}

	double observed_u;
	double observed_v;
	double observed_z;
};

class GlobalSFM
{
public:
	GlobalSFM();
	bool construct(int frame_num, Quaterniond *q, Vector3d *T, int l,
				   const Matrix3d relative_R, const Vector3d relative_T,
				   vector<SFMFeature> &sfm_f, map<int, Vector3d> &sfm_tracked_points);

private:
	bool solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i, vector<SFMFeature> &sfm_f);
	//changed by wz
	void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
						  Vector3d &point0, Vector3d &point1, Vector3d &point_3d);
	void triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0,
							  int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
							  vector<SFMFeature> &sfm_f);

	int feature_num;
};