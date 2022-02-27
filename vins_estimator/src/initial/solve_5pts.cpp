#include "solve_5pts.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

namespace cv
{
    void decomposeEssentialMat(InputArray _E, OutputArray _R1, OutputArray _R2, OutputArray _t)
    {

        Mat E = _E.getMat().reshape(1, 3);
        CV_Assert(E.cols == 3 && E.rows == 3);

        Mat D, U, Vt;
        SVD::compute(E, D, U, Vt);

        if (determinant(U) < 0)
            U *= -1.;
        if (determinant(Vt) < 0)
            Vt *= -1.;

        Mat W = (Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
        W.convertTo(W, E.type());

        Mat R1, R2, t;
        R1 = U * W * Vt;
        R2 = U * W.t() * Vt;
        t = U.col(2) * 1.0;

        R1.copyTo(_R1);
        R2.copyTo(_R2);
        t.copyTo(_t);
    }

    int recoverPose(InputArray E, InputArray _points1, InputArray _points2, InputArray _cameraMatrix,
                    OutputArray _R, OutputArray _t, InputOutputArray _mask)
    {

        Mat points1, points2, cameraMatrix;
        _points1.getMat().convertTo(points1, CV_64F);
        _points2.getMat().convertTo(points2, CV_64F);
        _cameraMatrix.getMat().convertTo(cameraMatrix, CV_64F);

        int npoints = points1.checkVector(2);
        CV_Assert(npoints >= 0 && points2.checkVector(2) == npoints &&
                  points1.type() == points2.type());

        CV_Assert(cameraMatrix.rows == 3 && cameraMatrix.cols == 3 && cameraMatrix.channels() == 1);

        if (points1.channels() > 1)
        {
            points1 = points1.reshape(1, npoints);
            points2 = points2.reshape(1, npoints);
        }

        double fx = cameraMatrix.at<double>(0, 0);
        double fy = cameraMatrix.at<double>(1, 1);
        double cx = cameraMatrix.at<double>(0, 2);
        double cy = cameraMatrix.at<double>(1, 2);

        points1.col(0) = (points1.col(0) - cx) / fx;
        points2.col(0) = (points2.col(0) - cx) / fx;
        points1.col(1) = (points1.col(1) - cy) / fy;
        points2.col(1) = (points2.col(1) - cy) / fy;

        points1 = points1.t();
        points2 = points2.t();

        Mat R1, R2, t;
        decomposeEssentialMat(E, R1, R2, t);
        Mat P0 = Mat::eye(3, 4, R1.type());
        Mat P1(3, 4, R1.type()), P2(3, 4, R1.type()), P3(3, 4, R1.type()), P4(3, 4, R1.type());

        P1(Range::all(), Range(0, 3)) = R1 * 1.0;
        P1.col(3) = t * 1.0;
        P2(Range::all(), Range(0, 3)) = R2 * 1.0;
        P2.col(3) = t * 1.0;
        P3(Range::all(), Range(0, 3)) = R1 * 1.0;
        P3.col(3) = -t * 1.0;
        P4(Range::all(), Range(0, 3)) = R2 * 1.0;
        P4.col(3) = -t * 1.0;

        double dist = 50.0;
        Mat Q;
        triangulatePoints(P0, P1, points1, points2, Q);
        Mat mask1 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask1 = (Q.row(2) < dist) & mask1;
        Q = P1 * Q;
        mask1 = (Q.row(2) > 0) & mask1;
        mask1 = (Q.row(2) < dist) & mask1;

        triangulatePoints(P0, P2, points1, points2, Q);
        Mat mask2 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask2 = (Q.row(2) < dist) & mask2;
        Q = P2 * Q;
        mask2 = (Q.row(2) > 0) & mask2;
        mask2 = (Q.row(2) < dist) & mask2;

        triangulatePoints(P0, P3, points1, points2, Q);
        Mat mask3 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask3 = (Q.row(2) < dist) & mask3;
        Q = P3 * Q;
        mask3 = (Q.row(2) > 0) & mask3;
        mask3 = (Q.row(2) < dist) & mask3;

        triangulatePoints(P0, P4, points1, points2, Q);
        Mat mask4 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask4 = (Q.row(2) < dist) & mask4;
        Q = P4 * Q;
        mask4 = (Q.row(2) > 0) & mask4;
        mask4 = (Q.row(2) < dist) & mask4;

        mask1 = mask1.t();
        mask2 = mask2.t();
        mask3 = mask3.t();
        mask4 = mask4.t();

        // If _mask is given, then use it to filter outliers.
        if (!_mask.empty())
        {
            Mat mask = _mask.getMat();
            CV_Assert(mask.size() == mask1.size());
            bitwise_and(mask, mask1, mask1);
            bitwise_and(mask, mask2, mask2);
            bitwise_and(mask, mask3, mask3);
            bitwise_and(mask, mask4, mask4);
        }
        if (_mask.empty() && _mask.needed())
        {
            _mask.create(mask1.size(), CV_8U);
        }

        CV_Assert(_R.needed() && _t.needed());
        _R.create(3, 3, R1.type());
        _t.create(3, 1, t.type());

        int good1 = countNonZero(mask1);
        int good2 = countNonZero(mask2);
        int good3 = countNonZero(mask3);
        int good4 = countNonZero(mask4);

        if (good1 >= good2 && good1 >= good3 && good1 >= good4)
        {
            R1.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed())
                mask1.copyTo(_mask);
            return good1;
        }
        else if (good2 >= good1 && good2 >= good3 && good2 >= good4)
        {
            R2.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed())
                mask2.copyTo(_mask);
            return good2;
        }
        else if (good3 >= good1 && good3 >= good2 && good3 >= good4)
        {
            t = -t;
            R1.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed())
                mask3.copyTo(_mask);
            return good3;
        }
        else
        {
            t = -t;
            R2.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed())
                mask4.copyTo(_mask);
            return good4;
        }
    }

    int recoverPose(InputArray E, InputArray _points1, InputArray _points2, OutputArray _R,
                    OutputArray _t, double focal, Point2d pp, InputOutputArray _mask)
    {
        Mat cameraMatrix = (Mat_<double>(3, 3) << focal, 0, pp.x, 0, focal, pp.y, 0, 0, 1);
        return cv::recoverPose(E, _points1, _points2, cameraMatrix, _R, _t, _mask);
    }
}

Eigen::Matrix3d MotionEstimator::compute_E_21(vector<Eigen::Vector3d> &bearings_1, vector<Eigen::Vector3d> &bearings_2)
{
    const auto num_points = bearings_1.size();

    typedef Eigen::Matrix<Eigen::Matrix3d::Scalar, Eigen::Dynamic, 9> CoeffMatrix;
    CoeffMatrix A(num_points, 9);

    for (unsigned int i = 0; i < num_points; i++)
    {
        A.block<1, 3>(i, 0) = bearings_2.at(i)(0) * bearings_1.at(i);
        A.block<1, 3>(i, 3) = bearings_2.at(i)(1) * bearings_1.at(i);
        A.block<1, 3>(i, 6) = bearings_2.at(i)(2) * bearings_1.at(i);
    }

    const Eigen::JacobiSVD<CoeffMatrix> init_svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Eigen::Matrix<Eigen::Matrix3d::Scalar, 9, 1> v = init_svd.matrixV().col(8);
    const Eigen::Matrix3d init_E_21 = Eigen::Matrix3d(v.data()).transpose();
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(init_E_21, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Eigen::Matrix3d &U = svd.matrixU();
    Eigen::Vector3d lambda = svd.singularValues();
    const Eigen::Matrix3d &V = svd.matrixV();

    lambda(2) = 0.0;

    const Eigen::Matrix3d E_21 = U * lambda.asDiagonal() * V.transpose();

    return E_21;
}
double MotionEstimator::check_inliers(Eigen::Matrix3d &E_21, vector<uchar> &is_inlier_match, vector<Eigen::Vector3d> corr1, vector<Eigen::Vector3d> corr2)
{
    const auto num_points = corr1.size();
    const Eigen::Matrix3d E_12 = E_21.transpose();
    float score = 0;

    constexpr float residual_cos_thr = 0.00872653549837;

    for (unsigned int i = 0; i < num_points; ++i)
    {
        const auto bearing_1 = corr1[i];
        const auto bearing_2 = corr2[i];

        const Eigen::Vector3d epiplane_in_2 = E_21 * bearing_1;
        const double residual_in_2 = std::abs(epiplane_in_2.dot(bearing_2) / epiplane_in_2.norm());
        if (residual_cos_thr < residual_in_2)
        {
            is_inlier_match.at(i) = 0;
            continue;
        }
        else
        {
            is_inlier_match.at(i) = 1;
            score += pow(residual_cos_thr - residual_in_2, 2.0);
        }

        const Eigen::Vector3d epiplane_in_1 = E_12 * bearing_2;

        const double residual_in_1 = std::abs(epiplane_in_1.dot(bearing_1) / epiplane_in_1.norm());
        if (residual_cos_thr < residual_in_1)
        {
            is_inlier_match.at(i) = 0;
            continue;
        }
        else
        {
            is_inlier_match.at(i) = 1;
            score += pow(residual_cos_thr - residual_in_1, 2.0);
        }
    }

    return score;
}

Eigen::Matrix3d MotionEstimator::myfindFundamentalMat(vector<Eigen::Vector3d> corr1, vector<Eigen::Vector3d> corr2, int method, double theshold, double unknown, vector<uchar> &status)
{
    int size1 = corr1.size();
    int size2 = corr1.size();

    constexpr unsigned int min_set_size = 8;

    if (size1 != size2 | size1 < 8 | size2 < 8)
    {
        ROS_ERROR("findFundamentalMat fault");
    }
    const unsigned int num_matches = size1;
    Eigen::Matrix3d best_E_21_;
    double best_score_ = 0.0;
    int max_num_iter = 100;

    vector<Eigen::Vector3d> min_set_bearings_1(min_set_size);
    vector<Eigen::Vector3d> min_set_bearings_2(min_set_size);
    vector<uchar> is_inlier_match_in_sac;
    vector<uchar> is_inlier_match_;

    Eigen::Matrix3d E_21_in_sac;
    double score_in_sac = 0.0;
    for (unsigned int iter = 0; iter < max_num_iter; iter++)
    {
        const auto indices = util::create_random_array(min_set_size, 0U, num_matches - 1);
        for (unsigned int i = 0; i < min_set_size; ++i)
        {
            const auto idx = indices.at(i);
            min_set_bearings_1.at(i) = corr1[idx];
            min_set_bearings_2.at(i) = corr2[idx];
        }
        E_21_in_sac = compute_E_21(min_set_bearings_1, min_set_bearings_2);
        is_inlier_match_in_sac.resize(num_matches, 0);
        score_in_sac = check_inliers(E_21_in_sac, is_inlier_match_in_sac, corr1, corr2);

        if (best_score_ < score_in_sac)
        {
            best_score_ = score_in_sac;
            best_E_21_ = E_21_in_sac;
            is_inlier_match_ = is_inlier_match_in_sac;
        }
    }
    ROS_INFO("best_score_:%f", best_score_);
    vector<Eigen::Vector3d> inlier_bearing_1;
    vector<Eigen::Vector3d> inlier_bearing_2;
    inlier_bearing_1.reserve(num_matches);
    inlier_bearing_2.reserve(num_matches);
    for (unsigned int i = 0; i < num_matches; ++i)
    {
        if (is_inlier_match_.at(i))
        {
            inlier_bearing_1.push_back(corr1[i]);
            inlier_bearing_2.push_back(corr2[i]);
        }
    }
    best_E_21_ = compute_E_21(inlier_bearing_1, inlier_bearing_2);
    best_score_ = check_inliers(best_E_21_, is_inlier_match_, corr1, corr2);
    status = is_inlier_match_;
    return best_E_21_;
}

void MotionEstimator::decomposeEssentialMat(Eigen::Matrix3d _E, Eigen::Matrix3d &_R1, Eigen::Matrix3d &_R2, Eigen::Vector3d &_t)
{

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(_E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Vector3d lambda = svd.singularValues();
    Eigen::Matrix3d V = svd.matrixV();

    if (U.determinant() < 0)
        U = -U;
    if (V.determinant() < 0)
        V = -V;

    Eigen::Matrix3d W;
    W << 0, 1, 0, -1, 0, 0, 0, 0, 1;

    Eigen::Matrix3d R1, R2;
    Eigen::Vector3d t;
    R1 = U * W * V.transpose();
    R2 = U * W.transpose() * V.transpose();
    t = U.col(2) * 1.0;

    _R1 = R1;
    _R2 = R2;
    _t = t;
}

void MotionEstimator::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                                       const Vector3d &point0, const Vector3d &point1, Vector3d &point_3d)
{
    Matrix4d design_matrix = Matrix4d::Zero();
    // changed by wz
    design_matrix.row(0) = point0[0] * Pose0.row(2) - point0[2] * Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - point0[2] * Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - point1[2] * Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - point1[2] * Pose1.row(1);
    Vector4d triangulated_point;
    triangulated_point =
        design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

void MotionEstimator::triangulatePoints(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                                        const vector<Vector3d> &points0, const vector<Vector3d> &points1, vector<Vector3d> &points_3d)
{
    Eigen::Matrix<double, 3, 4> P0 = Pose0;
    Eigen::Matrix<double, 3, 4> P1 = Pose1;
    points_3d.clear();
    points_3d.resize(points0.size());
    for (auto i = 0; i < points0.size(); i++)
    {
        Vector3d tmp_point_3d;
        triangulatePoint(P0, P1, points0[i], points1[i], tmp_point_3d);
        points_3d[i] = tmp_point_3d;
    }
}
int MotionEstimator::recoverPose(Eigen::Matrix3d E, vector<Eigen::Vector3d> _points1, vector<Eigen::Vector3d> _points2,
                                 Matrix3d _R, Vector3d _t, vector<uchar> _mask)
{
    Eigen::Matrix3d R1, R2;
    Eigen::Vector3d t;

    decomposeEssentialMat(E, R1, R2, t);
    Eigen::Matrix<double, 3, 4> P0, P1, P2, P3, P4;
    P0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0;
    P1.block<3, 3>(0, 0) = R1;
    P1.block<3, 1>(0, 3) = t;

    P2.block<3, 3>(0, 0) = R2;
    P2.block<3, 1>(0, 3) = t;

    P3.block<3, 3>(0, 0) = R1;
    P3.block<3, 1>(0, 3) = -t;

    P4.block<3, 3>(0, 0) = R2;
    P4.block<3, 1>(0, 3) = -t;

    double dist = 50.0;
    vector<Vector3d> Q;
    int good1 = 0, good2 = 0, good3 = 0, good4 = 0;
    vector<uchar> mask1, mask2, mask3, mask4;
    std::cout << "E\n"
              << E << std::endl;
    std::cout << "R1\n"
              << R1 << std::endl;
    std::cout << "R2\n"
              << R2 << std::endl;
    std::cout << "t\n"
              << t << std::endl;
    triangulatePoints(P0, P1, _points1, _points2, Q);
    std::cout << "Q.size():" << Q.size() << std::endl;
    std::cout << "_points1.size():" << _points1.size() << std::endl;
    std::cout << "_points2.size():" << _points2.size() << std::endl;
    for (int i = 0; i < Q.size(); i++)
    {
        double p_3d_l_dot = _points1[i].dot(Q[i]) / Q[i].norm();
        Vector3d tmp_Q = P1.block<3, 3>(0, 0) * Q[i] + P1.block<3, 1>(0, 3);
        double p_3d_r_dot = _points2[i].dot(tmp_Q) / tmp_Q.norm();

        if (p_3d_l_dot > 0 && p_3d_r_dot > 0)
        {
            good1++;
            mask1.push_back(1);
        }
        else
        {
            mask1.push_back(0);
        }
    }
    triangulatePoints(P0, P2, _points1, _points2, Q);
    for (int i = 0; i < Q.size(); i++)
    {
        double p_3d_l_dot = _points1[i].dot(Q[i]) / Q[i].norm();
        Vector3d tmp_Q = P2.block<3, 3>(0, 0) * Q[i] + P2.block<3, 1>(0, 3);
        double p_3d_r_dot = _points2[i].dot(tmp_Q) / tmp_Q.norm();

        if (p_3d_l_dot > 0 && p_3d_r_dot > 0)
        {
            good2++;
            mask2.push_back(1);
        }
        else
        {
            mask2.push_back(0);
        }
    }
    triangulatePoints(P0, P3, _points1, _points2, Q);
    for (int i = 0; i < Q.size(); i++)
    {
        double p_3d_l_dot = _points1[i].dot(Q[i]) / Q[i].norm();
        Vector3d tmp_Q = P3.block<3, 3>(0, 0) * Q[i] + P3.block<3, 1>(0, 3);
        double p_3d_r_dot = _points2[i].dot(tmp_Q) / tmp_Q.norm();

        if (p_3d_l_dot > 0 && p_3d_r_dot > 0)
        {
            good3++;
            mask3.push_back(1);
        }
        else
        {
            mask3.push_back(0);
        }
    }
    triangulatePoints(P0, P4, _points1, _points2, Q);
    for (int i = 0; i < Q.size(); i++)
    {
        double p_3d_l_dot = _points1[i].dot(Q[i]) / Q[i].norm();
        Vector3d tmp_Q = P4.block<3, 3>(0, 0) * Q[i] + P4.block<3, 1>(0, 3);
        double p_3d_r_dot = _points2[i].dot(tmp_Q) / tmp_Q.norm();

        if (p_3d_l_dot > 0 && p_3d_r_dot > 0)
        {

            good4++;
            mask4.push_back(1);
        }
        else
        {
            mask4.push_back(0);
        }
    }

    std::cout << "good1:" << good1 << std::endl;
    std::cout << "good2:" << good2 << std::endl;
    std::cout << "good3:" << good3 << std::endl;
    std::cout << "good4:" << good4 << std::endl;
    if (good1 >= good2 && good1 >= good3 && good1 >= good4)
    {
        _R = R1;
        _t = t;
        _mask = mask1;
        return good1;
    }
    else if (good2 >= good1 && good2 >= good3 && good2 >= good4)
    {
        _R = R2;
        _t = t;
        _mask = mask2;
        return good2;
    }
    else if (good3 >= good1 && good3 >= good2 && good3 >= good4)
    {
        _R = R1;
        _t = -t;
        _mask = mask3;
        return good3;
    }
    else
    {
        _R = R2;
        _t = -t;
        _mask = mask4;
        return good4;
    }
}
bool MotionEstimator::solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &Rotation, Vector3d &Translation)
{
    if (corres.size() >= 15)
    {
        vector<cv::Point2f> ll, rr;
        for (int i = 0; i < int(corres.size()); i++)
        {
            ll.push_back(cv::Point2f(corres[i].first(0) / corres[i].first(2), corres[i].first(1) / corres[i].first(2)));
            rr.push_back(cv::Point2f(corres[i].second(0) / corres[i].second(2), corres[i].second(1) / corres[i].second(2)));
        }

        vector<Eigen::Vector3d> ll_3d, rr_3d;
        for (int i = 0; i < int(corres.size()); i++)
        {
            ll_3d.push_back(corres[i].first);
            rr_3d.push_back(corres[i].second);
        }
        vector<uchar> state;
        Eigen::Matrix3d E_my = myfindFundamentalMat(ll_3d, rr_3d, cv::FM_RANSAC, 0.3 / 460, 0.99, state);

        cv::Mat true_E = cv::findFundamentalMat(ll, rr);
        Eigen::Matrix3d cv_E;
        cv::cv2eigen(true_E, cv_E);

        Matrix3d rot;
        Vector3d trans;
        vector<uchar> mask;

        int inlier_cnt = recoverPose(E_my, ll_3d, rr_3d, rot, trans, mask);

        Eigen::Matrix3d R;
        Eigen::Vector3d T;
        Rotation = R.transpose();
        Translation = -R.transpose() * T;
        if (inlier_cnt > 12)
            return true;
        else
            return false;
    }
    return false;
}