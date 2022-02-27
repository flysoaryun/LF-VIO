// changed by wz
#include "initial_ex_rotation.h"

InitialEXRotation::InitialEXRotation()
{
    frame_count = 0;
    Rc.push_back(Matrix3d::Identity());
    Rc_g.push_back(Matrix3d::Identity());
    Rimu.push_back(Matrix3d::Identity());
    ric = Matrix3d::Identity();
}

bool InitialEXRotation::CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result)
{
    frame_count++;
    Rc.push_back(solveRelativeR(corres));
    Rimu.push_back(delta_q_imu.toRotationMatrix());
    Rc_g.push_back(ric.inverse() * delta_q_imu * ric);

    Eigen::MatrixXd A(frame_count * 4, 4);
    A.setZero();
    int sum_ok = 0;
    for (int i = 1; i <= frame_count; i++)
    {
        Quaterniond r1(Rc[i]);
        Quaterniond r2(Rc_g[i]);

        double angular_distance = 180 / M_PI * r1.angularDistance(r2);
        ROS_DEBUG(
            "%d %f", i, angular_distance);

        double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;
        ++sum_ok;
        Matrix4d L, R;

        double w = Quaterniond(Rc[i]).w();
        Vector3d q = Quaterniond(Rc[i]).vec();
        L.block<3, 3>(0, 0) = w * Matrix3d::Identity() + Utility::skewSymmetric(q);
        L.block<3, 1>(0, 3) = q;
        L.block<1, 3>(3, 0) = -q.transpose();
        L(3, 3) = w;

        Quaterniond R_ij(Rimu[i]);
        w = R_ij.w();
        q = R_ij.vec();
        R.block<3, 3>(0, 0) = w * Matrix3d::Identity() - Utility::skewSymmetric(q);
        R.block<3, 1>(0, 3) = q;
        R.block<1, 3>(3, 0) = -q.transpose();
        R(3, 3) = w;

        A.block<4, 4>((i - 1) * 4, 0) = huber * (L - R);
    }

    JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
    Matrix<double, 4, 1> x = svd.matrixV().col(3);
    Quaterniond estimated_R(x);
    ric = estimated_R.toRotationMatrix().inverse();
    Vector3d ric_cov;
    ric_cov = svd.singularValues().tail<3>();
    if (frame_count >= WINDOW_SIZE && ric_cov(1) > 0.25)
    {
        calib_ric_result = ric;
        return true;
    }
    else
        return false;
}

Eigen::Matrix3d InitialEXRotation::compute_E_21(vector<Eigen::Vector3d> &bearings_1, vector<Eigen::Vector3d> &bearings_2)
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
    // need transpose() because elements are contained as col-major after it was constructed from a pointer
    const Eigen::Matrix3d init_E_21 = Eigen::Matrix3d(v.data()).transpose();

    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(init_E_21, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Eigen::Matrix3d &U = svd.matrixU();
    Eigen::Vector3d lambda = svd.singularValues();
    const Eigen::Matrix3d &V = svd.matrixV();

    lambda(2) = 0.0;

    const Eigen::Matrix3d E_21 = U * lambda.asDiagonal() * V.transpose();

    return E_21;
}
double InitialEXRotation::check_inliers(Eigen::Matrix3d &E_21, vector<uchar> &is_inlier_match, vector<Eigen::Vector3d> corr1, vector<Eigen::Vector3d> corr2)
{
    const auto num_points = corr1.size();
    const Eigen::Matrix3d E_12 = E_21.transpose();
    float score = 0;

    // outlier threshold as cosine value between a bearing vector and a normal vector of the epipolar plane
    // constexpr float residual_cos_thr = 0.01745240643;
    constexpr float residual_cos_thr = 0.00872653549837;

    for (unsigned int i = 0; i < num_points; ++i)
    {
        const auto bearing_1 = corr1[i];
        const auto bearing_2 = corr2[i];

        // 1. Transform a point in shot 1 to the epipolar plane in shot 2,
        //    then compute a transfer error (= dot product)

        const Eigen::Vector3d epiplane_in_2 = E_21 * bearing_1;
        const double residual_in_2 = std::abs(epiplane_in_2.dot(bearing_2) / epiplane_in_2.norm());
        // if a match is inlier, accumulate the score
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

        // 2. Transform a point in shot 2 to the epipolar plane in shot 1,
        //    then compute a transfer error (= dot product)

        const Eigen::Vector3d epiplane_in_1 = E_12 * bearing_2;

        const double residual_in_1 = std::abs(epiplane_in_1.dot(bearing_1) / epiplane_in_1.norm());
        // if a match is inlier, accumulate the score
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

Eigen::Matrix3d InitialEXRotation::myfindFundamentalMat(vector<Eigen::Vector3d> corr1, vector<Eigen::Vector3d> corr2)
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
    return best_E_21_;
}

Matrix3d InitialEXRotation::solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres)
{
    if (corres.size() >= 9)
    {

        vector<cv::Point2f> ll, rr;
        for (int i = 0; i < int(corres.size()); i++)
        {
            // changed by wz
            ll.push_back(cv::Point2f(corres[i].first(0) / corres[i].first(2), corres[i].first(1) / corres[i].first(2)));
            rr.push_back(cv::Point2f(corres[i].second(0) / corres[i].second(2), corres[i].second(1) / corres[i].second(2)));
        }
        cv::Mat true_E = cv::findFundamentalMat(ll, rr);
        cout << "true_E\n"
             << true_E << endl;

        vector<Eigen::Vector3d> ll_3d, rr_3d;
        for (int i = 0; i < int(corres.size()); i++)
        {
            ll_3d.push_back(corres[i].first);
            rr_3d.push_back(corres[i].second);
        }

        Eigen::Matrix3d E_my = myfindFundamentalMat(ll_3d, rr_3d);

        cout << "E_my\n"
             << E_my / E_my(2, 2) << endl;

        for (int i = 0; i < int(corres.size()); i++)
        {
            Matrix3d cvf;
            cv2eigen(true_E, cvf);
            cout << "corres[i].first:\n"
                 << corres[i].first << endl;
            cout << "corres[i].second:\n"
                 << corres[i].second << endl;

            double cvf_ans = (corres[i].second / corres[i].second(2)).transpose() * cvf * (corres[i].first / corres[i].first(2));
            double myans = corres[i].second.transpose() * E_my * corres[i].first;
            double myans_false = corres[i].first.transpose() * E_my * corres[i].second;
            cout << "cvf_ans" << cvf_ans << endl;
            cout << "myans" << myans << endl;
            cout << "myans_false" << myans_false << endl;
        }

        cv::Mat_<double> R1, R2, t1, t2;
        cv::Mat E;
        eigen2cv(E_my, E);
        decomposeE(E, R1, R2, t1, t2);

        if (determinant(R1) + 1.0 < 1e-09)
        {
            E = -E;
            decomposeE(E, R1, R2, t1, t2);
        }
        double ratio1 = max(testTriangulation(ll_3d, rr_3d, R1, t1), testTriangulation(ll_3d, rr_3d, R1, t2));
        double ratio2 = max(testTriangulation(ll_3d, rr_3d, R2, t1), testTriangulation(ll_3d, rr_3d, R2, t2));
        cv::Mat_<double> ans_R_cv = ratio1 > ratio2 ? R1 : R2;

        Matrix3d ans_R_eigen;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ans_R_eigen(j, i) = ans_R_cv(i, j);
        return ans_R_eigen;
    }
    return Matrix3d::Identity();
}

double InitialEXRotation::testTriangulation(const vector<Vector3d> &l,
                                            const vector<Vector3d> &r,
                                            cv::Mat_<double> R, cv::Mat_<double> t)
{
    vector<Vector3d> pointcloud;
    Eigen::Matrix<double, 3, 4> P;
    P << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0;
    Eigen::Matrix<double, 3, 4> P1;
    P1 << R(0, 0), R(0, 1), R(0, 2), t(0),
        R(1, 0), R(1, 1), R(1, 2), t(1),
        R(2, 0), R(2, 1), R(2, 2), t(2);
    triangulatePoints(P, P1, l, r, pointcloud);
    int front_count = 0;
    if (pointcloud.size() != l.size())
    {
        ROS_ERROR("pointcloud.size()!=l.size");
    }
    for (int i = 0; i < pointcloud.size(); i++)
    {
        double p_3d_l_dot = l[i].dot(pointcloud[i]);
        double p_3d_r_dot = r[i].dot(P1.block<3, 3>(0, 0) * pointcloud[i] + P1.block<3, 1>(0, 3));
        if (p_3d_l_dot > 0 && p_3d_r_dot > 0)
        {
            front_count++;
        }
    }
    ROS_DEBUG("MotionEstimator: %f", 1.0 * front_count / pointcloud.size());
    return 1.0 * front_count / pointcloud.size();
}

void InitialEXRotation::decomposeE(cv::Mat E,
                                   cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                                   cv::Mat_<double> &t1, cv::Mat_<double> &t2)
{
    cv::SVD svd(E, cv::SVD::MODIFY_A);
    cv::Matx33d W(0, -1, 0,
                  1, 0, 0,
                  0, 0, 1);
    cv::Matx33d Wt(0, 1, 0,
                   -1, 0, 0,
                   0, 0, 1);
    R1 = svd.u * cv::Mat(W) * svd.vt;
    R2 = svd.u * cv::Mat(Wt) * svd.vt;
    t1 = svd.u.col(2);
    t2 = -svd.u.col(2);
}

void InitialEXRotation::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
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

void InitialEXRotation::triangulatePoints(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
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