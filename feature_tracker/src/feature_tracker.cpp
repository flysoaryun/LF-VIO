#include "feature_tracker.h"
#include "random_array.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

// added by wz
void reduceVector(vector<cv::Point3f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker()
{
}

void FeatureTracker::setMask()
{
    if (FISHEYE)
        mask = fisheye_mask.clone();
    else
    {
        // mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(0));
        // added by wz
        cv::circle(mask, cv::Point(CENTER_X, CENTER_Y), MAX_R, cv::Scalar(255), -1);
        cv::circle(mask, cv::Point(CENTER_X, CENTER_Y), MIN_R, cv::Scalar(0), -1);
    }

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    vector<pair<int, pair<cv::Point3f, int>>> cnt_pts_id_3d;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         { return a.first > b.first; });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(41, 41), 3);

        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);

        // added by wz
        reduceVector(cur_un_pts_3d, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt)
        n++;

    if (PUB_THIS_FRAME)
    {
        rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if (mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    undistortedPoints();
    prev_time = cur_time;
}

Eigen::Matrix3d compute_E_21(vector<Eigen::Vector3d> &bearings_1, vector<Eigen::Vector3d> &bearings_2)
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
double check_inliers(Eigen::Matrix3d &E_21, vector<uchar> &is_inlier_match, vector<Eigen::Vector3d> corr1, vector<Eigen::Vector3d> corr2)
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

Eigen::Matrix3d myfindFundamentalMat(vector<Eigen::Vector3d> corr1, vector<Eigen::Vector3d> corr2, int method, double theshold, double unknown, vector<uchar> &status)
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
    TicToc t_ff;
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
    status = is_inlier_match_;
    return best_E_21_;
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<Eigen::Vector3d> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        vector<cv::Point2f> cv_un_cur_pts(cur_pts.size()), cv_un_forw_pts(forw_pts.size());
        vector<cv::Point2f> cv2_un_cur_pts(cur_pts.size()), cv2_un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p = tmp_p / tmp_p.norm();
            un_cur_pts.at(i) = tmp_p;

            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            cv2_un_cur_pts[i] = cv::Point2f(tmp_p.x() / tmp_p.z(), tmp_p.y() / tmp_p.z());
            tmp_p.x() = 160 * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = 160 * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            cv_un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p = tmp_p / tmp_p.norm();
            un_forw_pts.at(i) = tmp_p;

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            cv2_un_forw_pts[i] = cv::Point2f(tmp_p.x() / tmp_p.z(), tmp_p.y() / tmp_p.z());
            tmp_p.x() = 160 * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = 160 * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            cv_un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }
        ROS_INFO("1: %fms", t_f.toc());
        vector<uchar> status;
        Eigen::Matrix3d my_E;
        my_E = myfindFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        int count_0 = 0;

        for (auto i = 0; i < status.size(); i++)
        {
            if (status[i] == 0)
                count_0++;
        };

        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);

        // added by wz
        reduceVector(cur_un_pts_3d, status);

        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_INFO("FM ransac costs: %fms", t_f.toc());
    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        // cout << trackerData[0].K << endl;
        // printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        // printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            // ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();

    // added by wz
    cur_un_pts_3d.clear();

    cur_un_pts_map.clear();

    cur_un_pts_map_3d.clear();
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);

        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_3d.push_back(cv::Point3f(b.x() / b.norm(), b.y() / b.norm(), b.z() / b.norm()));

        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        cur_un_pts_map_3d.insert(make_pair(ids[i], cv::Point3f(b.x() / b.norm(), b.y() / b.norm(), b.z() / b.norm())));
    }
    // caculate points velocity
    // changed by wz
    if (!prev_un_pts_map_3d.empty())
    {
        double dt = cur_time - prev_time;

        // added by wz
        pts_velocity_3d.clear();
        for (unsigned int i = 0; i < cur_un_pts_3d.size(); i++)
        {
            if (ids[i] != -1)
            {
                // changed by wz
                std::map<int, cv::Point3f>::iterator it;
                it = prev_un_pts_map_3d.find(ids[i]);
                if (it != prev_un_pts_map_3d.end())
                {
                    double v_x_3d = (cur_un_pts_3d[i].x - it->second.x) / dt;
                    double v_y_3d = (cur_un_pts_3d[i].y - it->second.y) / dt;
                    double v_z_3d = (cur_un_pts_3d[i].z - it->second.z) / dt;
                    pts_velocity_3d.push_back(cv::Point3f(v_x_3d, v_y_3d, v_z_3d));
                }
                else
                    pts_velocity_3d.push_back(cv::Point3f(0, 0, 0));
            }
            else
            {
                pts_velocity_3d.push_back(cv::Point3f(0, 0, 0));
            }
        }
    }
    else
    {
        // changed by wz
        for (unsigned int i = 0; i < cur_un_pts_3d.size(); i++)
        {
            pts_velocity_3d.push_back(cv::Point3f(0, 0, 0));
        }
    }
    // changed by wz
    prev_un_pts_map_3d = cur_un_pts_map_3d;
}