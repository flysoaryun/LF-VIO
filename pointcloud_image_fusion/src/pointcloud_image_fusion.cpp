#include "pointcloud_image_fusion.h"


int Pointcloud_image_fusion::n_id = 0;
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

//added by wz
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


Pointcloud_image_fusion::Pointcloud_image_fusion()
{
}


// cv::Mat Pointcloud_image_fusion::euqual_rectangle_expansion(cv::Mat &img,int rows,int cols)
// {
//     cv::Mat result=cv::Mat::zeros(rows,cols,img.type());
//     double lon;
//     double lat;
//     for (int i = 0; i < img.cols; i++)
//     {
//         for (int j = 0; j < img.rows; j++)
//         {
//             Eigen::Vector3d tmp_p;
//                 m_camera->liftProjective(Eigen::Vector2d(i,j), tmp_p);
//                 tmp_p.normalize();
//                 lon= atan2(tmp_p.x(),tmp_p.y());
//                 // cout<<"tmp_p"<<tmp_p<<endl;
//                 lat= asin(tmp_p.z());
//                 // cout<<"lon"<<lon<<endl;
//                 // cout<<"lat"<<lat<<endl;
//                 int idx_x=cols*(0.5+lon/(2*M_PI));
//                 int idx_y=rows*(0.5-lat/(M_PI));

//                 // cout<<"idx_y"<<idx_y<<endl;
//                 // cout<<"idx_x"<<idx_x<<endl;
//                 // cout<<"j"<<j<<endl;
//                 // cout<<"i"<<i<<endl;

//                 result.at<uchar>(idx_y,idx_x)=img.at<uchar>(j,i);

//         }
//     }
//     // cv::imshow("vis", result);
//     // cv::waitKey(1);
//     return result;

// }


void Pointcloud_image_fusion::GetremapMat(cv::Mat &remap_x,cv::Mat &remap_y,int rows,int cols)
{

    double lon;
    double lat;

    for (int i = 0; i < cols; i++)
    {
        for (int j = 0; j < rows; j++)
        {
            lon=(i/double(cols)-0.5)*(2*M_PI);
            lat=-(j/double(rows)-0.5)*M_PI;
            
            Eigen::Vector3d tmp_p;
            Eigen::Vector2d tmp_s;

            tmp_p(2)=sin(lat);
            tmp_p(0)=cos(lat)*sin(lon);
            tmp_p(1)=cos(lat)*cos(lon);
            m_camera->spaceToPlane(tmp_p,tmp_s);

            // if(tmp_s(0)>0&&tmp_s(1)>0&& tmp_s(1)<remap_x.rows&&tmp_s(0)<remap_x.cols)
            // {
                // result.at<uchar>(j,i)=img.at<uchar>(tmp_s(1),tmp_s(0));
                remap_x.at<float>(j,i)=tmp_s(0);
                remap_y.at<float>(j,i)=tmp_s(1);
            // }

        }
    }

}


cv::Mat Pointcloud_image_fusion::euqual_rectangle_expansion(cv::Mat &img,int rows,int cols)
{
    cv::Mat result=cv::Mat::zeros(rows,cols,img.type());
    double lon;
    double lat;

    for (int i = 0; i < cols; i++)
    {
        for (int j = 0; j < rows; j++)
        {
            lon=(i/double(cols)-0.5)*(2*M_PI);
            lat=-(j/double(rows)-0.5)*M_PI;
            
            Eigen::Vector3d tmp_p;
            Eigen::Vector2d tmp_s;

            tmp_p(2)=sin(lat);
            tmp_p(0)=cos(lat)*sin(lon);
            tmp_p(1)=cos(lat)*cos(lon);
            m_camera->spaceToPlane(tmp_p,tmp_s);

            if(tmp_s(0)>0&&tmp_s(1)>0&& tmp_s(1)<img.rows&&tmp_s(0)<img.cols)
            {
                result.at<uchar>(j,i)=img.at<uchar>(tmp_s(1),tmp_s(0));
            }

        }
    }



    // cv::imshow("vis", result);
    // cv::waitKey(1);
    return result;

}

void Pointcloud_image_fusion::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(0));
        //for pal 40120
        //added by wz
        cv::circle(mask,cv::Point(606,362),500,cv::Scalar(255),-1);
        cv::circle(mask,cv::Point(606,362),170,cv::Scalar(0),-1);
        //for pal 3090
        //cv::circle(mask,cv::Point(631,346),230,cv::Scalar(255),-1);
        //cv::circle(mask,cv::Point(631,346),90,cv::Scalar(0),-1);
    

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

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

void Pointcloud_image_fusion::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}
bool greaterThanPtr(const float*i, const float* j)
{
    return (*i)>(*j);
}

void Pointcloud_image_fusion::mygoodFeaturesToTrack( cv::InputArray _image, cv::OutputArray _corners,
                              int maxCorners, double qualityLevel, double minDistance,
                              cv::InputArray _mask, int blockSize,
                              bool useHarrisDetector, double harrisK )
{
    TicToc t_m;
    //???????????????_image?????????????????????_mask??????cv::Mat()??????????????????????????????
	cv::Mat image = _image.getMat(), mask = _mask.getMat();  
    ROS_INFO("2.0.0: %fms", t_m.toc());
    CV_Assert( qualityLevel > 0 && minDistance >= 0 && maxCorners >= 0 );  //??????????????????????????????
    CV_Assert( mask.empty() || (mask.type() == CV_8UC1 && mask.size() == image.size()) );
     
    cv::Mat eig, tmp;   //eig??????????????????????????????????????????????????????tmp???????????????????????????eig

    ROS_INFO("2.0.1: %fms", t_m.toc());
    if( useHarrisDetector )
        cornerHarris( image, eig, blockSize, 3, harrisK ); //blockSize?????????2*2?????????????????????????????????sobel???????????????3???harrisK?????????Harris?????????????????????
    else
        cornerMinEigenVal( image, eig, blockSize, 3 );  //????????????????????????????????????????????????????????????????????????eig???


    // ROS_INFO("test_begin: %fms", t_m.toc());
    // cv::Mat eig1;   
    // cornerMinEigenVal(image.colRange(0,480), eig1, blockSize, 3 );


    // ROS_INFO("test_end: %fms", t_m.toc());
    ROS_INFO("2.0: %fms", t_m.toc());

    double maxVal = 0;
    minMaxLoc( eig, 0, &maxVal, 0, 0, mask );   //maxVal?????????eig????????????
    threshold( eig, eig, maxVal*qualityLevel, 0, cv::THRESH_TOZERO );  //???????????????maxVal??????qualityLevel???????????????????????????????????????????????????????????????0
    
	//?????????3*3??????????????????????????????????????????????????????????????????????????????????????????????????????  
    //3*3???????????????????????????????????????????????????????????????????????????????????????  
	dilate( eig, tmp, cv::Mat());  //tmp???????????????????????????eig
 
    cv::Size imgsize = image.size(); 
 
    vector<const float*> tmpCorners;  //??????????????????????????????

    ROS_INFO("2.1: %fms", t_m.toc());
 
    // collect list of pointers to features - put them into temporary image 
    for( int y = 1; y < imgsize.height - 1; y++ )
    {
        const float* eig_data = (const float*)eig.ptr(y);  //??????eig???y???????????????
        const float* tmp_data = (const float*)tmp.ptr(y);  //??????tmp???y???????????????
        const uchar* mask_data = mask.data ? mask.ptr(y) : 0;
 
        for( int x = 1; x < imgsize.width - 1; x++ )
        {
            float val = eig_data[x];
            if( val != 0 && val == tmp_data[x] && (!mask_data || mask_data[x]) )  //val == tmp_data[x]???????????????????????????
                tmpCorners.push_back(eig_data + x);  //???????????????
        }
    }
    ROS_INFO("2.2: %fms", t_m.toc());
	//-----------?????????????????????????????????????????????????????????????????????????????????----------//
	//-----------??????????????????????????????minDistance????????????????????????????????????????????????????????????????????????----------//
 
    std::sort( tmpCorners.begin(),tmpCorners.end(), greaterThanPtr );  //??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
    vector<cv::Point2f> corners;
    size_t i, j, total = tmpCorners.size(), ncorners = 0;
    ROS_INFO("2.3: %fms", t_m.toc());
    //???????????????????????????????????????????????????????????????
	if(minDistance >= 1)  
    {
         // Partition the image into larger grids
        int w = image.cols;
        int h = image.rows;
 
        const int cell_size = cvRound(minDistance);   //????????????????????????
 
	//????????????cell_size???????????????????????????grid(???????????????grid????????????vector<vector>?????????????????????????????????????????????????????????????????????,?????????grid???????????????grid_width * grid_height????????????????????????)?????????cell_size??????grid????????????????????????????????????_image?????????minDistance?????????????????????????????????cell_size - 1????????????
        const int grid_width = (w + cell_size - 1) / cell_size; 
        const int grid_height = (h + cell_size - 1) / cell_size;
 
        std::vector<std::vector<cv::Point2f> > grid(grid_width*grid_height);  //vector?????????vector???grid????????????????????????????????????
 
        minDistance *= minDistance;  //?????????????????????????????????????????????
 
        for( i = 0; i < total; i++ )     // ?????????????????????????????????????????????????????????????????????
        {
            int ofs = (int)((const uchar*)tmpCorners[i] - eig.data);  //tmpCorners??????????????????????????????eig.data??????eig?????????????????????
            int y = (int)(ofs / eig.step);   //???????????????????????????
            int x = (int)((ofs - y*eig.step)/sizeof(float));  //?????????????????????
 
            bool good = true;  //????????????????????????????????????????????????????????????
 
            int x_cell = x / cell_size;  //x_cell???y_cell????????????y,x??????grid??????????????????
            int y_cell = y / cell_size;
 
            int x1 = x_cell - 1;  // (y_cell???x_cell??????4????????????
            int y1 = y_cell - 1;  //???????????????????????????grid_width??????????????????cell_size - 1??????????????????????????????y,x??????grid??????4????????????????????????????????????(y_cell???x_cell???????????????????????????
            int x2 = x_cell + 1;  
            int y2 = y_cell + 1;
 
            // boundary check???????????????x1,y1,x2???y2????????????grid??????
            x1 = std::max(0, x1);  //??????0???x1?????????
            y1 = std::max(0, y1);
            x2 = std::min(grid_width-1, x2);
            y2 = std::min(grid_height-1, y2);
 
            //??????grid?????????????????????????????????_image????????????minDistance?????????
			for( int yy = y1; yy <= y2; yy++ )  // ???
            {
                for( int xx = x1; xx <= x2; xx++ )  //???
                {
                    vector <cv::Point2f> &m = grid[yy*grid_width + xx];  //??????
 
                    if( m.size() )  //??????(y_cell???x_cell)???4????????????????????????(y,x)???minDistance??????????????????????????????????????????
                    {               
                        for(j = 0; j < m.size(); j++)   //??????????????????????????????????????????????????????????????????
                        {
                            float dx = x - m[j].x;
                            float dy = y - m[j].y;
			   //????????????(y,x)???minDistance????????????????????????????????????????????????????????????????????????(y,x)????????????????????????????????????tmpCorners???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
                            if( dx*dx + dy*dy < minDistance )
                            {                              							
				good = false;
                                goto break_out;
                            }
                        }
                    }
                }   // ???
            }    //???
 
            break_out:
 
            if(good)
            {
                // printf("%d: %d %d -> %d %d, %d, %d -- %d %d %d %d, %d %d, c=%d\n",
                //    i,x, y, x_cell, y_cell, (int)minDistance, cell_size,x1,y1,x2,y2, grid_width,grid_height,c);
                grid[y_cell*grid_width + x_cell].push_back(cv::Point2f((float)x, (float)y));
 
                corners.push_back(cv::Point2f((float)x, (float)y));
                ++ncorners;
 
                if( maxCorners > 0 && (int)ncorners == maxCorners )  //????????????????????????????????????ncorners??????maxCorners?????????????????????????????????tmpCorners??????????????????????????????????????????????????????
                    break;
            }
        }
        ROS_INFO("2.4: %fms", t_m.toc());
    }
    else    //?????????????????????????????????????????????????????????????????????minDistance < 1,?????????????????????????????????
    {
        for( i = 0; i < total; i++ )
        {
            int ofs = (int)((const uchar*)tmpCorners[i] - eig.data);
            int y = (int)(ofs / eig.step);   //????????????????????????????????????
            int x = (int)((ofs - y*eig.step)/sizeof(float));  //??????????????????
 
            corners.push_back(cv::Point2f((float)x, (float)y));
            ++ncorners;
            if( maxCorners > 0 && (int)ncorners == maxCorners )  
                break;
        }
    }
 
    cv::Mat(corners).convertTo(_corners, _corners.fixedType() ? _corners.type() : CV_32F);
 
    /*
    for( i = 0; i < total; i++ )
    {
        int ofs = (int)((const uchar*)tmpCorners[i] - eig.data);
        int y = (int)(ofs / eig.step);
        int x = (int)((ofs - y*eig.step)/sizeof(float));
        if( minDistance > 0 )
        {
            for( j = 0; j < ncorners; j++ )
            {
                float dx = x - corners[j].x;
                float dy = y - corners[j].y;
                if( dx*dx + dy*dy < minDistance )
                    break;
            }
            if( j < ncorners )
                continue;
        }
        corners.push_back(Point2f((float)x, (float)y));
        ++ncorners;
        if( maxCorners > 0 && (int)ncorners == maxCorners )
            break;
    }
*/
}


void Pointcloud_image_fusion::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;
    cv::Mat img2(_img.rows,_img.cols,CV_8UC1,cv::Scalar(0));
    // cv::Mat cmask=cv::Mat::zeros(720,1280,CV_8UC1);
    // cv::circle(cmask,cv::Point(631,346),230,cv::Scalar(255),-1);
    // cv::circle(cmask,cv::Point(631,346),90,cv::Scalar(0),-1);
    
    if (0)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(5.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        //img.copyTo(img2,cmask);
        //img=img2;
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;
    


        //img.copyTo(img2,cmask);
        //img=img2;

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
        // cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
        // cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        // for (int i = 0; i < int(forw_pts.size()); i++)
        //     if (status[i] && !inBorder(forw_pts[i]))
        //         status[i] = 0;
        // reduceVector(prev_pts, status);
        // reduceVector(cur_pts, status);
        // reduceVector(forw_pts, status);
        // reduceVector(ids, status);
        // reduceVector(cur_un_pts, status);

        // //added by wz
        // reduceVector(cur_un_pts_3d, status);

        // reduceVector(track_cnt, status);
        // ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    // for (auto &n : track_cnt)
    //     n++;

    // if (PUB_THIS_FRAME)
    // {
    //     // rejectWithF();
    //     // ROS_DEBUG("set mask begins");
    //     TicToc t_m;
    //     // setMask();
    //     // ROS_DEBUG("set mask costs %fms", t_m.toc());

    //     // ROS_DEBUG("detect feature begins");
    //     TicToc t_t;
    //     int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
    //     if (n_max_cnt > 0)
    //     {
    //         if(mask.empty())
    //             cout << "mask is empty " << endl;
    //         if (mask.type() != CV_8UC1)
    //             cout << "mask type wrong " << endl;
    //         if (mask.size() != forw_img.size())
    //             cout << "wrong size " << endl;
    //         mygoodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
    //     }
    //     else
    //         n_pts.clear();
    //     ROS_DEBUG("detect feature costs: %fms", t_t.toc());

    //     ROS_DEBUG("add feature begins");
    //     TicToc t_a;
    //     addPoints();
    //     ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    // }
    // prev_img = cur_img;
    // prev_pts = cur_pts;
    // prev_un_pts = cur_un_pts;
    // cur_img = forw_img;
    // cur_pts = forw_pts;
    // //undistortedPoints();
    // prev_time = cur_time;
}

void Pointcloud_image_fusion::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);


        //added by wz
        reduceVector(cur_un_pts_3d, status);

        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

bool Pointcloud_image_fusion::updateID(unsigned int i)
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

void Pointcloud_image_fusion::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void Pointcloud_image_fusion::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            //added by wz
            if(b.z()>0)
            {
                distortedp.push_back(a);
                undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            }
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    // cv::imshow(name, undistortedImg);
    // cv::waitKey(1);
}

void Pointcloud_image_fusion::undistortedPoints()
{
    cur_un_pts.clear();

    //added by wz
    cur_un_pts_3d.clear();


    cur_un_pts_map.clear();

    cur_un_pts_map_3d.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);

        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_3d.push_back(cv::Point3f(b.x()/b.norm(), b.y() /b.norm(),b.z()/b.norm()));

        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        cur_un_pts_map_3d.insert(make_pair(ids[i], cv::Point3f(b.x()/b.norm(), b.y() /b.norm(),b.z()/b.norm())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    //changed by wz
    // if (!prev_un_pts_map.empty())
    if (!prev_un_pts_map_3d.empty())
    {
        double dt = cur_time - prev_time;
        //changed by wz
        // pts_velocity.clear();
        //added by wz
        pts_velocity_3d.clear();
        // for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        for (unsigned int i = 0; i < cur_un_pts_3d.size(); i++)
        {
            if (ids[i] != -1)
            {
                //changed by wz
                // std::map<int, cv::Point2f>::iterator it;
                std::map<int, cv::Point3f>::iterator it;
                // it = prev_un_pts_map.find(ids[i]);
                it = prev_un_pts_map_3d.find(ids[i]);
                // if (it != prev_un_pts_map.end())
                if (it != prev_un_pts_map_3d.end())
                {

                    // double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    // double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    // pts_velocity.push_back(cv::Point2f(v_x, v_y));
                    double v_x_3d = (cur_un_pts_3d[i].x - it->second.x) / dt;
                    double v_y_3d = (cur_un_pts_3d[i].y - it->second.y) / dt;
                    double v_z_3d = (cur_un_pts_3d[i].z - it->second.z) / dt;
                    pts_velocity_3d.push_back(cv::Point3f(v_x_3d, v_y_3d, v_z_3d));


                }
                else
                    // pts_velocity.push_back(cv::Point2f(0, 0));
                    pts_velocity_3d.push_back(cv::Point3f(0, 0, 0));
            }
            else
            {
                // pts_velocity.push_back(cv::Point2f(0, 0));
                pts_velocity_3d.push_back(cv::Point3f(0, 0, 0));
            }
        }
    }
    else
    {
        // for (unsigned int i = 0; i < cur_pts.size(); i++)
        // {
        //     pts_velocity.push_back(cv::Point2f(0, 0));
        // }
        //changed by wz
        for (unsigned int i = 0; i < cur_un_pts_3d.size(); i++)
        {
            pts_velocity_3d.push_back(cv::Point3f(0, 0, 0));
        }
    }
    //changed by wz
    // prev_un_pts_map = cur_un_pts_map;
    prev_un_pts_map_3d = cur_un_pts_map_3d;
    
}
