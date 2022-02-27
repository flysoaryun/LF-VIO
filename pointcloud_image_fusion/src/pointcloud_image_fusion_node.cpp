#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <eigen3/Eigen/Dense>
#include <cv_bridge/cv_bridge.h>

#include "pointcloud_image_fusion.h"

queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_equal_rectangle_img;

Pointcloud_image_fusion fusion[NUM_OF_CAM];

Eigen::Matrix3d camera_R_lidar;
Eigen::Vector3d camera_T_lidar;

pcl::PointCloud<pcl::PointXYZ> laserCloudIn;

int rows = 480;
int cols = 960;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    static bool init_flag = 0;

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        // ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    else
        // ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat show_img = ptr->image;
    cv::Mat tmp_img = show_img.clone();
    cv::Mat equal_rectangle_img;
    TicToc t;

    static cv::Mat remap_x = cv::Mat::zeros(rows, cols, CV_32FC1);
    static cv::Mat remap_y = cv::Mat::zeros(rows, cols, CV_32FC1);

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        // equal_rectangle_img=fusion[i].euqual_rectangle_expansion(tmp_img,500,1000);
        fusion[i].readImage(equal_rectangle_img, img_msg->header.stamp.toSec());

        if (init_flag == 0)
        {
            fusion[i].GetremapMat(remap_x, remap_y, rows, cols);
            init_flag = 1;
        }

        // cv::imshow("vis", equal_rectangle_img);
        // cv::waitKey(5);
    }

    //    equal_rectangle_img=fusion[0].euqual_rectangle_expansion(show_img,rows,cols);
    cv::remap(show_img, equal_rectangle_img, remap_x, remap_y, 0);
    ROS_FATAL("equal_rectangle_img costs %fms", t.toc());

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image pub_img_msg; // >> message to be sent

    std_msgs::Header header;  // empty header
    header = img_msg->header; // user defined counter

    //    cv::cvtColor(equal_rectangle_img, equal_rectangle_img, CV_GRAY2RGB);

    //    for(auto i=0;i<laserCloudIn.size();i++)
    //    {
    //       double theta=M_PI/2;

    //       camera_R_lidar<<cos(theta),-sin(theta),0.0,
    //                     sin(theta),-cos(theta),0.0,
    //                     0.0,0.0,1.0;

    //       camera_T_lidar<<0,0,-0.12;

    //        pcl::PointXYZ tmp_point_pcl=laserCloudIn[i];
    //        Eigen::Vector3d tmp_point_eigen(tmp_point_pcl.x,tmp_point_pcl.y,tmp_point_pcl.z);
    //        tmp_point_eigen=camera_R_lidar*tmp_point_eigen+camera_T_lidar;
    //        tmp_point_eigen.normalize();

    //        double lon= atan2(tmp_point_eigen.x(),tmp_point_eigen.y());
    //        double lat= asin(tmp_point_eigen.z());
    //        int idx_x=cols*(0.5+lon/(2*M_PI));
    //        int idx_y=rows*(0.5-lat/(M_PI));
    //        equal_rectangle_img.at<cv::Vec3b>(idx_y,idx_x)[0]=255;
    //        equal_rectangle_img.at<cv::Vec3b>(idx_y,idx_x)[1]=0;
    //        equal_rectangle_img.at<cv::Vec3b>(idx_y,idx_x)[2]=0;

    //    }

    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, equal_rectangle_img);
    img_bridge.toImageMsg(pub_img_msg); // from cv_bridge to sensor_msgs::Image
    pub_equal_rectangle_img.publish(pub_img_msg);
}
void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg)
{

    pcl::fromROSMsg(*pointcloud_msg, laserCloudIn);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_image_fusion");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    for (int i = 0; i < NUM_OF_CAM; i++)
        fusion[i].readIntrinsicParameter(CAM_NAMES[i]);

    if (FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            fusion[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if (!fusion[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    string POINTCLOUD_TOPIC = "/os_cloud_node/points";
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 1, img_callback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_img = n.subscribe("/feature_tracker/feature_img", 1, img_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_pointcloud = n.subscribe(POINTCLOUD_TOPIC, 1, pointcloud_callback);
    pub_equal_rectangle_img = n.advertise<sensor_msgs::Image>("equal_rectangle_img", 100);
    ros::spin();
    return 0;
}
