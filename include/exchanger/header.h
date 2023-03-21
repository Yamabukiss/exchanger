#pragma once

#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <algorithm>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <dynamic_reconfigure/server.h>
#include <exchanger/dynamicConfig.h>
#include<iostream>
#include<fstream>
#include <thread>
#include <mutex>
#include <future>
#include <chrono>
#include <ctime>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class Exchanger
{
public:
    void onInit();
    void receiveFromCam(const sensor_msgs::ImageConstPtr &image);
    void dynamicCallback(exchanger::dynamicConfig& config);
    void imgProcess();
    void getTemplateImg();
    int findMatchPoint(const cv::Point2f &rotate_point ,  const std::vector<cv::Point2i> &inline_points_vec);
    bool checkSequence(const cv::Point2i &p1,const cv::Point2i &p2,const cv::Point2i &p3);
    bool checkArrowSequence(const cv::Point2i &p1,const cv::Point2i &p2,const cv::Point2i &p3);
    void getPnP(const cv::Mat &rvec,const cv::Mat &tvec,bool shape_signal);
    bool checkArrow(std::vector<std::vector<cv::Point2i>> &hull_vec);
    void getLongLength(int * llength_index,const std::vector<cv::Point2f> &approx_points);
    float getLineLength(const cv::Point2f & p1,const cv::Point2f & p2);
    ros::NodeHandle nh_;
    cv_bridge::CvImagePtr cv_image_;
    ros::Subscriber img_subscriber_;
    ros::Publisher binary_publisher_;
    ros::Publisher segmentation_publisher_;
    ros::Publisher camera_pose_publisher_;
    dynamic_reconfigure::Server<exchanger::dynamicConfig> server_;
    dynamic_reconfigure::Server<exchanger::dynamicConfig>::CallbackType callback_;

    int morph_type_;
    int morph_iterations_;
    int morph_size_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coefficients_;
    std::vector<cv::Point> temp_triangle_hull_;
    bool save_on_;
    bool shape_signal_;
    bool direction_signal_;
    double triangle_moment_bias_;
    int triangle_approx_epsilon_;
    std::vector<cv::Point3f> w_points1_vec_;
    std::vector<cv::Point3f> w_points2_vec_;
    std::vector<cv::Point3f> arrow_left_points1_vec_;
    std::vector<cv::Point3f> arrow_left_points2_vec_;
    std::vector<cv::Point3f> arrow_right_points1_vec_;
    std::vector<cv::Point3f> arrow_right_points2_vec_;
    cv::Mat exchanger_rvec_;
    cv::Mat exchanger_tvec_;
    cv::Mat arrow_rvec_;
    cv::Mat arrow_tvec_;
    ros::Publisher pnp_publisher_;
    ros::Publisher flag_publisher_;
    tf2_ros::Buffer tf_buffer_;
    tf::TransformBroadcaster tf_broadcaster_;
};