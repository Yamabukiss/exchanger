#include "../include/exchanger/header.h"

void Exchanger::onInit()
{
    img_subscriber_= nh_.subscribe("/hk_camera/image_raw", 1, &Exchanger::receiveFromCam,this);
    binary_publisher_ = nh_.advertise<sensor_msgs::Image>("exchanger_bianry_publisher", 1);
    segmentation_publisher_ = nh_.advertise<sensor_msgs::Image>("exchanger_segmentation_publisher", 1);
    camera_pose_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("camera_pose_publisher", 1);
    pnp_publisher_ = nh_.advertise<geometry_msgs::Pose>("pnp_publisher", 1);

    callback_ = boost::bind(&Exchanger::dynamicCallback, this, _1);
    server_.setCallback(callback_);
    camera_matrix_ = (cv::Mat_<float>(3, 3) << 1805.52667,    0.     ,  716.03783,
            0.     , 1803.61747,  532.87966,
            0.     ,    0.     ,    1.     );
    distortion_coefficients_=(cv::Mat_<float>(1,5) <<-0.074709, 0.138271, -0.001170, -0.000512, 0.000000);

    w_points1_vec_.reserve(4);
    w_points1_vec_.push_back(cv::Point3f(-0.120,0.120,0));//lb
    w_points1_vec_.push_back(cv::Point3f(-0.120,-0.120,0));//lt
    w_points1_vec_.push_back(cv::Point3f(0.1265,-0.1265,0)); //rt
    w_points1_vec_.push_back(cv::Point3f(0.120,0.120,0)); //rb

    w_points2_vec_.reserve(4);
    w_points2_vec_.push_back(cv::Point3f(0.120,0.120,0)); //rb
    w_points2_vec_.push_back(cv::Point3f(-0.120,0.120,0));//lb
    w_points2_vec_.push_back(cv::Point3f(-0.120,-0.120,0));//lt
    w_points2_vec_.push_back(cv::Point3f(0.1265,-0.1265,0)); //rt

    arrow_left_points1_vec_.reserve(4);
    arrow_left_points1_vec_.push_back(cv::Point3f(0,0.100,0));//bottom
    arrow_left_points1_vec_.push_back(cv::Point3f(0.100,0,0));//right
    arrow_left_points1_vec_.push_back(cv::Point3f(0,-0.100,0)); //top
    arrow_left_points1_vec_.push_back(cv::Point3f(0,0,0)); //mid

    arrow_left_points2_vec_.reserve(4);
    arrow_left_points2_vec_.push_back(cv::Point3f(0,-0.100,0)); //top
    arrow_left_points2_vec_.push_back(cv::Point3f(0,0.100,0));//bottom
    arrow_left_points2_vec_.push_back(cv::Point3f(0.100,0,0));//right
    arrow_left_points2_vec_.push_back(cv::Point3f(0,0,0)); //mid

    arrow_right_points1_vec_.reserve(4);
    arrow_right_points1_vec_.push_back(cv::Point3f(0,0.100,0));//bottom
    arrow_right_points1_vec_.push_back(cv::Point3f(0,-0.100,0)); //top
    arrow_right_points1_vec_.push_back(cv::Point3f(-0.100,0,0));//left
    arrow_right_points1_vec_.push_back(cv::Point3f(0,0,0)); //mid

    arrow_right_points2_vec_.reserve(4);
    arrow_right_points2_vec_.push_back(cv::Point3f(0,-0.100,0)); //top
    arrow_right_points2_vec_.push_back(cv::Point3f(-0.100,0,0));//left
    arrow_right_points2_vec_.push_back(cv::Point3f(0,0.100,0));//bottom
    arrow_right_points2_vec_.push_back(cv::Point3f(0,0,0)); //mid


    cv::Mat temp_triangle=cv::imread("/home/yamabuki/detect_ws/src/exchanger/temp_triangle.png",cv::IMREAD_GRAYSCALE);

    cv::Mat binary_1;

    cv::threshold(temp_triangle,binary_1,0, 255, CV_THRESH_BINARY + cv::THRESH_OTSU);

    std::vector<std::vector<cv::Point>> tri_temp_contour;
    std::vector<cv::Point> tri_hull;

    cv::findContours(binary_1,tri_temp_contour,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

    cv::convexHull(tri_temp_contour[0],tri_hull, true);
    temp_triangle_hull_=tri_hull;

    static tf2_ros::TransformListener tfListener(tf_buffer_);

    std::cout<<"temp init finished"<<std::endl;
}

void Exchanger::dynamicCallback(exchanger::dynamicConfig &config)
{
    morph_type_ = config.morph_type;
    morph_iterations_ = config.morph_iterations;
    morph_size_=config.morph_size;
    save_on_=config.save_on;
    triangle_moment_bias_=config.triangle_moment_bias;
    triangle_approx_epsilon_=config.triangle_approx_epsilon;
}


void Exchanger::receiveFromCam(const sensor_msgs::ImageConstPtr& image)
{
    cv_image_ = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(image, image->encoding));
    imgProcess();
    segmentation_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),cv_image_->encoding , cv_image_->image).toImageMsg());
//    getTemplateImg();
}

void Exchanger::getTemplateImg()
{
    cv::Mat gray_img;
    cv::cvtColor(cv_image_->image,gray_img,CV_BGR2GRAY);
    segmentation_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , gray_img).toImageMsg());
    if (save_on_)
    {
        cv::imwrite("/home/yamabuki/detect_ws/src/exchanger/temp_img.jpg",gray_img);
    }
}

int Exchanger::findMatchPoint(const cv::Point2f &rotate_point, const std::vector<cv::Point2i> &inline_points_vec)
{
    std::vector<std::pair<int,int>> dist_vec;
    int index = 0;
    for (auto &point : inline_points_vec)
    {
        dist_vec.push_back(std::make_pair(index,sqrt(pow((rotate_point.x-point.x),2) + pow((rotate_point.y-point.y),2))));
        index++;
    }
    std::sort(dist_vec.begin(),dist_vec.end(),[](std::pair<int,int> v1,std::pair<int,int> v2){return v1.second<v2.second;});
    return dist_vec[0].first;
}

//inline int Exchanger::calculateDistance(const cv::Point2i &p1, const cv::Point2i &p2)
//{
//    return sqrt(pow((p1.x-p2.x),2) + pow((p1.y-p2.y),2));
//}

inline bool Exchanger::checkSequence(const cv::Point2i &p1, const cv::Point2i &p2, const cv::Point2i &p3)
{
    if (p1.x>p3.x) return true;
    else return false;
}

inline bool Exchanger::checkArrowSequence(const cv::Point2i &p1, const cv::Point2i &p2, const cv::Point2i &p3)
{
    if (direction_signal_)
    {
        if (p1.y>p3.y && p1.x<p2.x) return true;
        else return false;
    }
    else
    {
        if (p1.y > p2.y && p1.y > p3.y) return true;
        else return false;
    }
}

bool Exchanger::checkArrow(std::vector<std::vector<cv::Point2i>> &hull_vec)
{
    if (hull_vec.size() == 1) return true;
    std::sort(hull_vec.begin(),hull_vec.end(),[](const auto &v1,const auto &v2){return cv::contourArea(v1)>cv::contourArea(v2);});
    if (cv::contourArea(hull_vec[0])>=3*cv::contourArea(hull_vec[1])) return true;
    else return false;
}

void Exchanger::getPnP(const cv::Mat &rvec,const cv::Mat &tvec,bool shape_signal)
{
    cv::Mat r_mat = cv::Mat_<double>(3, 3);

    cv::Rodrigues(rvec, r_mat);
    tf::Matrix3x3 tf_rotate_matrix(r_mat.at<double>(0, 0), r_mat.at<double>(0, 1), r_mat.at<double>(0, 2),
                                   r_mat.at<double>(1, 0), r_mat.at<double>(1, 1), r_mat.at<double>(1, 2),
                                   r_mat.at<double>(2, 0), r_mat.at<double>(2, 1), r_mat.at<double>(2, 2));

    tf::Quaternion quaternion;
    double r;
    double p;
    double y;

    static geometry_msgs::Pose  pose;

    pose.position.x=tvec.at<double>(0,0);
    pose.position.y=tvec.at<double>(0,1);
    pose.position.z=tvec.at<double>(0,2);

    tf_rotate_matrix.getRPY(r, p, y);
    quaternion.setRPY(r,p,y);
    pose.orientation.x=quaternion.x();
    pose.orientation.y=quaternion.y();
    pose.orientation.z=quaternion.z();
    pose.orientation.w=quaternion.w();

    pnp_publisher_.publish(pose);

    geometry_msgs::TransformStamped pose_in , pose_out;

    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(r,p,y);
    geometry_msgs::Quaternion quat_msg = tf2::toMsg(tf_quaternion);

    pose_in.transform.rotation.x = quat_msg.x;
    pose_in.transform.rotation.y = quat_msg.y;
    pose_in.transform.rotation.z = quat_msg.z;
    pose_in.transform.rotation.w = quat_msg.w;

//    tf2::doTransform(pose_in, pose_out, tf_buffer_.lookupTransform("camera_optical_frame", "base_link", ros::Time(0)));

    pose_out.transform.translation.x = pose.position.x;
    pose_out.transform.translation.y = pose.position.y;
    pose_out.transform.translation.z = pose.position.z;

    tf::Transform transform;

//    transform.setOrigin(tf::Vector3(pose_out.transform.translation.x, pose_out.transform.translation.y,
//                                    pose_out.transform.translation.z));
//    transform.setRotation(tf::Quaternion(pose_out.transform.rotation.x, pose_out.transform.rotation.y,
//                                         pose_out.transform.rotation.z, pose_out.transform.rotation.w));

    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y,
                                    pose.position.z));
    transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y,
                                         pose.orientation.z, pose.orientation.w));
    if (shape_signal) tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_optical_frame", "exchanger"));
    else  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_optical_frame", "arrow"));
}

float Exchanger::getLineLength(const cv::Point2f &p1, const cv::Point2f &p2)
{
    return sqrt(pow((p1.x-p2.x),2) + pow((p1.y-p2.y),2));
}

void Exchanger::getLongLength(int *llength_index, const std::vector<cv::Point2f> &approx_points)
{
    float length12 = getLineLength(approx_points[0],approx_points[1]);
    float length13 = getLineLength(approx_points[0],approx_points[2]);
    if (length12>length13)
    {
        llength_index[0] = 0;
        llength_index[1] = 1;
    }
    else
    {
        llength_index[0] = 0;
        llength_index[1] = 2;
    }

}

void Exchanger::imgProcess() {
    //segementation
    auto *mor_ptr = new cv::Mat();
    auto * gray_ptr= new cv::Mat();
    auto *binary_ptr = new cv::Mat();
    cv::cvtColor(cv_image_->image, *gray_ptr, cv::COLOR_BGR2GRAY);
    cv::threshold(*gray_ptr,*binary_ptr,0,255,CV_THRESH_BINARY + CV_THRESH_OTSU);
    delete gray_ptr;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1 + 2 * morph_size_, 1 + 2 * morph_size_),
                                               cv::Point(-1, -1));
    cv::morphologyEx(*binary_ptr, *mor_ptr, morph_type_, kernel, cv::Point(-1, -1), morph_iterations_);
    delete binary_ptr;
    binary_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", *mor_ptr).toImageMsg());
    // hsv contours process
    auto *contours_ptr = new std::vector<std::vector<cv::Point> >();
    cv::findContours(*mor_ptr, *contours_ptr, cv::RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    delete mor_ptr;
    std::vector<cv::Point2i> inline_points_vec;
    std::vector<std::vector<cv::Point2i>> hull_vec;
    for (auto &contours: *contours_ptr)
    {
        std::vector<cv::Point2i> hull;
        cv::convexHull(contours, hull, true);
        auto moment = cv::moments(hull);

        std::vector<cv::Point2i> approx_points;
        cv::approxPolyDP(hull, approx_points, triangle_approx_epsilon_, true);
        if (cv::matchShapes(hull, temp_triangle_hull_, cv::CONTOURS_MATCH_I2, 0) <= triangle_moment_bias_ && approx_points.size() == 3)
        {
            for (auto &point : approx_points) cv::circle(cv_image_->image,point,3,cv::Scalar(255,255,255),3);
            int cx = int(moment.m10 / moment.m00);
            int cy = int(moment.m01 / moment.m00);
            cv::Point2f centroid(cx, cy);
            cv::polylines(cv_image_->image, hull, true, cv::Scalar(0, 0, 255), 3);
            cv::circle(cv_image_->image, centroid, 2, cv::Scalar(0, 0, 255), 3);
            inline_points_vec.push_back(centroid);
            hull_vec.push_back(hull);
        }
    }
    delete contours_ptr;
    std::vector<std::vector<cv::Point2i>> tmp_hull_vec;
    tmp_hull_vec = hull_vec;
    std::sort(tmp_hull_vec.begin(),tmp_hull_vec.end(),[](const auto &v1,const auto &v2){return cv::contourArea(v1)>cv::contourArea(v2);});
    bool polygon_signal = false;
    if(!tmp_hull_vec.empty()) polygon_signal = inline_points_vec.size()==4 && cv::contourArea(tmp_hull_vec[0])<3*cv::contourArea(tmp_hull_vec[1]);
    if (polygon_signal)
    {
        cv::RotatedRect rotate_rect=cv::minAreaRect(inline_points_vec);
        cv::Point2f vertex[4];
        rotate_rect.points(vertex);
        cv::Point2i match_points[4];
        for (int i = 0; i < 4; i++)
        {
            auto match_point = inline_points_vec[findMatchPoint(vertex[i],inline_points_vec)];
            match_points[i] = match_point;
        }
        std::vector<cv::Point2f> pixel_points_vec;
        pixel_points_vec.reserve(4);
        for (int i = 0;i<4 ;i++)
        {
            pixel_points_vec.emplace_back(match_points[i]);
            cv::line(cv_image_->image, match_points[i], match_points[(i + 1) % 4], cv::Scalar(255, 100, 200),2);
            cv::putText(cv_image_->image,std::to_string(i),match_points[i],1,3,cv::Scalar(0,255,0),3);
        }

        // get pnp

        bool signal = checkSequence(match_points[0],match_points[1],match_points[2]);
        if (signal)  cv::solvePnP(w_points2_vec_,pixel_points_vec,camera_matrix_,distortion_coefficients_,exchanger_rvec_,exchanger_tvec_,bool(),cv::SOLVEPNP_ITERATIVE);
        else cv::solvePnP(w_points1_vec_,pixel_points_vec,camera_matrix_,distortion_coefficients_,exchanger_rvec_,exchanger_tvec_,bool(),cv::SOLVEPNP_ITERATIVE);
        shape_signal_ = true;
        getPnP(exchanger_rvec_,exchanger_tvec_,shape_signal_);
    }
    else if (!hull_vec.empty() && checkArrow(hull_vec))
    {
        std::vector<cv::Point2f> approx_points;
        cv::approxPolyDP(hull_vec[0], approx_points, triangle_approx_epsilon_, true);
        auto moment = cv::moments(hull_vec[0]);
        if (approx_points.size() == 3)
        {
            cv::Point2d centroid(moment.m10 / moment.m00, moment.m01 / moment.m00);
            int llength_index[2];
            getLongLength(llength_index,approx_points);
            cv::Point2f mid_edge_point = (approx_points[llength_index[0]] + approx_points[llength_index[1]])/2;

            if (mid_edge_point.x > centroid.x) direction_signal_ = false;
            else direction_signal_ = true;

            for (auto &point : approx_points) cv::circle(cv_image_->image,point,8,cv::Scalar(0,0,255),3);
            for (int i = 0;i<3 ;i++) cv::putText(cv_image_->image, std::to_string(i), approx_points[i], 1, 3, cv::Scalar(0, 255, 0), 3);
            bool signal = checkArrowSequence(approx_points[0],approx_points[1],approx_points[2]);
            if (direction_signal_)
            {
                if (signal)
                {
                    approx_points.push_back(mid_edge_point);
                    cv::solvePnP(arrow_left_points1_vec_,approx_points,camera_matrix_,distortion_coefficients_,arrow_rvec_,arrow_tvec_,bool(),cv::SOLVEPNP_P3P);
                }
                else
                {
                    approx_points.push_back(mid_edge_point);
                    cv::solvePnP(arrow_left_points2_vec_,approx_points,camera_matrix_,distortion_coefficients_,arrow_rvec_,arrow_tvec_,bool(),cv::SOLVEPNP_P3P);
                }
            }
            else
            {
                if (signal)
                {
                    approx_points.push_back(mid_edge_point);
                    cv::solvePnP(arrow_right_points1_vec_,approx_points,camera_matrix_,distortion_coefficients_,arrow_rvec_,arrow_tvec_,bool(),cv::SOLVEPNP_P3P);
                }
                else
                {
                    approx_points.push_back(mid_edge_point);
                    cv::solvePnP(arrow_right_points2_vec_,approx_points,camera_matrix_,distortion_coefficients_,arrow_rvec_,arrow_tvec_,bool(),cv::SOLVEPNP_P3P);
                }
            }
            shape_signal_ = false;
            getPnP(arrow_rvec_,arrow_tvec_,shape_signal_);
        }
        else
        {
            ROS_INFO("wrong arrow matches");
        }
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "exchanger_node");
    Exchanger Exchanger;
    Exchanger.onInit();
    while (ros::ok())
    {
        ros::spinOnce();
    }

}
