#include "../include/exchanger/header.h"

void Exchanger::onInit()
{
    img_subscriber_= nh_.subscribe("/hk_camera/image_raw", 1, &Exchanger::receiveFromCam,this);
    tf_updated_subscriber_ = nh_.subscribe("/is_update_exchanger", 1, &Exchanger::receiveFromEng, this);
    binary_publisher_ = nh_.advertise<sensor_msgs::Image>("exchanger_binary_publisher", 1);
    segmentation_publisher_ = nh_.advertise<sensor_msgs::Image>("exchanger_segmentation_publisher", 1);
    camera_pose_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("camera_pose_publisher", 1);
    pnp_publisher_ = nh_.advertise<rm_msgs::ExchangerMsg>("pnp_publisher", 1);
    w_points2_vec_.reserve(4);
    w_points1_vec_.reserve(4);
    callback_ = boost::bind(&Exchanger::dynamicCallback, this, _1);
    server_.setCallback(callback_);

    camera_matrix_ = (cv::Mat_<float>(3, 3) << 1811.208049,    0.     ,  692.262792,
            0.     , 1811.768707,  576.194205,
            0.     ,    0.     ,    1.     );
    distortion_coefficients_=(cv::Mat_<float>(1,5) << -0.079091 ,0.108809 ,-0.000094, -0.000368, 0.000000);
    w_points1_vec_.push_back(cv::Point3f(-0.120,0.120,0));//lb
    w_points1_vec_.push_back(cv::Point3f(-0.120,-0.120,0));//lt
    w_points1_vec_.push_back(cv::Point3f(0.120 + small_offset_,-0.120 - small_offset_,0)); //rt
    w_points1_vec_.push_back(cv::Point3f(0.120,0.120,0)); //rb
    w_points2_vec_.push_back(cv::Point3f(0.120,0.120,0)); //rb
    w_points2_vec_.push_back(cv::Point3f(-0.120,0.120,0));//lb
    w_points2_vec_.push_back(cv::Point3f(-0.120,-0.120,0));//lt
    w_points2_vec_.push_back(cv::Point3f(0.120 + small_offset_,-0.120 - small_offset_,0)); //rt

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


    cv::Mat temp_triangle=cv::imread(ros::package::getPath("exchanger")+"/temp_triangle.png",cv::IMREAD_GRAYSCALE);

    cv::Mat binary_1;

    cv::threshold(temp_triangle,binary_1,0, 255, CV_THRESH_BINARY_INV + cv::THRESH_OTSU);

    std::vector<std::vector<cv::Point>> tri_temp_contour;
    std::vector<cv::Point> tri_hull;
    cv::findContours(binary_1,tri_temp_contour,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    std::sort(tri_temp_contour.begin(), tri_temp_contour.end(), [](const auto &v1, const auto &v2){ return cv::contourArea(v1) > cv::contourArea(v2);});

    cv::convexHull(tri_temp_contour[0],tri_hull, true);
    temp_triangle_hull_=tri_hull;
    prev_msg_.pose.orientation.w = 1;
    static tf2_ros::TransformListener tfListener(tf_buffer_);
    tf_update_ = true;
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
    arrow_area_threshold_ = config.arrow_area_threshold;

    red_lower_hsv_h_=config.red_lower_hsv_h;
    red_lower_hsv_s_=config.red_lower_hsv_s;
    red_lower_hsv_v_=config.red_lower_hsv_v;
    red_upper_hsv_h_=config.red_upper_hsv_h;
    red_upper_hsv_s_=config.red_upper_hsv_s;
    red_upper_hsv_v_=config.red_upper_hsv_v;

    blue_lower_hsv_h_=config.blue_lower_hsv_h;
    blue_lower_hsv_s_=config.blue_lower_hsv_s;
    blue_lower_hsv_v_=config.blue_lower_hsv_v;
    blue_upper_hsv_h_=config.blue_upper_hsv_h;
    blue_upper_hsv_s_=config.blue_upper_hsv_s;
    blue_upper_hsv_v_=config.blue_upper_hsv_v;
    min_triangle_threshold_=config.min_triangle_threshold;
    max_variance_threshold_=config.max_variance_threshold;
    red_=config.red;
    small_offset_ = config.small_offset;
    w_points1_vec_[2] = (cv::Point3f(0.120 + small_offset_,-0.120 - small_offset_,0.)); //rt
    w_points2_vec_[3] = (cv::Point3f(0.120 + small_offset_,-0.120 - small_offset_,0.)); //rt
}

double square(double in)
{
    double out = pow(in,2);
    return out;
}

void quatToRPY(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
{
    double as = std::min(-2. * (q.x * q.z - q.w * q.y), .99999);
    yaw = std::atan2(2 * (q.x * q.y + q.w * q.z), square(q.w) + square(q.x) - square(q.y) - square(q.z));
    pitch = std::asin(as);
    roll = std::atan2(2 * (q.y * q.z + q.w * q.x), square(q.w) - square(q.x) - square(q.y) + square(q.z));
}

void Exchanger::receiveFromEng(const std_msgs::BoolConstPtr &signal)
{
    bool is_update = signal->data;
    tf_update_ = is_update;
}

void Exchanger::receiveFromCam(const sensor_msgs::ImageConstPtr& msg)
{
    cv_image_ = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    imgProcess();
    segmentation_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),cv_image_->encoding , cv_image_->image).toImageMsg());
    ros::Duration(0.1).sleep();
}

void Exchanger::getTemplateImg()
{
    cv::Mat gray_img;
    cv::cvtColor(cv_image_->image,gray_img,CV_BGR2GRAY);
    segmentation_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , gray_img).toImageMsg());
    if (save_on_)
    {
        cv::imwrite(ros::package::getPath("exchanger")+"/temp_img.jpg",gray_img);
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

inline bool Exchanger::checkSequence(const cv::Point2i &p1, const cv::Point2i &p2, const cv::Point2i &p3)
{
    return (p1.x > p3.x);
}

inline bool Exchanger::checkArrowSequence(const cv::Point2i &p1, const cv::Point2i &p2, const cv::Point2i &p3)
{
    if (direction_signal_)
    {
        return (p1.y > p3.y && p1.x < p2.x);
    }
    else
    {
        return (p1.y > p2.y && p1.y > p3.y);
    }
}

bool Exchanger::checkArrow(std::vector<std::vector<cv::Point2i>> &hull_vec)
{
    if (hull_vec.size() == 1) return true;
    std::sort(hull_vec.begin(),hull_vec.end(),[](const auto &v1,const auto &v2){return cv::contourArea(v1)>cv::contourArea(v2);});
    return cv::contourArea(hull_vec[0]) >= 3 * cv::contourArea(hull_vec[1]);
}

void Exchanger::getPnP(const cv::Mat &rvec,const cv::Mat &tvec)
{
    cv::Mat r_mat = cv::Mat_<double>(3, 3);

    if (rvec.empty())
    {
        ROS_INFO("opencv mat bug,return and pose nonsense pnp");
        poseNonSensePnP();
        return;
    }

    // the projection for the center of exchanger
    cv::Point3f point_o(0, 0, 0);
    cv::Point2f projected_point;
    std::vector<cv::Point3f> w_points_vector;
    std::vector<cv::Point2f> p_points_vector;
    w_points_vector.reserve(1);
    p_points_vector.reserve(1);
    w_points_vector.emplace_back(point_o);
    p_points_vector.emplace_back(projected_point);

    cv::projectPoints(w_points_vector, rvec, tvec, camera_matrix_, distortion_coefficients_, p_points_vector);
    cv::circle(cv_image_->image, p_points_vector[0], 2, cv::Scalar(255, 255, 255), 3);
    
    cv::Rodrigues(rvec, r_mat);
    tf::Matrix3x3 tf_rotate_matrix(r_mat.at<double>(0, 0), r_mat.at<double>(0, 1), r_mat.at<double>(0, 2),
                                   r_mat.at<double>(1, 0), r_mat.at<double>(1, 1), r_mat.at<double>(1, 2),
                                   r_mat.at<double>(2, 0), r_mat.at<double>(2, 1), r_mat.at<double>(2, 2));

    tf::Quaternion quaternion;
    double r;
    double p;
    double y;

    tf_rotate_matrix.getRPY(r, p, y);
    quaternion.setRPY(r,p,y);

    // upon for the origin pose and translation
    geometry_msgs::TransformStamped pose_in, pose_out;

    tf2::Quaternion tf_quaternion;
    //here for the tfListener
    tf_quaternion.setRPY(r,p,y);

    geometry_msgs::Quaternion quat_msg = tf2::toMsg(tf_quaternion); // tmp for the pose
    pose_in.transform.translation.x = tvec.at<double>(0,0);
    pose_in.transform.translation.y = tvec.at<double>(0,1);
    pose_in.transform.translation.z = tvec.at<double>(0,2);

    pose_in.transform.rotation.x = quat_msg.x;
    pose_in.transform.rotation.y = quat_msg.y;
    pose_in.transform.rotation.z = quat_msg.z;
    pose_in.transform.rotation.w = quat_msg.w;
    try
    {
        tf2::doTransform(pose_in, pose_out, tf_buffer_.lookupTransform("map", "camera_optical_frame", ros::Time(0)));

    }
    catch (tf2::TransformException& ex)
    {
        ROS_INFO_STREAM("tf error from ros");
        return;
    }

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose_out.transform.translation.x, pose_out.transform.translation.y,
                                    pose_out.transform.translation.z));
    transform.setRotation(tf::Quaternion(pose_out.transform.rotation.x, pose_out.transform.rotation.y,
                                         pose_out.transform.rotation.z, pose_out.transform.rotation.w));

    double roll_temp, pitch_temp, yaw_temp;
    quatToRPY(pose_out.transform.rotation, roll_temp, pitch_temp, yaw_temp);
    roll_temp +=CV_PI/2;
    yaw_temp +=CV_PI/2;

    tf2::Quaternion tmp_tf_quaternion;
    tmp_tf_quaternion.setRPY(pitch_temp,-roll_temp,yaw_temp);
    geometry_msgs::Quaternion tmp_quat_msg = tf2::toMsg(tmp_tf_quaternion); // tmp for the pose
    transform.setRotation(tf::Quaternion(tmp_quat_msg.x, tmp_quat_msg.y,
                                         tmp_quat_msg.z, tmp_quat_msg.w));

    rm_msgs::ExchangerMsg msg;
    msg.flag = 1;
    if (shape_signal_) msg.shape = 1;
    else msg.shape = 0;

    msg.pose.position.x=pose_in.transform.translation.x;
    msg.pose.position.y=pose_in.transform.translation.y;
    msg.pose.position.z=pose_in.transform.translation.z;

    msg.pose.orientation.x=tmp_quat_msg.x;
    msg.pose.orientation.y=tmp_quat_msg.y;
    msg.pose.orientation.z=tmp_quat_msg.z;
    msg.pose.orientation.w=tmp_quat_msg.w;

    pnp_publisher_.publish(msg);
    prev_msg_ = msg;
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "exchanger"));
}

inline double Exchanger::getLineLength(const cv::Point2f &p1, const cv::Point2f &p2)
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

inline void Exchanger::poseNonSensePnP()
{
    rm_msgs::ExchangerMsg msg;
    msg = prev_msg_;
    msg.flag = 0;
    tf::Transform transform;

    transform.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y,
                                    msg.pose.position.z));
    transform.setRotation(tf::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y,
                                         msg.pose.orientation.z, msg.pose.orientation.w));
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "exchanger"));

    pnp_publisher_.publish(msg);
}

void Exchanger::combinationSolver(const std::vector<cv::Point2i> &inline_points_vec, int start, int k, std::vector<cv::Point2i> &combination_vec,
                                  std::vector<std::vector<cv::Point2i>> &combination_save_vec)
{
    if (k == 0)
    {
        combination_save_vec.emplace_back(combination_vec);
        return;
    }
    for (int i = start; i<= inline_points_vec.size()- k; i++)
    {
        combination_vec.emplace_back(inline_points_vec[i]);
        combinationSolver(inline_points_vec, i+1, k-1, combination_vec,combination_save_vec);
        combination_vec.pop_back();
    }
}

bool Exchanger::findRectPoints(std::vector<cv::Point2i> &rect_points_vec, const std::vector<cv::Point2i> &inline_points_vec, std::vector<cv::Point2i> &combination_result_vec)
{
    if (inline_points_vec.size() < 5 || inline_points_vec.size() > 7)
        return false;
    std::vector<std::pair<std::vector<cv::Point2i>,double>> point_variance_vec;
    std::vector<cv::Point2i> combination_vec;
    std::vector<std::vector<cv::Point2i>> combination_save_vec;
    combinationSolver(inline_points_vec, 0, 4, combination_vec, combination_save_vec);
    for (const auto &points_vec : combination_save_vec)
    {
        cv::Point2i middle_point = (points_vec[0] + points_vec[1] + points_vec[2] +points_vec[3]) / 4;
        double distance1 = getLineLength(points_vec[0], middle_point);
        double distance2 = getLineLength(points_vec[1], middle_point);
        double distance3 = getLineLength(points_vec[2], middle_point);
        double distance4 = getLineLength(points_vec[3], middle_point);
        double max_distance = std::max(std::max(distance1,distance2),std::max(distance3,distance4));
        double variance = (pow(distance1 - max_distance, 2) + pow(distance2 - max_distance, 2) + pow(distance3 - max_distance, 2) + pow(distance4 - max_distance, 2)) / 4;
        point_variance_vec.emplace_back(points_vec, variance);
    }
    std::sort(point_variance_vec.begin(), point_variance_vec.end(), [](const auto &v1, const auto &v2){return v1.second < v2.second;});
    cv::Point2i middle_point = (point_variance_vec[0].first[0] + point_variance_vec[0].first[1] + point_variance_vec[0].first[2] +point_variance_vec[0].first[3]) / 4;
    cv::putText(cv_image_->image, std::to_string(point_variance_vec[0].second), middle_point, 1,1,cv::Scalar(255,255,255),2);
    combination_result_vec = point_variance_vec[0].first;
    if (point_variance_vec[0].second < max_variance_threshold_)
        return true;
    else
        return false;
}

void Exchanger::imgProcess() {
    //segementation
    auto *mor_ptr = new cv::Mat();
    auto *hsv_ptr= new cv::Mat();
    auto *binary_ptr = new cv::Mat();
    cv::cvtColor(cv_image_->image, *hsv_ptr, cv::COLOR_BGR2HSV);
    if (red_)
        cv::inRange(*hsv_ptr,cv::Scalar(red_lower_hsv_h_,red_lower_hsv_s_,red_lower_hsv_v_),cv::Scalar(red_upper_hsv_h_,red_upper_hsv_s_,red_upper_hsv_v_),*binary_ptr);
    else
        cv::inRange(*hsv_ptr,cv::Scalar(blue_lower_hsv_h_,blue_lower_hsv_s_,blue_lower_hsv_v_),cv::Scalar(blue_upper_hsv_h_,blue_upper_hsv_s_,blue_upper_hsv_v_),*binary_ptr);
    delete hsv_ptr;
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
    std::vector<cv::Point2i> rect_points_vec;
    std::vector<std::vector<cv::Point2i>> hull_vec;
    for (auto &contours: *contours_ptr)
    {
        std::vector<cv::Point2i> hull;
        cv::convexHull(contours, hull, true);
        auto moment = cv::moments(hull);

        std::vector<cv::Point2i> approx_points;
        cv::approxPolyDP(hull, approx_points, triangle_approx_epsilon_, true);
        if (cv::matchShapes(hull, temp_triangle_hull_, cv::CONTOURS_MATCH_I2, 0) <= triangle_moment_bias_ && cv::contourArea(hull) >= min_triangle_threshold_ && approx_points.size() > 1)
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

    std::vector<cv::Point2i> combination_result_vec;
    bool rect_signal = findRectPoints(rect_points_vec, inline_points_vec, combination_result_vec);

    if (polygon_signal && tf_update_)
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
            cv::line(cv_image_->image, match_points[i], match_points[(i + 1) % 4], cv::Scalar(255, 100, 200), 2, cv::LINE_AA);
            cv::putText(cv_image_->image,std::to_string(i),match_points[i],1,3,cv::Scalar(0,255,0),3);
        }
        // get pnp
        bool signal = checkSequence(match_points[0],match_points[1],match_points[2]);
        if (signal)  cv::solvePnP(w_points2_vec_,pixel_points_vec,camera_matrix_,distortion_coefficients_,exchanger_rvec_,exchanger_tvec_,bool(),cv::SOLVEPNP_ITERATIVE);
        else cv::solvePnP(w_points1_vec_,pixel_points_vec,camera_matrix_,distortion_coefficients_,exchanger_rvec_,exchanger_tvec_,bool(),cv::SOLVEPNP_ITERATIVE);
        shape_signal_ = true;
        getPnP(exchanger_rvec_,exchanger_tvec_);
    }
    else if (!hull_vec.empty() && checkArrow(hull_vec) && cv::contourArea(hull_vec[0]) > arrow_area_threshold_ && tf_update_)
    {
        std::vector<cv::Point2f> approx_points;
        cv::approxPolyDP(hull_vec[0], approx_points, triangle_approx_epsilon_, true);
        auto moment = cv::moments(hull_vec[0]);
        if (approx_points.size() == 3 && tf_update_)
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
                    cv::solvePnP(arrow_left_points1_vec_,approx_points,camera_matrix_,distortion_coefficients_,arrow_left_rvec_,arrow_left_tvec_,bool(),cv::SOLVEPNP_P3P);
                }
                else
                {
                    approx_points.push_back(mid_edge_point);
                    cv::solvePnP(arrow_left_points2_vec_,approx_points,camera_matrix_,distortion_coefficients_,arrow_left_rvec_,arrow_left_tvec_,bool(),cv::SOLVEPNP_P3P);
                }
            }
            else
            {
                if (signal)
                {
                    approx_points.push_back(mid_edge_point);
                    cv::solvePnP(arrow_right_points1_vec_,approx_points,camera_matrix_,distortion_coefficients_,arrow_left_rvec_,arrow_left_tvec_,bool(),cv::SOLVEPNP_P3P);
                }
                else
                {
                    approx_points.push_back(mid_edge_point);
                    cv::solvePnP(arrow_right_points2_vec_,approx_points,camera_matrix_,distortion_coefficients_,arrow_left_rvec_,arrow_left_tvec_,bool(),cv::SOLVEPNP_P3P);
                }
            }
            shape_signal_ = false;
            getPnP(arrow_left_rvec_,arrow_left_tvec_);
        }
        else
        {
            ROS_INFO("wrong arrow matches");
            poseNonSensePnP();
        }
    }
    else if (rect_signal && tf_update_)
    {
        cv::RotatedRect rotate_rect=cv::minAreaRect(combination_result_vec);
        cv::Point2f vertex[4];
        rotate_rect.points(vertex);
        cv::Point2i match_points[4];
        for (int i = 0; i < 4; i++)
        {
            auto match_point = combination_result_vec[findMatchPoint(vertex[i],combination_result_vec)];
            match_points[i] = match_point;
        }
        std::vector<cv::Point2f> pixel_points_vec;
        pixel_points_vec.reserve(4);
        for (int i = 0;i<4 ;i++)
        {
            pixel_points_vec.emplace_back(match_points[i]);
            cv::line(cv_image_->image, match_points[i], match_points[(i + 1) % 4], cv::Scalar(255, 255, 0),2, cv::LINE_AA);
            cv::putText(cv_image_->image,std::to_string(i),match_points[i],1,3,cv::Scalar(0,255,255),3);
        }
        // get pnp
        bool signal = checkSequence(match_points[0],match_points[1],match_points[2]);
        if (signal)  cv::solvePnP(w_points2_vec_,pixel_points_vec,camera_matrix_,distortion_coefficients_,exchanger_rvec_,exchanger_tvec_,bool(),cv::SOLVEPNP_ITERATIVE);
        else cv::solvePnP(w_points1_vec_,pixel_points_vec,camera_matrix_,distortion_coefficients_,exchanger_rvec_,exchanger_tvec_,bool(),cv::SOLVEPNP_ITERATIVE);
        shape_signal_ = true;
        getPnP(exchanger_rvec_,exchanger_tvec_);
    }
    else
    {
        poseNonSensePnP();
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
