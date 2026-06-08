#include "ndt_localization.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/filters/crop_box.h>
#include <thread>
NDTLocalization::NDTLocalization()
: Node("ndt_localization")
{
    // --- 状态 ---
    initial_pose_received_ = false;
    map_loaded_ = false;
    odom_initialized_ = false;
    received_points_ = false;
    state_ = UNINITIALIZED;

    // --- TF ---
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // --- 点云 ---
    map_cloud_.reset(new pcl::PointCloud<PointType>);

    while (rclcpp::ok()) {
        try {
            tf_lidar_base = tf_buffer_->lookupTransform(
                "livox_frame",     
                "base_footprint",   
                rclcpp::Time(0),    
                rclcpp::Duration::from_seconds(0.5) 
            );
            break;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            rclcpp::sleep_for(std::chrono::milliseconds(100)); 
        }
    }
    lidar_base_ = tf2::transformToEigen(tf_lidar_base).matrix().cast<float>();

    // --- NDT 参数优化 ---
    ndt_.setTransformationEpsilon(0.05);
    ndt_.setStepSize(0.05);
    ndt_.setResolution(2.0);
    ndt_.setMaximumIterations(20);

    // --- Voxel 滤波 ---
    voxel_filter_.setLeafSize(0.4, 0.4, 0.4);

    // --- 订阅话题 ---
    pcd_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/aligned_cloud", 1,
        std::bind(&NDTLocalization::mapCallback, this, std::placeholders::_1));

    points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/filtered_cloud",
        // "/livox/lidar", 
        10,
        std::bind(&NDTLocalization::pointsCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 50,
        std::bind(&NDTLocalization::odomCallback, this, std::placeholders::_1));

    initialpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/ndtinitialpose", 1,
        std::bind(&NDTLocalization::initialPoseCallback, this, std::placeholders::_1));
    amcl_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10);

    // --- 初始化 pose ---
    current_pose_ = Eigen::Matrix4f::Identity();
    predicted_pose_ = Eigen::Matrix4f::Identity();
    last_odom_pose_ = Eigen::Matrix4f::Identity();

    // --- 定时发布 TF ---
    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), // 50Hz
        std::bind(&NDTLocalization::tfTimerCallback, this)
    );
    // calculate_timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(100), // 10Hz
    //     std::bind(&NDTLocalization::calculateTimerCb, this)
    // );
    std::thread(&NDTLocalization::calculatendt, this).detach();
}


// --- 地图回调 ---
void NDTLocalization::mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if(map_loaded_ || msg->width == 0) return;

    pcl::fromROSMsg(*msg, *map_cloud_);
    ndt_.setInputTarget(map_cloud_);
    map_loaded_ = true;

    RCLCPP_INFO(this->get_logger(),
        "Map received: %ld points", map_cloud_->size());
}

// --- 初始位姿回调 ---
void NDTLocalization::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if(initial_pose_received_) return;
    Eigen::Quaternionf q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);

    current_pose_.setIdentity();
    current_pose_.block<3,3>(0,0) = q.toRotationMatrix();
    current_pose_(0,3) = msg->pose.pose.position.x;
    current_pose_(1,3) = msg->pose.pose.position.y;
    current_pose_(2,3) = msg->pose.pose.position.z;

    geometry_msgs::msg::TransformStamped tf_odom_base;

    try
    {
        tf_odom_base = tf_buffer_->lookupTransform(
            "odom",
            "base_footprint",
            // msg->header.stamp);
            tf2::TimePointZero);
    }
    catch(tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return;
    }

    odom_base = tf2::transformToEigen(tf_odom_base).matrix().cast<float>();

    predicted_pose_ = current_pose_;
    last_odom_pose_ = current_pose_;
    map_odom = current_pose_;

    initial_pose_received_ = true;
    odom_initialized_ = true;
    state_ = RUNNING;  // ✅ 状态切换
}

// --- 里程计回调 ---
void NDTLocalization::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if(!odom_initialized_) return;

    Eigen::Matrix4f odom_pose = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    odom_pose.block<3,3>(0,0) = q.toRotationMatrix();
    odom_pose(0,3) = msg->pose.pose.position.x;
    odom_pose(1,3) = msg->pose.pose.position.y;
    odom_pose(2,3) = msg->pose.pose.position.z;

    Eigen::Matrix4f odom_delta = last_odom_pose_.inverse() * odom_pose;
    predicted_pose_ = current_pose_ * odom_delta;
    last_odom_pose_ = odom_pose;
}

// --- 点云回调 ---
void NDTLocalization::pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if(state_ != RUNNING || !map_loaded_) return;

    cloud.reset(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msg, *cloud);
    cloud_msg = msg;

    received_points_ = true;

    // RCLCPP_INFO(this->get_logger(), "got points:)");

    // pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);

    // for (const auto& point : cloud->points)
    // {
    //     if (point.z >= 0.1 && point.z <= 2.5)
    //     {
    //         filtered_cloud->points.push_back(point);
    //     }
    // }

    // cloud = filtered_cloud;   

    // --- Voxel 滤波 ---

    // pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>);
    // voxel_filter_.setInputCloud(cloud);
    // voxel_filter_.filter(*filtered);

    // // --- NDT 配准 ---
    // ndt_.setInputSource(filtered);
    // pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>);
    // ndt_.align(*output, predicted_pose_);

    // // if(ndt_.hasConverged())
    // // {
    // //     Eigen::Matrix4f map_lidar_ = ndt_.getFinalTransformation();
    // //     double score = ndt_.getFitnessScore();
    // //     Eigen::Matrix4f map_base = map_lidar_ * lidar_base_;

    // //     if(score < 2.0)
    // //         current_pose_ = map_base;
    // //     else
    // //     {
    // //         RCLCPP_WARN(this->get_logger(), "High NDT score %.3f, using predicted pose", score);
    // //         current_pose_ = predicted_pose_;
    // //     }
    // // }
    // // else
    // // {
    // //     RCLCPP_WARN(this->get_logger(),"NDT did not converge, using predicted pose");
    // //     current_pose_ = predicted_pose_;
    // // }
    // if(ndt_.hasConverged())
    // {
    // Eigen::Matrix4f map_lidar_ = ndt_.getFinalTransformation();
    // double score = ndt_.getFitnessScore();
    // Eigen::Matrix4f map_base = map_lidar_ * lidar_base_;
    // Eigen::Matrix4f ndt_pose = map_base;

    // // --- 计算误差 ---
    // Eigen::Matrix4f error = predicted_pose_.inverse() * ndt_pose;

    // float trans_error = error.block<3,1>(0,3).norm();
    // Eigen::Matrix3f R = error.block<3,3>(0,0);
    // float angle_error = std::acos(std::min(1.0f, std::max(-1.0f, (R.trace()-1)/2)));

    // // --- 阈值 ---
    // const float trans_thresh_small = 0.1;   // 10cm
    // const float trans_thresh_large = 1.0;   // 1m

    // // --- 融合 ---
    // if(score < 2.0)
    // {
    //     if(trans_error < trans_thresh_small)
    //     {
    //         // ✅ 小误差：完全信任 odom
    //         current_pose_ = predicted_pose_;
    //     }
    //     else
    //     {
    //         // ✅ 平滑融合（关键）
    //         float alpha = std::min(1.0f, trans_error / trans_thresh_large);

    //         Eigen::Vector3f t_pred = predicted_pose_.block<3,1>(0,3);
    //         Eigen::Vector3f t_ndt  = ndt_pose.block<3,1>(0,3);

    //         Eigen::Vector3f t_fused = (1 - alpha) * t_pred + alpha * t_ndt;

    //         Eigen::Quaternionf q_pred(predicted_pose_.block<3,3>(0,0));
    //         Eigen::Quaternionf q_ndt(ndt_pose.block<3,3>(0,0));

    //         Eigen::Quaternionf q_fused = q_pred.slerp(alpha, q_ndt);

    //         current_pose_.setIdentity();
    //         current_pose_.block<3,3>(0,0) = q_fused.toRotationMatrix();
    //         current_pose_.block<3,1>(0,3) = t_fused;
    //     }
    // }
    // else
    // {
    //     current_pose_ = predicted_pose_;
    // }
    // }

    // // geometry_msgs::msg::TransformStamped tf_odom_base;

    // // try
    // // {
    // //     tf_odom_base = tf_buffer_->lookupTransform(
    // //         "odom",
    // //         "base_footprint",
    // //         msg->header.stamp);
    // //         // tf2::TimePointZero);
    // // }
    // // catch(tf2::TransformException &ex)
    // // {
    // //     RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    // //     return;
    // // }

    // // odom_base = tf2::transformToEigen(tf_odom_base).matrix().cast<float>();
    // // map_odom = current_pose_ * odom_base.inverse();
    // odom_base = map_odom.inverse()*current_pose_;

    // geometry_msgs::msg::TransformStamped tf;
    // tf.header.stamp = cloud_msg->header.stamp;
    // tf.header.frame_id = "odom";
    // tf.child_frame_id = "base_footprint";

    // tf.transform.translation.x = odom_base(0,3);
    // tf.transform.translation.y = odom_base(1,3);
    // tf.transform.translation.z = odom_base(2,3);

    // Eigen::Quaternionf q(odom_base.block<3,3>(0,0));
    // tf.transform.rotation.x = q.x();
    // tf.transform.rotation.y = q.y();
    // tf.transform.rotation.z = q.z();
    // tf.transform.rotation.w = q.w();

    // tf_broadcaster_->sendTransform(tf);
    
}
/*
void NDTLocalization::calculateTimerCb()
{
    if(!received_points_) return;
    // while(1){
    pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>);
    voxel_filter_.setInputCloud(cloud);
    voxel_filter_.filter(*filtered);

    // --- NDT 配准 ---
    ndt_.setInputSource(filtered);
    pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>);
    ndt_.align(*output, predicted_pose_);

    // if(ndt_.hasConverged())
    // {
    //     Eigen::Matrix4f map_lidar_ = ndt_.getFinalTransformation();
    //     double score = ndt_.getFitnessScore();
    //     Eigen::Matrix4f map_base = map_lidar_ * lidar_base_;

    //     if(score < 2.0)
    //         current_pose_ = map_base;
    //     else
    //     {
    //         RCLCPP_WARN(this->get_logger(), "High NDT score %.3f, using predicted pose", score);
    //         current_pose_ = predicted_pose_;
    //     }
    // }
    // else
    // {
    //     RCLCPP_WARN(this->get_logger(),"NDT did not converge, using predicted pose");
    //     current_pose_ = predicted_pose_;
    // }
    if(ndt_.hasConverged())
    {
    Eigen::Matrix4f map_lidar_ = ndt_.getFinalTransformation();
    double score = ndt_.getFitnessScore();
    Eigen::Matrix4f map_base = map_lidar_ * lidar_base_;
    Eigen::Matrix4f ndt_pose = map_base;

    // --- 计算误差 ---
    Eigen::Matrix4f error = predicted_pose_.inverse() * ndt_pose;

    float trans_error = error.block<3,1>(0,3).norm();
    Eigen::Matrix3f R = error.block<3,3>(0,0);
    float angle_error = std::acos(std::min(1.0f, std::max(-1.0f, (R.trace()-1)/2)));

    // --- 阈值 ---
    const float trans_thresh_small = 0.1;   // 10cm
    const float trans_thresh_large = 1.0;   // 1m

    // --- 融合 ---
    if(score < 2.0)
    {
        if(trans_error < trans_thresh_small)
        {
            // ✅ 小误差：完全信任 odom
            current_pose_ = predicted_pose_;
        }
        else
        {
            // ✅ 平滑融合（关键）
            float alpha = std::min(1.0f, trans_error / trans_thresh_large);

            Eigen::Vector3f t_pred = predicted_pose_.block<3,1>(0,3);
            Eigen::Vector3f t_ndt  = ndt_pose.block<3,1>(0,3);

            Eigen::Vector3f t_fused = (1 - alpha) * t_pred + alpha * t_ndt;

            Eigen::Quaternionf q_pred(predicted_pose_.block<3,3>(0,0));
            Eigen::Quaternionf q_ndt(ndt_pose.block<3,3>(0,0));

            Eigen::Quaternionf q_fused = q_pred.slerp(alpha, q_ndt);

            current_pose_.setIdentity();
            current_pose_.block<3,3>(0,0) = q_fused.toRotationMatrix();
            current_pose_.block<3,1>(0,3) = t_fused;
        }
    }
    else
    {
        current_pose_ = predicted_pose_;
    }
    }

    // geometry_msgs::msg::TransformStamped tf_odom_base;

    // try
    // {
    //     tf_odom_base = tf_buffer_->lookupTransform(
    //         "odom",
    //         "base_footprint",
    //         msg->header.stamp);
    //         // tf2::TimePointZero);
    // }
    // catch(tf2::TransformException &ex)
    // {
    //     RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    //     return;
    // }

    // odom_base = tf2::transformToEigen(tf_odom_base).matrix().cast<float>();
    // map_odom = current_pose_ * odom_base.inverse();
    odom_base = map_odom.inverse()*current_pose_;

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = cloud_msg->header.stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";

    tf.transform.translation.x = odom_base(0,3);
    tf.transform.translation.y = odom_base(1,3);
    tf.transform.translation.z = odom_base(2,3);

    Eigen::Quaternionf q(odom_base.block<3,3>(0,0));
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    // usleep(1000*1000);
    // }
}
*/
void NDTLocalization::calculatendt()
{
    if(!received_points_) return;
    while(1){
    pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>);
    voxel_filter_.setInputCloud(cloud);
    voxel_filter_.filter(*filtered);

    // --- NDT 配准 ---
    ndt_.setInputSource(filtered);
    pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>);
    ndt_.align(*output, predicted_pose_);

    // if(ndt_.hasConverged())
    // {
    //     Eigen::Matrix4f map_lidar_ = ndt_.getFinalTransformation();
    //     double score = ndt_.getFitnessScore();
    //     Eigen::Matrix4f map_base = map_lidar_ * lidar_base_;

    //     if(score < 2.0)
    //         current_pose_ = map_base;
    //     else
    //     {
    //         RCLCPP_WARN(this->get_logger(), "High NDT score %.3f, using predicted pose", score);
    //         current_pose_ = predicted_pose_;
    //     }
    // }
    // else
    // {
    //     RCLCPP_WARN(this->get_logger(),"NDT did not converge, using predicted pose");
    //     current_pose_ = predicted_pose_;
    // }
    if(ndt_.hasConverged())
    {
    Eigen::Matrix4f map_lidar_ = ndt_.getFinalTransformation();
    double score = ndt_.getFitnessScore();
    Eigen::Matrix4f map_base = map_lidar_ * lidar_base_;
    Eigen::Matrix4f ndt_pose = map_base;

    // --- 计算误差 ---
    Eigen::Matrix4f error = predicted_pose_.inverse() * ndt_pose;

    float trans_error = error.block<3,1>(0,3).norm();
    Eigen::Matrix3f R = error.block<3,3>(0,0);
    float angle_error = std::acos(std::min(1.0f, std::max(-1.0f, (R.trace()-1)/2)));

    // --- 阈值 ---
    const float trans_thresh_small = 0.1;   // 10cm
    const float trans_thresh_large = 1.0;   // 1m

    // --- 融合 ---
    if(score < 2.0)
    {
        if(trans_error < trans_thresh_small)
        {
            // ✅ 小误差：完全信任 odom
            current_pose_ = predicted_pose_;
        }
        else
        {
            // ✅ 平滑融合（关键）
            float alpha = std::min(1.0f, trans_error / trans_thresh_large);

            Eigen::Vector3f t_pred = predicted_pose_.block<3,1>(0,3);
            Eigen::Vector3f t_ndt  = ndt_pose.block<3,1>(0,3);

            Eigen::Vector3f t_fused = (1 - alpha) * t_pred + alpha * t_ndt;

            Eigen::Quaternionf q_pred(predicted_pose_.block<3,3>(0,0));
            Eigen::Quaternionf q_ndt(ndt_pose.block<3,3>(0,0));

            Eigen::Quaternionf q_fused = q_pred.slerp(alpha, q_ndt);

            current_pose_.setIdentity();
            current_pose_.block<3,3>(0,0) = q_fused.toRotationMatrix();
            current_pose_.block<3,1>(0,3) = t_fused;
        }
    }
    else
    {
        current_pose_ = predicted_pose_;
    }
    }

    geometry_msgs::msg::TransformStamped tf_odom_base;

    try
    {
        tf_odom_base = tf_buffer_->lookupTransform(
            "odom",
            "base_footprint",
            cloud_msg->header.stamp);
            // tf2::TimePointZero);
    }
    catch(tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return;
    }

    odom_base = tf2::transformToEigen(tf_odom_base).matrix().cast<float>();
    map_odom = current_pose_ * odom_base.inverse();
    // odom_base = map_odom.inverse()*current_pose_;

    // geometry_msgs::msg::TransformStamped tf;
    // tf.header.stamp = cloud_msg->header.stamp;
    // tf.header.frame_id = "odom";
    // tf.child_frame_id = "base_footprint";

    // tf.transform.translation.x = odom_base(0,3);
    // tf.transform.translation.y = odom_base(1,3);
    // tf.transform.translation.z = odom_base(2,3);

    // Eigen::Quaternionf q(odom_base.block<3,3>(0,0));
    // tf.transform.rotation.x = q.x();
    // tf.transform.rotation.y = q.y();
    // tf.transform.rotation.z = q.z();
    // tf.transform.rotation.w = q.w();
    usleep(1000*1000);
    }
}

// --- 定时 TF 发布 ---
void NDTLocalization::tfTimerCallback()
{
    if(state_ != RUNNING && !initial_pose_received_) return;
    // RCLCPP_INFO(this->get_logger(), "tf published:)");
    publishTF(this->now());
    // publishAMCLPose(this->now());
}

// --- 发布 TF ---
void NDTLocalization::publishTF(const rclcpp::Time &stamp)
{
    // geometry_msgs::msg::TransformStamped tf_odom_base;

    // try
    // {
    //     tf_odom_base = tf_buffer_->lookupTransform(
    //         "odom",
    //         "base_footprint",
    //         tf2::TimePointZero);
    // }
    // catch(tf2::TransformException &ex)
    // {
    //     RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    //     return;
    // }

    // odom_base = tf2::transformToEigen(tf_odom_base).matrix().cast<float>();
    // map_odom = current_pose_ * odom_base.inverse();

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";

    tf.transform.translation.x = map_odom(0,3);
    tf.transform.translation.y = map_odom(1,3);
    tf.transform.translation.z = map_odom(2,3);

    Eigen::Quaternionf q(map_odom.block<3,3>(0,0));
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf);
}

