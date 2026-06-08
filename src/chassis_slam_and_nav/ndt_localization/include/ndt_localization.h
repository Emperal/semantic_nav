#ifndef NDT_LOCALIZATION_HPP_
#define NDT_LOCALIZATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>


#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>

enum LocalizationState
{
    UNINITIALIZED,
    INITIALIZING,
    RUNNING
};

class NDTLocalization : public rclcpp::Node
{

public:

    using PointType = pcl::PointXYZI;

    explicit NDTLocalization();

private:

    void pointsCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void mapCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);


    void initialPoseCallback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg);

    void publishTF(const rclcpp::Time &stamp);

    void tfTimerCallback();
    void calculateTimerCb();
    void calculatendt();

    pcl::PointCloud<PointType>::Ptr map_cloud_;
    pcl::PointCloud<PointType>::Ptr cloud;

    pcl::NormalDistributionsTransform<PointType, PointType> ndt_;

    pcl::VoxelGrid<PointType> voxel_filter_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_map_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    Eigen::Matrix4f current_pose_;
    Eigen::Matrix4f last_odom_pose_;
    Eigen::Matrix4f lidar_base_;

    Eigen::Matrix4f predicted_pose_;
    Eigen::Matrix4f odom_base;
    Eigen::Matrix4f map_odom;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::TransformStamped tf_lidar_base;
    geometry_msgs::msg::TransformStamped tf_odom_base;

    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg;

    bool initial_pose_received_;
    bool map_loaded_;
    bool odom_initialized_;
    bool received_points_;

    rclcpp::Time last_tf_publish_time_;
    LocalizationState state_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr calculate_timer_;
    
    // geometry_msgs::msg::TransformStamped tf;
};

#endif
