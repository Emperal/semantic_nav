#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/conversions.h"
#include "pcl/registration/icp.h"
#include "../include/utilities.h"
#include "pcl_conversions/pcl_conversions.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
const int MAX_ICP_RETRY = 5;


class PcdToMapAlignmentNode : public rclcpp::Node
{
public:
  PcdToMapAlignmentNode() : Node("pcd_to_map_alignment")
  {
    this->declare_parameter<std::string>("pcd_file", "/path/to/default.pcd");
    this->declare_parameter<std::string>("yaml_file","/path/to/defaulst.yaml");
    this->declare_parameter<std::string>("pgm_file","/path/to/defaulst.pgm");

    pcd_file_path = this->get_parameter("pcd_file").as_string();
    yaml_file_path = this->get_parameter("yaml_file").as_string();
    pgm_file_path = this->get_parameter("pgm_file").as_string();
    RCLCPP_INFO(this->get_logger(), "PCD file: %s", pcd_file_path.c_str());
    liosam_3d_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
    icp_fail_count_ = 0;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    while (rclcpp::ok()) {
    try {
        // 读取 base_footprint -> livox_frame
        tf_lidar_base = tf_buffer_->lookupTransform(
            "livox_frame",       // target_frame
            "base_footprint",    // source_frame
            rclcpp::Time(0),     // 最近可用 transform
            rclcpp::Duration::from_seconds(0.5) // 超时等待 0.5 秒
        );
        break; // 成功拿到 transform，跳出循环
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        rclcpp::sleep_for(std::chrono::milliseconds(100)); // 等待再试
    }
  }
    lidar_base_ = tf2::transformToEigen(tf_lidar_base).matrix().cast<float>();


    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/aligned_cloud", 10);

    // lv_filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      // "/lv_filtered_cloud", 10);
    
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
             .transient_local()
             .reliable();
    // rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10));  
    // qos.reliable();  
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", map_qos, std::bind(&PcdToMapAlignmentNode::mapcb, this, std::placeholders::_1));
    // RCLCPP_INFO(this->get_logger(), "PCD file: %s", pcd_file_path.c_str());
    //subscribe /map topic from /map_server
    // map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    //     "/map", rclcpp::SensorDataQoS(), std::bind(&PcdToMapAlignmentNode::mapcb, this, std::placeholders::_1));
    
    // publish pc that has been transformed
    // nav_msgs::msg::OccupancyGrid map;
    // map = cloud_icp_.loadMapFromYamlAndPgm(yaml_file_path,pgm_file_path);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr loser_pc(new pcl::PointCloud<pcl::PointXYZI>());
    // tf = cloud_icp_.align_and_transform(pcd_file_path, map);
    // liosam_3d_pc = cloud_icp_.get_pc();
    // pcl::transformPointCloud(*liosam_3d_pc, *loser_pc, tf);
    // liosam_3d_pc = loser_pc;
    
    pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PcdToMapAlignmentNode::publishCloud, this));
    map_ready_ = 0;

    ndt_initial_pose_pub_ = 
      this->create_publisher<
      geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/ndtinitialpose",10);

    initialpose_pub_ =
      this->create_publisher<
      geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose",10);

    

    lidar_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/filtered_cloud",
      // "/livox/lidar",
      10,
      std::bind(&PcdToMapAlignmentNode::lidarCallback,this,std::placeholders::_1));

    initial_pose_sent_ = false;

  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lv_filtered_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ndt_initial_pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;

  pcl::PointCloud<pcl::PointXYZI>::Ptr liosam_3d_pc;
  std::string pcd_file_path;
  std::string yaml_file_path;
  std::string pgm_file_path;
  int map_ready_;
  Eigen::Matrix4f tf;
  pc2mapUtilities cloud_icp_;

  Eigen::Matrix4f lidar_base_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::TransformStamped tf_lidar_base;
  geometry_msgs::msg::TransformStamped tf_odom_base;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  bool initial_pose_sent_;

  int icp_fail_count_;





  void mapcb(const nav_msgs::msg::OccupancyGrid &map)
  {
    RCLCPP_INFO(this->get_logger(), "unsubscribed from /map");

    pcl::PointCloud<pcl::PointXYZI>::Ptr loser_pc(new pcl::PointCloud<pcl::PointXYZI>());
    tf = cloud_icp_.align_and_transform(pcd_file_path, map);
    liosam_3d_pc = cloud_icp_.get_pc();
    pcl::transformPointCloud(*liosam_3d_pc, *loser_pc, tf);
    liosam_3d_pc = loser_pc;
    map_ready_ = 1;
    map_sub_.reset();
    RCLCPP_INFO(this->get_logger(), "Map alignment done, unsubscribed from /map");
  }

  void publishCloud()
  {
    if(!map_ready_) return;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*liosam_3d_pc, msg);
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    cloud_pub_->publish(msg);
  }



  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (initial_pose_sent_)
      return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan(
        new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*msg, *scan);

    if (scan->empty() || liosam_3d_pc->empty())
    {
      RCLCPP_WARN(this->get_logger(), "Empty point cloud");
      return;
    }

    // ============================
    // ⭐ 第一阶段：粗配准
    // ============================
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp_coarse;
    icp_coarse.setInputSource(scan);
    icp_coarse.setInputTarget(liosam_3d_pc);

    icp_coarse.setMaximumIterations(30);
    icp_coarse.setMaxCorrespondenceDistance(2.0);

    pcl::PointCloud<pcl::PointXYZI> coarse_aligned;
    icp_coarse.align(coarse_aligned);

    if (!icp_coarse.hasConverged())
    {
      icp_fail_count_++;
      RCLCPP_WARN(this->get_logger(), "ICP coarse not converged (%d)", icp_fail_count_);
      if (icp_fail_count_ > MAX_ICP_RETRY)
        resetICP();
      return;
    }

    // ============================
    // ⭐ 第二阶段：精配准
    // ============================
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp_fine;
    icp_fine.setInputSource(coarse_aligned.makeShared());
    icp_fine.setInputTarget(liosam_3d_pc);

    icp_fine.setMaximumIterations(50);
    icp_fine.setMaxCorrespondenceDistance(0.8);
    icp_fine.setTransformationEpsilon(1e-6);
    icp_fine.setEuclideanFitnessEpsilon(1e-6);

    pcl::PointCloud<pcl::PointXYZI> aligned;
    icp_fine.align(aligned);

    if (!icp_fine.hasConverged())
    {
      icp_fail_count_++;
      RCLCPP_WARN(this->get_logger(), "ICP fine not converged (%d)", icp_fail_count_);
      if (icp_fail_count_ > MAX_ICP_RETRY)
        resetICP();
      return;
    }

    // ============================
    // ⭐ 质量评估
    // ============================
    double fitness = icp_fine.getFitnessScore();
    RCLCPP_INFO(this->get_logger(), "ICP fitness: %.4f", fitness);

    if (fitness > 0.8)   // ⚠️ 可调
    {
      icp_fail_count_++;
      RCLCPP_WARN(this->get_logger(), "ICP fitness too large (%d)", icp_fail_count_);
      if (icp_fail_count_ > MAX_ICP_RETRY)
        resetICP();
      return;
    }

    // ============================
    // ⭐ 位姿合理性判断
    // ============================
    Eigen::Matrix4f pose = icp_fine.getFinalTransformation();

    float dx = pose(0,3);
    float dy = pose(1,3);
    float dist = std::sqrt(dx*dx + dy*dy);

    Eigen::Matrix3f rot = pose.block<3,3>(0,0);
    Eigen::AngleAxisf angleAxis(rot);
    float yaw = angleAxis.angle();

    if (dist > 5.0 || yaw > M_PI/3)
    {
      icp_fail_count_++;
      RCLCPP_WARN(this->get_logger(),
        "ICP pose abnormal dist=%.2f yaw=%.2f (%d)", dist, yaw, icp_fail_count_);

      if (icp_fail_count_ > MAX_ICP_RETRY)
        resetICP();

      return;
    }

    // ============================
    // ⭐ 成功
    // ============================
    icp_fail_count_ = 0;
    Eigen::Matrix4f pose_inverse;
    pose_inverse = pose.inverse();
    pose = pose_inverse;

    pose *= lidar_base_;

    publishInitialPose(pose);

    initial_pose_sent_ = true;

    lidar_sub_.reset();

    RCLCPP_INFO(this->get_logger(), "ICP SUCCESS → Initial pose published");
  }


  void publishInitialPose(const Eigen::Matrix4f &pose)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

    msg.pose.pose.position.x = pose(0,3);
    msg.pose.pose.position.y = pose(1,3);
    msg.pose.pose.position.z = pose(2,3);

    Eigen::Matrix3f rot = pose.block<3,3>(0,0);
    Eigen::Quaternionf q(rot);

    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    for(int i=0;i<36;i++)
        msg.pose.covariance[i] = 0.0;

    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.1;

    initial_pose_msg = msg;

    for(int i=0;i<10;i++)
    {
      initialpose_pub_->publish(msg);
      ndt_initial_pose_pub_->publish(initial_pose_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
  }

  void resetICP()
{
  icp_fail_count_ = 0;

  RCLCPP_ERROR(this->get_logger(), "Resetting ICP...");
  lidar_sub_.reset();

  initial_pose_sent_ = false;

  // 重新订阅
  lidar_sub_ =
    this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/filtered_cloud",
      10,
      std::bind(&PcdToMapAlignmentNode::lidarCallback,this,std::placeholders::_1));
}



};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdToMapAlignmentNode>());
  rclcpp::shutdown();
  return 0;
}
