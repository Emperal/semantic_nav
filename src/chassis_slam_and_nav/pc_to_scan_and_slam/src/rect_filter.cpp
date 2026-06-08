#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <limits>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/conversions.h"
#include "pcl/registration/icp.h"
#include "pcl_conversions/pcl_conversions.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <pcl/filters/crop_box.h>
#include <cstring>
#include <algorithm>


class LaserRectFilterNode : public rclcpp::Node
{
public:
LaserRectFilterNode() : Node("laser_rect_filter")
{
  // 参数声明
  this->declare_parameter<double>("rect_center_x", 0.0);
  this->declare_parameter<double>("rect_center_y", 0.0);
  this->declare_parameter<double>("rect_width", 0.2);
  this->declare_parameter<double>("rect_height", 0.2);
  this->declare_parameter<std::string>("input_scan_topic", "laser_origin");
  this->declare_parameter<std::string>("output_scan_topic", "scan");
  this->declare_parameter<std::string>("input_lidar_topic", "/livox/lidar");
  this->declare_parameter<std::string>("output_lidar_topic", "/filtered_cloud");

  // 获取参数
  rect_center_x_ = this->get_parameter("rect_center_x").as_double();
  rect_center_y_ = this->get_parameter("rect_center_y").as_double();
  rect_width_    = this->get_parameter("rect_width").as_double();
  rect_height_   = this->get_parameter("rect_height").as_double();
  input_topic_   = this->get_parameter("input_scan_topic").as_string();
  output_topic_  = this->get_parameter("output_scan_topic").as_string();
  input_lidar_topic_ = this->get_parameter("input_lidar_topic").as_string();
  output_lidar_topic_ = this->get_parameter("output_lidar_topic").as_string();

  filtered_lidar.reset(new pcl::PointCloud<pcl::PointXYZI>());

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LaserRectFilterNode::scanCallback, this, std::placeholders::_1)
  );

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    // "/livox/lidar",
    input_lidar_topic_,
    10,
    std::bind(&LaserRectFilterNode::lidarCallback,this,std::placeholders::_1)
  );

  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    output_topic_, 10
  );

  lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    // "/filtered_cloud", 
    output_lidar_topic_,
    10);

  RCLCPP_INFO(this->get_logger(), "Laser rectangle filter started");
}
private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_lidar;

  double rect_center_x_;
  double rect_center_y_;
  double rect_width_;
  double rect_height_;

  std::string input_topic_;
  std::string output_topic_;
  std::string input_lidar_topic_;
  std::string output_lidar_topic_;

void scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  auto output = *msg;

  double half_w = rect_width_ * 0.5;
  double half_h = rect_height_ * 0.5;

  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    float r = msg->ranges[i];

    if (!std::isfinite(r))
      continue;

    double angle = msg->angle_min + i * msg->angle_increment;

    double x = r * std::cos(angle);
    double y = r * std::sin(angle);

    bool inside_rect =
      (x >= rect_center_x_ - half_w &&
       x <= rect_center_x_ + half_w &&
       y >= rect_center_y_ - half_h &&
       y <= rect_center_y_ + half_h);

    if (inside_rect)
    {
      output.ranges[i] = std::numeric_limits<float>::infinity();
    }
  }

  scan_pub_->publish(output);
}
//*************************************************************** */
// int getFieldOffset(
//   const sensor_msgs::msg::PointCloud2 & msg,
//   const std::string & field_name)
// {
//   for (const auto & field : msg.fields)
//   {
//     if (field.name == field_name)
//     {
//       return static_cast<int>(field.offset);
//     }
//   }
//   return -1;
// }

// float readFloat32(
//   const sensor_msgs::msg::PointCloud2 & msg,
//   size_t point_base,
//   int offset)
// {
//   float value;
//   std::memcpy(&value, &msg.data[point_base + offset], sizeof(float));
//   return value;
// }

// void writeFloat32(
//   sensor_msgs::msg::PointCloud2 & msg,
//   size_t point_base,
//   int offset,
//   float value)
// {
//   std::memcpy(&msg.data[point_base + offset], &value, sizeof(float));
// }

// void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
// {
//   int x_offset = getFieldOffset(*msg, "x");
//   int y_offset = getFieldOffset(*msg, "y");
//   int z_offset = getFieldOffset(*msg, "z");

//   if (x_offset < 0 || y_offset < 0 || z_offset < 0)
//   {
//     RCLCPP_ERROR(this->get_logger(), "PointCloud2 does not contain x/y/z fields");
//     return;
//   }

//   /*
//    * 关键：
//    * 不重新构造 PointCloud2；
//    * 不改变 width/height/row_step/point_step/fields；
//    * 不删除任何点；
//    * 不改变 timestamp/tag/line；
//    * 只把车体点 x/y/z 改成 NaN。
//    */
//   sensor_msgs::msg::PointCloud2 output = *msg;

//   size_t removed_count = 0;
//   size_t invalid_count = 0;

//   const float nan_value = std::numeric_limits<float>::quiet_NaN();

//   for (uint32_t row = 0; row < msg->height; ++row)
//   {
//     for (uint32_t col = 0; col < msg->width; ++col)
//     {
//       size_t point_base =
//         static_cast<size_t>(row) * msg->row_step +
//         static_cast<size_t>(col) * msg->point_step;

//       if (point_base + msg->point_step > msg->data.size())
//       {
//         invalid_count++;
//         continue;
//       }

//       float x = readFloat32(*msg, point_base, x_offset);
//       float y = readFloat32(*msg, point_base, y_offset);
//       float z = readFloat32(*msg, point_base, z_offset);

//       if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
//       {
//         invalid_count++;
//         continue;
//       }

//       /*
//        * 这里是雷达坐标系 livox_frame 下的车体区域。
//        * 根据你的实际安装位置继续调。
//        */
//       bool inside_robot_box =
//         x >= -0.70f && x <= 0.035f &&
//         y >= -0.24f && y <= 0.24f  &&
//         z >= 0.10f  && z <= 2.50f;

//       if (inside_robot_box)
//       {
//         writeFloat32(output, point_base, x_offset, nan_value);
//         writeFloat32(output, point_base, y_offset, nan_value);
//         writeFloat32(output, point_base, z_offset, nan_value);

//         removed_count++;
//       }
//     }
//   }

//   output.is_dense = false;

//   RCLCPP_INFO_THROTTLE(
//     this->get_logger(),
//     *this->get_clock(),
//     2000,
//     "Body filter: keep cloud layout unchanged, invalid body points: %zu, original points: %u",
//     removed_count,
//     msg->width * msg->height
//   );

//   lidar_pub_->publish(output);
// }
//******************************************************************************************   */

void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::fromROSMsg(*msg,*filtered_lidar);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for (const auto& point : filtered_lidar->points)
    {
        if (point.z >= 0.1 && point.z <= 2.5)
        {
            filtered_cloud->points.push_back(point);
        }
    }

    filtered_lidar = filtered_cloud;


    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(filtered_lidar);

    // 设置 box 范围
    crop.setMin(Eigen::Vector4f(-0.7, -0.24, 0.1, 1.0));
    crop.setMax(Eigen::Vector4f(0.035, 0.24, 2.5, 1.0));

    // 关键：true = 删除 box 内点
    crop.setNegative(true);

    crop.filter(*filtered_cloud);

    filtered_lidar = filtered_cloud;
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered_lidar, filtered_msg);
    filtered_msg.header.stamp = msg->header.stamp;
    filtered_msg.header.frame_id = msg->header.frame_id;
    lidar_pub_->publish(filtered_msg);
}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserRectFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

