#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>

using std::placeholders::_1;

class ScanMapMatcher : public rclcpp::Node
{
public:
  ScanMapMatcher() : Node("scan_map_matcher")
  {
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                 .transient_local()
                 .reliable();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos, std::bind(&ScanMapMatcher::mapCallback, this, _1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ScanMapMatcher::scanCallback, this, _1));

    initpose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1);

    RCLCPP_INFO(get_logger(), "ScanMapMatcher node started");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_pub_;

  nav_msgs::msg::OccupancyGrid map_;
  cv::Mat distance_map_;
  bool map_ready_ = false;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = *msg;

    int w = map_.info.width;
    int h = map_.info.height;

    cv::Mat occ(h, w, CV_8UC1);

    for (int y = 0; y < h; y++)
    {
      for (int x = 0; x < w; x++)
      {
        int8_t v = map_.data[y * w + x];
        occ.at<uint8_t>(y, x) = (v > 50) ? 0 : 255;
      }
    }

    cv::distanceTransform(occ, distance_map_, cv::DIST_L2, 5);
    distance_map_ *= map_.info.resolution;

    map_ready_ = true;
    RCLCPP_INFO(get_logger(), "Distance map computed");
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    if (!map_ready_)
      return;

    double best_score = -1e9;
    double best_x = 0, best_y = 0, best_yaw = 0;

    double sigma = 0.3;

    for (double yaw = -M_PI; yaw < M_PI; yaw += M_PI / 90.0)
    {
      for (double x = -2.0; x < 2.0; x += 0.1)
      {
        for (double y = -2.0; y < 2.0; y += 0.1)
        {
          double score = computeScore(*scan, x, y, yaw, sigma);
          if (score > best_score)
          {
            best_score = score;
            best_x = x;
            best_y = y;
            best_yaw = yaw;
          }
        }
      }
    }

    publishInitialPose(best_x, best_y, best_yaw);
    RCLCPP_INFO(get_logger(), "Initial pose published");

    scan_sub_.reset();  //once
  }

  double computeScore(const sensor_msgs::msg::LaserScan &scan,
                      double x, double y, double yaw, double sigma)
  {
    double score = 0.0;
    double angle = scan.angle_min;

    for (size_t i = 0; i < scan.ranges.size(); i++, angle += scan.angle_increment)
    {
      double r = scan.ranges[i];
      if (!std::isfinite(r))
        continue;

      double lx = r * std::cos(angle);
      double ly = r * std::sin(angle);

      double mx = std::cos(yaw) * lx - std::sin(yaw) * ly + x;
      double my = std::sin(yaw) * lx + std::cos(yaw) * ly + y;

      int ix = static_cast<int>((mx - map_.info.origin.position.x) / map_.info.resolution);
      int iy = static_cast<int>((my - map_.info.origin.position.y) / map_.info.resolution);

      if (ix < 0 || iy < 0 ||
          ix >= (int)map_.info.width || iy >= (int)map_.info.height)
        continue;

      double d = distance_map_.at<float>(iy, ix);
      score += std::exp(-(d * d) / (2 * sigma * sigma));
    }

    return score;
  }

  void publishInitialPose(double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.pose.covariance[0] = 0.04;
    msg.pose.covariance[7] = 0.04;
    msg.pose.covariance[35] = 0.01;

    initpose_pub_->publish(msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanMapMatcher>());
  rclcpp::shutdown();
  return 0;
}
