#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdomToTFNode : public rclcpp::Node
{
public:
  OdomToTFNode() : Node("odom_to_tf_node")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/dt/odom_info",
      rclcpp::QoS(50),
      std::bind(&OdomToTFNode::odomCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Odom to TF broadcaster started.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = msg->header.frame_id;        // odom
    tf_msg.child_frame_id = msg->child_frame_id;          // base_footprint

    // translation
    tf_msg.transform.translation.x = msg->pose.pose.position.x;
    tf_msg.transform.translation.y = msg->pose.pose.position.y;
    tf_msg.transform.translation.z = msg->pose.pose.position.z;

    // rotation
    tf_msg.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTFNode>());
  rclcpp::shutdown();
  return 0;
}
