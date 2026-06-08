#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>

using std::placeholders::_1;

class SemanticStaticMapNode : public rclcpp::Node
{
public:

    SemanticStaticMapNode()
    : Node("semantic_static_map_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        object_cloud_sub_ =
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/object_cloud",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &SemanticStaticMapNode::objectCloudCallback,
                    this,
                    _1));

        semantic_map_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/semantic_static_map",
                10);

        semantic_map_.reset(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        RCLCPP_INFO(
            this->get_logger(),
            "Semantic Static Map Node Started");
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        object_cloud_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        semantic_map_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr semantic_map_;

    void objectCloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud2 transformed_cloud;

        try
        {
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_.lookupTransform(
                    "map",
                    msg->header.frame_id,
                    tf2::TimePointZero);

            tf2::doTransform(
                *msg,
                transformed_cloud,
                transform);
        }
        catch (tf2::TransformException & ex)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "TF transform failed: %s",
                ex.what());

            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg(transformed_cloud, *cloud);

        // ===============================
        // Voxel Filter
        // ===============================

        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;

        voxel_filter.setInputCloud(cloud);

        voxel_filter.setLeafSize(
            0.03f,
            0.03f,
            0.03f);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        voxel_filter.filter(*filtered_cloud);

        // ===============================
        // 合并进入静态语义地图
        // ===============================

        *semantic_map_ += *filtered_cloud;

        // ===============================
        // 发布地图
        // ===============================

        sensor_msgs::msg::PointCloud2 semantic_map_msg;

        pcl::toROSMsg(
            *semantic_map_,
            semantic_map_msg);

        semantic_map_msg.header.frame_id = "map";

        semantic_map_msg.header.stamp =
            this->now();

        semantic_map_pub_->publish(
            semantic_map_msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Semantic map updated: %ld points",
            semantic_map_->points.size());
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<SemanticStaticMapNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}