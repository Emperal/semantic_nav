#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>

#include "semantic_mapping_pkg/msg/recognized_object.hpp"

using std::placeholders::_1;

class SemanticStaticMapNode : public rclcpp::Node
{
public:
    SemanticStaticMapNode()
    : Node("semantic_static_map_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        this->declare_parameter<std::string>(
            "recognized_object_topic",
            "/yoloe/recognized_object");

        this->declare_parameter<std::string>(
            "target_frame",
            "map");

        this->declare_parameter<double>(
            "min_confidence",
            0.5);

        this->declare_parameter<double>(
            "voxel_leaf_size",
            0.03);

        this->declare_parameter<double>(
            "static_distance_threshold",
            0.20);

        this->declare_parameter<int>(
            "static_observation_threshold",
            5);

        this->declare_parameter<bool>(
            "avoid_duplicate_insert",
            true);

        recognized_object_topic_ =
            this->get_parameter("recognized_object_topic").as_string();

        target_frame_ =
            this->get_parameter("target_frame").as_string();

        min_confidence_ =
            this->get_parameter("min_confidence").as_double();

        voxel_leaf_size_ =
            this->get_parameter("voxel_leaf_size").as_double();

        static_distance_threshold_ =
            this->get_parameter("static_distance_threshold").as_double();

        static_observation_threshold_ =
            this->get_parameter("static_observation_threshold").as_int();

        avoid_duplicate_insert_ =
            this->get_parameter("avoid_duplicate_insert").as_bool();

        semantic_map_.reset(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        recognized_object_sub_ =
            this->create_subscription<semantic_mapping_pkg::msg::RecognizedObject>(
                recognized_object_topic_,
                rclcpp::SensorDataQoS(),
                std::bind(
                    &SemanticStaticMapNode::recognizedObjectCallback,
                    this,
                    _1));

        semantic_map_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/semantic_static_map",
                10);

        RCLCPP_INFO(
            this->get_logger(),
            "Semantic Static Map Node started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Subscribe: %s",
            recognized_object_topic_.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "Publish: /semantic_static_map");
    }

private:
    struct StaticCandidate
    {
        std::string label;
        Eigen::Vector3f position;
        int stable_count = 0;
        bool inserted = false;
    };

private:
    std::string recognized_object_topic_;
    std::string target_frame_;

    double min_confidence_;
    double voxel_leaf_size_;
    double static_distance_threshold_;
    int static_observation_threshold_;
    bool avoid_duplicate_insert_;

    rclcpp::Subscription<semantic_mapping_pkg::msg::RecognizedObject>::SharedPtr
        recognized_object_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        semantic_map_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr semantic_map_;

    std::vector<StaticCandidate> static_candidates_;

private:
    void recognizedObjectCallback(
        const semantic_mapping_pkg::msg::RecognizedObject::SharedPtr msg)
    {
        if (msg->label.empty() || msg->label == "unknown")
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Skip unknown object.");
            return;
        }

        if (msg->confidence < min_confidence_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Skip low confidence object: label=%s, confidence=%.2f",
                msg->label.c_str(),
                msg->confidence);
            return;
        }

        if (msg->cloud.header.frame_id.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Object cloud frame_id is empty.");
            return;
        }

        sensor_msgs::msg::PointCloud2 transformed_cloud_msg;

        try
        {
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_.lookupTransform(
                    target_frame_,
                    msg->cloud.header.frame_id,
                    tf2::TimePointZero);

            tf2::doTransform(
                msg->cloud,
                transformed_cloud_msg,
                transform);
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "TF transform failed from %s to %s: %s",
                msg->cloud.header.frame_id.c_str(),
                target_frame_.c_str(),
                ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg(
            transformed_cloud_msg,
            *object_cloud);

        if (object_cloud->empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Transformed object cloud is empty.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        voxelFilter(
            object_cloud,
            filtered_cloud);

        if (filtered_cloud->empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Filtered object cloud is empty.");
            return;
        }

        Eigen::Vector4f centroid4;
        pcl::compute3DCentroid(
            *filtered_cloud,
            centroid4);

        Eigen::Vector3f centroid(
            centroid4.x(),
            centroid4.y(),
            centroid4.z());

        bool should_insert =
            updateStaticCandidate(
                msg->label,
                centroid);

        if (!should_insert)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Candidate updated but not static yet: label=%s",
                msg->label.c_str());
            return;
        }

        if (avoid_duplicate_insert_)
        {
            if (isAlreadyInsertedNearby(msg->label, centroid))
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Static object already inserted nearby: label=%s",
                    msg->label.c_str());
                return;
            }
        }

        *semantic_map_ += *filtered_cloud;

        downsampleSemanticMap();

        publishSemanticMap();

        RCLCPP_INFO(
            this->get_logger(),
            "Inserted static semantic object: label=%s, confidence=%.2f, total_points=%zu",
            msg->label.c_str(),
            msg->confidence,
            semantic_map_->points.size());
    }

    void voxelFilter(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output)
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;

        voxel_filter.setInputCloud(input);

        float leaf =
            static_cast<float>(voxel_leaf_size_);

        voxel_filter.setLeafSize(
            leaf,
            leaf,
            leaf);

        voxel_filter.filter(*output);
    }

    bool updateStaticCandidate(
        const std::string & label,
        const Eigen::Vector3f & current_position)
    {
        for (auto & candidate : static_candidates_)
        {
            if (candidate.label != label)
            {
                continue;
            }

            float distance =
                (candidate.position - current_position).norm();

            if (distance < static_distance_threshold_)
            {
                candidate.position =
                    0.8f * candidate.position
                    + 0.2f * current_position;

                candidate.stable_count++;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Static candidate: label=%s, count=%d/%d, distance=%.3f",
                    label.c_str(),
                    candidate.stable_count,
                    static_observation_threshold_,
                    distance);

                if (!candidate.inserted &&
                    candidate.stable_count >= static_observation_threshold_)
                {
                    candidate.inserted = true;
                    return true;
                }

                return false;
            }
        }

        StaticCandidate new_candidate;
        new_candidate.label = label;
        new_candidate.position = current_position;
        new_candidate.stable_count = 1;
        new_candidate.inserted = false;

        static_candidates_.push_back(new_candidate);

        RCLCPP_INFO(
            this->get_logger(),
            "New static candidate created: label=%s",
            label.c_str());

        return false;
    }

    bool isAlreadyInsertedNearby(
        const std::string & label,
        const Eigen::Vector3f & current_position)
    {
        (void)label;

        for (const auto & candidate : static_candidates_)
        {
            if (!candidate.inserted)
            {
                continue;
            }

            float distance =
                (candidate.position - current_position).norm();

            if (distance < static_distance_threshold_)
            {
                return true;
            }
        }

        return false;
    }

    void downsampleSemanticMap()
    {
        if (semantic_map_->empty())
        {
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_map(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        voxelFilter(
            semantic_map_,
            filtered_map);

        semantic_map_ = filtered_map;
    }

    void publishSemanticMap()
    {
        sensor_msgs::msg::PointCloud2 semantic_map_msg;

        pcl::toROSMsg(
            *semantic_map_,
            semantic_map_msg);

        semantic_map_msg.header.frame_id =
            target_frame_;

        semantic_map_msg.header.stamp =
            this->now();

        semantic_map_pub_->publish(
            semantic_map_msg);
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