#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <limits>
#include <algorithm>

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
      tf_listener_(tf_buffer_),
      running_(true)
    {
        this->declare_parameter<std::string>("recognized_object_topic", "/yoloe/recognized_object");
        this->declare_parameter<std::string>("semantic_map_topic", "/semantic_static_map");
        this->declare_parameter<std::string>("target_frame", "map");

        this->declare_parameter<double>("min_confidence", 0.5);
        this->declare_parameter<double>("voxel_leaf_size", 0.05);

        this->declare_parameter<double>("instance_match_distance", 0.5);
        this->declare_parameter<double>("duplicate_reject_distance", 0.7);
        this->declare_parameter<double>("bbox_iou_threshold", 0.10);

        this->declare_parameter<int>("static_observation_threshold", 4);
        this->declare_parameter<int>("max_queue_size", 30);
        this->declare_parameter<double>("publish_period_sec", 0.5);

        recognized_object_topic_ = this->get_parameter("recognized_object_topic").as_string();
        semantic_map_topic_ = this->get_parameter("semantic_map_topic").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();

        min_confidence_ = this->get_parameter("min_confidence").as_double();
        voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();

        instance_match_distance_ = this->get_parameter("instance_match_distance").as_double();
        duplicate_reject_distance_ = this->get_parameter("duplicate_reject_distance").as_double();
        bbox_iou_threshold_ = this->get_parameter("bbox_iou_threshold").as_double();

        static_observation_threshold_ = this->get_parameter("static_observation_threshold").as_int();
        max_queue_size_ = this->get_parameter("max_queue_size").as_int();
        publish_period_sec_ = this->get_parameter("publish_period_sec").as_double();

        semantic_map_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        recognized_object_sub_ =
            this->create_subscription<semantic_mapping_pkg::msg::RecognizedObject>(
                recognized_object_topic_,
                rclcpp::SensorDataQoS(),
                std::bind(&SemanticStaticMapNode::recognizedObjectCallback, this, _1));

        semantic_map_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                semantic_map_topic_,
                10);

        publish_timer_ =
            this->create_wall_timer(
                std::chrono::duration<double>(publish_period_sec_),
                std::bind(&SemanticStaticMapNode::publishTimerCallback, this));

        processing_thread_ =
            std::thread(&SemanticStaticMapNode::processingLoop, this);

        RCLCPP_INFO(this->get_logger(), "Semantic Static Map Node started.");
        RCLCPP_INFO(this->get_logger(), "Subscribe: %s", recognized_object_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish: %s", semantic_map_topic_.c_str());
    }

    ~SemanticStaticMapNode()
    {
        running_ = false;
        queue_cv_.notify_all();

        if (processing_thread_.joinable())
        {
            processing_thread_.join();
        }
    }

private:
    struct SemanticInstance
    {
        int instance_id = -1;
        std::string label;

        Eigen::Vector3f centroid;
        Eigen::Vector3f frozen_centroid;

        int bbox_x1 = 0;
        int bbox_y1 = 0;
        int bbox_x2 = 0;
        int bbox_y2 = 0;

        int observation_count = 0;

        bool inserted_to_static_map = false;
        bool frozen = false;
    };

private:
    std::string recognized_object_topic_;
    std::string semantic_map_topic_;
    std::string target_frame_;

    double min_confidence_;
    double voxel_leaf_size_;

    double instance_match_distance_;
    double duplicate_reject_distance_;
    double bbox_iou_threshold_;

    int static_observation_threshold_;
    int max_queue_size_;
    double publish_period_sec_;

    int next_instance_id_ = 0;

    rclcpp::Subscription<semantic_mapping_pkg::msg::RecognizedObject>::SharedPtr recognized_object_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr semantic_map_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr semantic_map_;
    std::vector<SemanticInstance> instances_;

    std::deque<semantic_mapping_pkg::msg::RecognizedObject::SharedPtr> msg_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    std::mutex map_msg_mutex_;
    sensor_msgs::msg::PointCloud2 latest_semantic_map_msg_;
    bool has_semantic_map_msg_ = false;

    std::thread processing_thread_;
    std::atomic<bool> running_;

private:
    void recognizedObjectCallback(
        const semantic_mapping_pkg::msg::RecognizedObject::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);

            if (static_cast<int>(msg_queue_.size()) >= max_queue_size_)
            {
                msg_queue_.pop_front();
            }

            msg_queue_.push_back(msg);
        }

        queue_cv_.notify_one();
    }

    void processingLoop()
    {
        while (running_)
        {
            semantic_mapping_pkg::msg::RecognizedObject::SharedPtr msg;

            {
                std::unique_lock<std::mutex> lock(queue_mutex_);

                queue_cv_.wait(
                    lock,
                    [this]()
                    {
                        return !msg_queue_.empty() || !running_;
                    });

                if (!running_)
                {
                    break;
                }

                msg = msg_queue_.front();
                msg_queue_.pop_front();
            }

            if (msg)
            {
                processRecognizedObject(msg);
            }
        }
    }

    void processRecognizedObject(
        const semantic_mapping_pkg::msg::RecognizedObject::SharedPtr msg)
    {
        if (msg->label.empty() || msg->label == "unknown")
        {
            return;
        }

        if (msg->confidence < min_confidence_)
        {
            return;
        }

        if (msg->cloud.header.frame_id.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Object cloud frame_id is empty.");
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

        pcl::fromROSMsg(transformed_cloud_msg, *object_cloud);

        if (object_cloud->empty())
        {
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        voxelFilter(object_cloud, filtered_cloud);

        if (filtered_cloud->empty())
        {
            return;
        }

        Eigen::Vector4f centroid4;
        pcl::compute3DCentroid(*filtered_cloud, centroid4);

        Eigen::Vector3f centroid(
            centroid4.x(),
            centroid4.y(),
            centroid4.z());

        int inserted_nearby_index =
            findInsertedInstanceNearby(
                msg->label,
                centroid,
                msg->bbox_x1,
                msg->bbox_y1,
                msg->bbox_x2,
                msg->bbox_y2);

        if (inserted_nearby_index >= 0)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Reject duplicate: label=%s near inserted instance_id=%d",
                msg->label.c_str(),
                instances_[inserted_nearby_index].instance_id);
            return;
        }

        int instance_index =
            associateOrCreateInstance(
                msg->label,
                centroid,
                msg->bbox_x1,
                msg->bbox_y1,
                msg->bbox_x2,
                msg->bbox_y2);

        if (instance_index < 0)
        {
            return;
        }

        auto & instance = instances_[instance_index];

        RCLCPP_INFO(
            this->get_logger(),
            "Associated: label=%s, instance_id=%d, count=%d/%d",
            instance.label.c_str(),
            instance.instance_id,
            instance.observation_count,
            static_observation_threshold_);

        if (instance.inserted_to_static_map)
        {
            return;
        }

        if (instance.observation_count < static_observation_threshold_)
        {
            return;
        }

        *semantic_map_ += *filtered_cloud;

        downsampleSemanticMap();

        instance.inserted_to_static_map = true;
        instance.frozen = true;
        instance.frozen_centroid = instance.centroid;

        updateLatestSemanticMapMsg();

        RCLCPP_INFO(
            this->get_logger(),
            "Inserted static object: label=%s, instance_id=%d, total_points=%zu",
            instance.label.c_str(),
            instance.instance_id,
            semantic_map_->points.size());
    }

    int associateOrCreateInstance(
        const std::string & label,
        const Eigen::Vector3f & centroid,
        int bbox_x1,
        int bbox_y1,
        int bbox_x2,
        int bbox_y2)
    {
        int best_index = -1;
        float best_distance = std::numeric_limits<float>::max();

        for (size_t i = 0; i < instances_.size(); ++i)
        {
            auto & instance = instances_[i];

            if (instance.label != label)
            {
                continue;
            }

            if (instance.inserted_to_static_map)
            {
                continue;
            }

            float distance =
                (instance.centroid - centroid).norm();

            double iou =
                computeBboxIoU(
                    bbox_x1, bbox_y1, bbox_x2, bbox_y2,
                    instance.bbox_x1, instance.bbox_y1,
                    instance.bbox_x2, instance.bbox_y2);

            bool centroid_match =
                distance < instance_match_distance_;

            bool bbox_match =
                iou > bbox_iou_threshold_;

            if (centroid_match && bbox_match && distance < best_distance)
            {
                best_distance = distance;
                best_index = static_cast<int>(i);
            }
        }

        if (best_index >= 0)
        {
            auto & instance = instances_[best_index];

            instance.centroid =
                0.9f * instance.centroid
                + 0.1f * centroid;

            instance.bbox_x1 =
                static_cast<int>(0.8 * instance.bbox_x1 + 0.2 * bbox_x1);
            instance.bbox_y1 =
                static_cast<int>(0.8 * instance.bbox_y1 + 0.2 * bbox_y1);
            instance.bbox_x2 =
                static_cast<int>(0.8 * instance.bbox_x2 + 0.2 * bbox_x2);
            instance.bbox_y2 =
                static_cast<int>(0.8 * instance.bbox_y2 + 0.2 * bbox_y2);

            instance.observation_count++;

            return best_index;
        }

        SemanticInstance new_instance;
        new_instance.instance_id = next_instance_id_++;
        new_instance.label = label;
        new_instance.centroid = centroid;
        new_instance.frozen_centroid = centroid;

        new_instance.bbox_x1 = bbox_x1;
        new_instance.bbox_y1 = bbox_y1;
        new_instance.bbox_x2 = bbox_x2;
        new_instance.bbox_y2 = bbox_y2;

        new_instance.observation_count = 1;
        new_instance.inserted_to_static_map = false;
        new_instance.frozen = false;

        instances_.push_back(new_instance);

        RCLCPP_INFO(
            this->get_logger(),
            "New instance: label=%s, instance_id=%d, bbox=[%d,%d,%d,%d]",
            label.c_str(),
            new_instance.instance_id,
            bbox_x1,
            bbox_y1,
            bbox_x2,
            bbox_y2);

        return static_cast<int>(instances_.size() - 1);
    }

    int findInsertedInstanceNearby(
        const std::string & label,
        const Eigen::Vector3f & centroid,
        int bbox_x1,
        int bbox_y1,
        int bbox_x2,
        int bbox_y2)
    {
        int best_index = -1;
        float best_distance = std::numeric_limits<float>::max();

        for (size_t i = 0; i < instances_.size(); ++i)
        {
            const auto & instance = instances_[i];

            if (!instance.inserted_to_static_map)
            {
                continue;
            }

            if (instance.label != label)
            {
                continue;
            }

            float distance =
                (instance.frozen_centroid - centroid).norm();

            double iou =
                computeBboxIoU(
                    bbox_x1, bbox_y1, bbox_x2, bbox_y2,
                    instance.bbox_x1, instance.bbox_y1,
                    instance.bbox_x2, instance.bbox_y2);

            bool same_instance =
                distance < duplicate_reject_distance_
                && iou > bbox_iou_threshold_;

            if (same_instance && distance < best_distance)
            {
                best_distance = distance;
                best_index = static_cast<int>(i);
            }
        }

        return best_index;
    }

    double computeBboxIoU(
        int ax1, int ay1, int ax2, int ay2,
        int bx1, int by1, int bx2, int by2)
    {
        int inter_x1 = std::max(ax1, bx1);
        int inter_y1 = std::max(ay1, by1);
        int inter_x2 = std::min(ax2, bx2);
        int inter_y2 = std::min(ay2, by2);

        int inter_w = std::max(0, inter_x2 - inter_x1);
        int inter_h = std::max(0, inter_y2 - inter_y1);

        double inter_area =
            static_cast<double>(inter_w * inter_h);

        double area_a =
            static_cast<double>(
                std::max(0, ax2 - ax1) *
                std::max(0, ay2 - ay1));

        double area_b =
            static_cast<double>(
                std::max(0, bx2 - bx1) *
                std::max(0, by2 - by1));

        double union_area =
            area_a + area_b - inter_area;

        if (union_area <= 1e-6)
        {
            return 0.0;
        }

        return inter_area / union_area;
    }

    void voxelFilter(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output)
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;

        voxel_filter.setInputCloud(input);

        const float leaf =
            static_cast<float>(voxel_leaf_size_);

        voxel_filter.setLeafSize(
            leaf,
            leaf,
            leaf);

        voxel_filter.filter(*output);
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

    void updateLatestSemanticMapMsg()
    {
        sensor_msgs::msg::PointCloud2 semantic_map_msg;

        pcl::toROSMsg(
            *semantic_map_,
            semantic_map_msg);

        semantic_map_msg.header.frame_id =
            target_frame_;

        semantic_map_msg.header.stamp =
            this->now();

        {
            std::lock_guard<std::mutex> lock(map_msg_mutex_);
            latest_semantic_map_msg_ = semantic_map_msg;
            has_semantic_map_msg_ = true;
        }
    }

    void publishTimerCallback()
    {
        sensor_msgs::msg::PointCloud2 msg;

        {
            std::lock_guard<std::mutex> lock(map_msg_mutex_);

            if (!has_semantic_map_msg_)
            {
                return;
            }

            msg = latest_semantic_map_msg_;
            msg.header.stamp = this->now();
        }

        semantic_map_pub_->publish(msg);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<SemanticStaticMapNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}