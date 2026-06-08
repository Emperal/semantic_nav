#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <chrono>

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
using namespace std::chrono_literals;

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
        this->declare_parameter<double>("voxel_leaf_size", 0.03);
        this->declare_parameter<double>("static_distance_threshold", 0.20);
        this->declare_parameter<int>("static_observation_threshold", 5);
        this->declare_parameter<int>("max_queue_size", 5);
        this->declare_parameter<double>("publish_period_sec", 0.5);
        this->declare_parameter<bool>("avoid_duplicate_insert", true);

        recognized_object_topic_ = this->get_parameter("recognized_object_topic").as_string();
        semantic_map_topic_ = this->get_parameter("semantic_map_topic").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();

        min_confidence_ = this->get_parameter("min_confidence").as_double();
        voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
        static_distance_threshold_ = this->get_parameter("static_distance_threshold").as_double();
        static_observation_threshold_ = this->get_parameter("static_observation_threshold").as_int();
        max_queue_size_ = this->get_parameter("max_queue_size").as_int();
        publish_period_sec_ = this->get_parameter("publish_period_sec").as_double();
        avoid_duplicate_insert_ = this->get_parameter("avoid_duplicate_insert").as_bool();

        semantic_map_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        while (rclcpp::ok()) {
            try {
                transform = tf_buffer_.lookupTransform(
                    target_frame,     
                    "camera_color_optical_frame",   
                    rclcpp::Time(0),    
                    rclcpp::Duration::from_seconds(0.5) 
                );
                break;
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(100)); 
            }
        }

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
    struct StaticCandidate
    {
        std::string label;
        Eigen::Vector3f position;
        int stable_count = 0;
        bool inserted = false;
    };

private:
    std::string recognized_object_topic_;
    std::string semantic_map_topic_;
    std::string target_frame_;

    double min_confidence_;
    double voxel_leaf_size_;
    double static_distance_threshold_;
    int static_observation_threshold_;
    int max_queue_size_;
    double publish_period_sec_;
    bool avoid_duplicate_insert_;

    rclcpp::Subscription<semantic_mapping_pkg::msg::RecognizedObject>::SharedPtr recognized_object_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr semantic_map_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    geometry_msgs::msg::TransformStamped transform;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr semantic_map_;
    std::vector<StaticCandidate> static_candidates_;

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
            RCLCPP_WARN(this->get_logger(), "Skip unknown object.");
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
            RCLCPP_WARN(this->get_logger(), "Object cloud frame_id is empty.");
            return;
        }

        sensor_msgs::msg::PointCloud2 transformed_cloud_msg;

        tf2::doTransform(
            msg->cloud,
            transformed_cloud_msg,
            transform);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg(transformed_cloud_msg, *object_cloud);

        if (object_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Transformed object cloud is empty.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        voxelFilter(object_cloud, filtered_cloud);

        if (filtered_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Filtered object cloud is empty.");
            return;
        }

        Eigen::Vector4f centroid4;
        pcl::compute3DCentroid(*filtered_cloud, centroid4);

        Eigen::Vector3f centroid(
            centroid4.x(),
            centroid4.y(),
            centroid4.z());

        int candidate_index =
            updateStaticCandidate(
                msg->label,
                centroid);

        if (candidate_index < 0)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Candidate updated but not static yet: label=%s",
                msg->label.c_str());
            return;
        }

        if (avoid_duplicate_insert_)
        {
            if (isAlreadyInsertedNearby(msg->label, centroid, candidate_index))
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Static object already inserted nearby: label=%s",
                    msg->label.c_str());

                static_candidates_[candidate_index].inserted = true;
                return;
            }
        }

        *semantic_map_ += *filtered_cloud;

        downsampleSemanticMap();

        static_candidates_[candidate_index].inserted = true;

        updateLatestSemanticMapMsg();

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

        const float leaf =
            static_cast<float>(voxel_leaf_size_);

        voxel_filter.setLeafSize(
            leaf,
            leaf,
            leaf);

        voxel_filter.filter(*output);
    }

    int updateStaticCandidate(
        const std::string & label,
        const Eigen::Vector3f & current_position)
    {
        for (size_t i = 0; i < static_candidates_.size(); ++i)
        {
            auto & candidate = static_candidates_[i];

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
                    return static_cast<int>(i);
                }

                return -1;
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

        return -1;
    }

    bool isAlreadyInsertedNearby(
        const std::string & label,
        const Eigen::Vector3f & current_position,
        int self_index)
    {
        for (size_t i = 0; i < static_candidates_.size(); ++i)
        {
            if (static_cast<int>(i) == self_index)
            {
                continue;
            }

            const auto & candidate = static_candidates_[i];

            if (!candidate.inserted)
            {
                continue;
            }

            if (candidate.label != label)
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