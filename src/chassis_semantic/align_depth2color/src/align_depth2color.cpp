// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "sensor_msgs/msg/camera_info.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "tf2_ros/transform_listener.h"
// #include <cv_bridge/cv_bridge.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include "opencv2/opencv.hpp"
// #include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/opencv.hpp>

using namespace std;

class ImageAlignmentNode : public rclcpp::Node
{
public:
    ImageAlignmentNode() : Node("image_alignment_node")
    {
        // 初始化参数
        this->declare_parameter("image_topic", "/camera/color/image_raw");
        this->declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw");
        this->declare_parameter("camera_info_topic", "/camera/color/camera_info");
        this->declare_parameter("output_topic", "/camera/aligned_output");

        image_topic_ = this->get_parameter("image_topic").as_string();
        depth_topic_ = this->get_parameter("depth_topic").as_string();
        camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();

         
        // 创建订阅者
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_, 10, std::bind(&ImageAlignmentNode::image_callback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, 10, std::bind(&ImageAlignmentNode::depth_callback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, 10, std::bind(&ImageAlignmentNode::camera_info_callback, this, std::placeholders::_1));

        // 发布对齐后的图像
        aligned_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

        // 初始化 TF2 监听器
        // tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        while (rclcpp::ok()) {
            try {
                transform = tf_buffer_->lookupTransform(
                    "camera_color_frame",     
                    "camera_depth_frame",   
                    rclcpp::Time(0),    
                    rclcpp::Duration::from_seconds(0.5) 
                );
                break;
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(100)); 
            }
        }
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // 转换为 OpenCV 格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            color_image_ = cv_ptr->image;
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Color image conversion failed: %s", e.what());
        }
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // 转换为 OpenCV 格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");
            depth_image_ = cv_ptr->image;

            // 获取对应的 TF 转换
            

            // 对深度图像进行变换
            cv::Mat aligned_depth_image;
            align_images(depth_image_, transform, aligned_depth_image);

            // 发布对齐后的深度图像
            cv_bridge::CvImage aligned_cv_ptr;
            aligned_cv_ptr.header = msg->header;
            aligned_cv_ptr.encoding = "16UC1";
            aligned_cv_ptr.image = aligned_depth_image;
            aligned_pub_->publish(*aligned_cv_ptr.toImageMsg());
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", e.what());
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Depth image conversion failed: %s", e.what());
        }
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        camera_info_ = *msg;
    }

    void align_images(const cv::Mat &depth_image, const geometry_msgs::msg::TransformStamped &transform,
                       cv::Mat &aligned_depth_image)
    {
        // 这个方法可以利用 TF 转换矩阵，将深度图像对齐到彩色图像
        // 此处我们假设 TF 转换是一个简单的平移加旋转操作
        cv::Mat rotation_matrix = cv::Mat(3, 3, CV_64F);
        rotation_matrix.at<double>(0, 0) = transform.transform.rotation.w;
        // 此处简化了旋转矩阵的处理，您可以扩展以进行更复杂的变换

        // 对深度图像进行对齐，具体根据您的需求实现
        // 这里只是一个示例，您需要根据相机内参、旋转矩阵进行更精确的计算
        cv::warpAffine(depth_image, aligned_depth_image, rotation_matrix, depth_image.size());
    }

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    
    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr aligned_pub_;
    
    // TF 相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 图像和相机信息
    cv::Mat color_image_;
    cv::Mat depth_image_;
    sensor_msgs::msg::CameraInfo camera_info_;

    // 参数
    std::string image_topic_;
    std::string depth_topic_;
    std::string camera_info_topic_;
    std::string output_topic_;

    geometry_msgs::msg::TransformStamped transform;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageAlignmentNode>());
    rclcpp::shutdown();
    return 0;
}