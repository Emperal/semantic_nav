#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/opencv.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <thread> 
#include <modbus/modbus.h> 

#include <cmath>
#include <vector>
using NavigateToPose = nav2_msgs::action::NavigateToPose;


using std::placeholders::_1;
using std::placeholders::_2;

class ScanMapMatcher : public rclcpp::Node
{
public:
  ScanMapMatcher() : Node("scan_map_matcher")
  {
    this->nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // 等待服务器启动
    // while (!this->nav_client_->wait_for_action_server(1s)) {
    //   RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    // }
    usleep(1000000);
    initpose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1);
    usleep(1000*1000);
    // for(int i = 0;i<20;i++){ `
    //     publishInitialPose(0.10, 0.20, 20.0);
    // }
    // usleep(1000*1000*20);
        for(int j = 0;j<20;j++){
      // start_navigation(2.941542148590088,-0.23749884963035583,-2.07);

    // start_navigation(1.0,1.0,40);}
    //  modbus_thread_ = std::thread(&ScanMapMatcher::modbus_listen, this);
    // modbus_thread_.detach();
    start_navigation(0.8864645957946777,1.8454951047897339,1.57);
    // start_navigation(-0.7815746665000916,-0.0131874084472656,1.07);
    // start_navigation(-1.1793692111968994,1.630757451057434,2.07);
  }
    usleep(1000*1000*20);

        for(int j = 0;j<20;j++){
      // start_navigation(2.941542148590088,-0.23749884963035583,-1.57);

    // start_navigation(1.0,1.0,40);}
    //  modbus_thread_ = std::thread(&ScanMapMatcher::modbus_listen, this);
    // modbus_thread_.detach();
    start_navigation(0.8864645957946777,1.8454951047897339,1.57);
    // start_navigation(-0.7815746665000916,-0.0131874084472656,1.07);
    // start_navigation(-1.1793692111968994,1.630757451057434,2.07);
  }
    usleep(1000*1000*20);

        for(int j = 0;j<20;j++){
      // start_navigation(2.941542148590088,-0.23749884963035583,-1.57);

    // start_navigation(1.0,1.0,40);}
    //  modbus_thread_ = std::thread(&ScanMapMatcher::modbus_listen, this);
    // modbus_thread_.detach();
    // start_navigation(0.8864645957946777,1.8454951047897339,1.57);
    start_navigation(-0.7815746665000916,-0.0131874084472656,1.07);
    // start_navigation(-1.1793692111968994,1.630757451057434,2.07);
  }
    usleep(1000*1000*20);

    for(int j = 0;j<20;j++){
      // start_navigation(2.941542148590088,-0.23749884963035583,-1.57);

    // start_navigation(1.0,1.0,40);}
    //  modbus_thread_ = std::thread(&ScanMapMatcher::modbus_listen, this);
    // modbus_thread_.detach();
    // start_navigation(0.8864645957946777,1.8454951047897339,1.57);
    // start_navigation(-0.7815746665000916,-0.0131874084472656,1.07);
    start_navigation(-1.1793692111968994,1.630757451057434,2.07);
  }
}

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  bool started_;
  bool navigation_active_;
  std::thread modbus_thread_;
  nav_msgs::msg::OccupancyGrid map_;
  cv::Mat distance_map_;
  modbus_t *mb = nullptr;

  bool map_ready_ = false;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;


    //   void modbus_listen()
    // {
    //   mb = modbus_new_tcp("192.168.1.100", 502);
    // if (mb == nullptr) {
    //   RCLCPP_ERROR(this->get_logger(), "Failed to create Modbus client.");
    //   return;
    // }

    // if (modbus_connect(mb) == -1) {
    //   RCLCPP_ERROR(this->get_logger(), "Failed to connect to Modbus server.");
    //   modbus_free(mb);
    //   return;
    // }

    // while (rclcpp::ok())
    // {
    //   uint8_t tab_reg[32];  // 用于存储 Modbus 读取的寄存器值
    //   int rc = modbus_read_bits(mb, 0, 1, tab_reg);  // 读取线圈

    //   if (rc == -1) {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to read Modbus coil.");
    //     continue;
    //   }

    //   // 检查线圈值并在收到启动信号时开始导航
    //   bool coil_value = tab_reg[0];
      // if (coil_value && !navigation_active_)
      // {
    //     RCLCPP_INFO(this->get_logger(), "Received signal to start navigation.");
    //     start_navigation(0.0, 0.0, 1.0);
    //   }

    //   std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 每100毫秒检查一次
    // }

    // modbus_close(mb);
    // modbus_free(mb);
    // }

    void start_navigation(double x, double y, double yaw)
    {
        if (started_)
            return;

        started_ = true;

        RCLCPP_INFO(this->get_logger(), "Publishing initial pose...");



        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;

        tf2::Quaternion q;
        q.setRPY(0,0,yaw);

        goal_msg.pose.pose.orientation.x = q.x();
        goal_msg.pose.pose.orientation.y = q.y();
        goal_msg.pose.pose.orientation.z = q.z();
        goal_msg.pose.pose.orientation.w = q.w();

  //  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  //   send_goal_options.goal_response_callback = std::bind(&ScanMapMatcher::responseCallback, this, _1);
  //   send_goal_options.feedback_callback = std::bind(&ScanMapMatcher::feedbackCallback, this, _1, _2);
  //   send_goal_options.result_callback = std::bind(&ScanMapMatcher::resultCallback, this, _1);

    nav_client_->async_send_goal(goal_msg);
  }

  // Response callback when the goal is accepted or rejected
  void responseCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  // Feedback callback for receiving progress updates
  // void feedbackCallback(
  //   const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
  //   const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
  // {
  //   double distance_remaining = feedback->distance_remaining;
  //   RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f m", distance_remaining);

  //   // Send Modbus signal based on distance remaining
  //   if (distance_remaining < 0.1) {
  //     RCLCPP_INFO(this->get_logger(), "Reached the goal.");
  //     send_modbus_signal(1);  // Send signal when goal is reached
  //   } else {
  //     send_modbus_signal(0);  // Otherwise, keep signal as 0
  //   }
  // }

  // void resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
  // {
  //   if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
  //     RCLCPP_INFO(this->get_logger(), "Navigation finished successfully");
  //   } else {
  //     RCLCPP_ERROR(this->get_logger(), "Navigation failed");
  //   }

  //   navigation_active_ = false;  // Reset navigation state
  // }

  // void send_modbus_signal(int signal)
  // {
  //   modbus_t *mb = modbus_new_tcp("192.168.1.100", 502);
  //   if (mb == nullptr) {
  //     RCLCPP_ERROR(this->get_logger(), "Failed to create Modbus client.");
  //     return;
  //   }

  //   if (modbus_connect(mb) == -1) {
  //     RCLCPP_ERROR(this->get_logger(), "Failed to connect to Modbus server.");
  //     modbus_free(mb);
  //     return;
  //   }

  //   uint8_t coil_value = signal == 1 ? 1 : 0;  // 将信号转为线圈的状态
  //   int rc = modbus_write_bits(mb, 0, 1, &coil_value);  // 写入线圈状态

  //   if (rc == -1) {
  //     RCLCPP_ERROR(this->get_logger(), "Failed to send Modbus signal.");
  //   } else {
  //     RCLCPP_INFO(this->get_logger(), "Modbus signal sent successfully.");
  //   }

  //   modbus_close(mb);
  //   modbus_free(mb);
  // }


  void publishInitialPose(double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();

    msg.pose.pose.position.x = 2.9294986477714384;
    msg.pose.pose.position.y = -0.22955531452193056;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = -0.7621380136384762;
    msg.pose.pose.orientation.w = 0.6474145875458768;

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


