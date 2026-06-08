#ifndef _NAV_SDK__H_
#define _NAV_SDK__H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <queue>
#include <mutex>
#include <thread>
#include <atomic>

class NavigationSDK : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    enum class NavState
    {
        IDLE,
        RUNNING,
        SUCCESS,
        FAILED,
        CANCELED,
        PAUSED
    };

    struct GoalPose
    {
        double x;
        double y;
        double yaw;
    };

    NavigationSDK();
    ~NavigationSDK();

    // === 外部接口 ===
    void addGoal(double x, double y, double yaw);
    void cancelAll();
    void cancelOne();

    NavState getState();
    size_t getQueueSize();

private:
    // === ROS相关 ===
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // === 状态 ===
    std::atomic<NavState> state_;

    // === 队列 ===
    std::queue<GoalPose> goal_queue_;
    std::mutex queue_mutex_;

    // === 线程 ===
    std::thread worker_thread_;
    std::atomic<bool> running_;

    // === 内部函数 ===
    void processQueue();
    void navigateTo(double x, double y, double yaw);

    // === 回调 ===
    void goalResponseCallback(GoalHandleNav::SharedPtr goal_handle);

    void feedbackCallback(
        GoalHandleNav::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback);

    void resultCallback(const GoalHandleNav::WrappedResult & result);
};
#endif
