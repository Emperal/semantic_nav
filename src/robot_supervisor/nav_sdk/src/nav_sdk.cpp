#include "nav_sdk.h"
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
// #include "jsonrpc.hpp"
// using namespace pooriayousefi;
// using json = nlohmann::json;
#define LOG_MOD
using namespace std::chrono_literals;

NavigationSDK::NavigationSDK()
: Node("navigation_sdk"), state_(NavState::IDLE), running_(true)
{
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // 等待 action server（只做一次）
    if (!client_->wait_for_action_server(5s))
    {
        RCLCPP_ERROR(get_logger(), "Nav2 action server not available");
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Nav2 action server ready");
    }

    // 启动队列线程
    worker_thread_ = std::thread(&NavigationSDK::processQueue, this);
    // timer_ = create_wall_timer(
    // std::chrono::milliseconds(100),
    // std::bind(&NavigationSDK::processQueue, this));
}

NavigationSDK::~NavigationSDK()
{
    running_ = false;

    if (worker_thread_.joinable())
        worker_thread_.join();
}

//////////////////////////////////////////////////
// 外部接口
//////////////////////////////////////////////////

void NavigationSDK::addGoal(double x, double y, double yaw)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    goal_queue_.push({x, y, yaw});
    RCLCPP_INFO(get_logger(), "Add goal: (%.2f, %.2f, %.2f)", x, y, yaw);
}

void NavigationSDK::cancelAll()
{
    client_->async_cancel_all_goals();

    std::lock_guard<std::mutex> lock(queue_mutex_);
    std::queue<GoalPose> empty;
    std::swap(goal_queue_, empty);

    state_ = NavState::CANCELED;

    RCLCPP_WARN(get_logger(), "All goals canceled");
}

void NavigationSDK::cancelOne()
{
    client_->async_cancel_all_goals();
    state_ = NavState::PAUSED;
    RCLCPP_WARN(get_logger(),"Goal canceled");
}

NavigationSDK::NavState NavigationSDK::getState()
{
    return state_;
}

size_t NavigationSDK::getQueueSize()
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return goal_queue_.size();
}

//////////////////////////////////////////////////
// 队列线程（核心）
//////////////////////////////////////////////////

void NavigationSDK::processQueue()
{
    while (running_)
    {
        // 如果正在执行任务 → 等待
        if (state_ == NavState::RUNNING)
        {
            std::this_thread::sleep_for(100ms);
            continue;
        }

        GoalPose goal;

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);

            if (goal_queue_.empty())
            {
                std::this_thread::sleep_for(100ms);
                continue;
            }

            goal = goal_queue_.front();
            goal_queue_.pop();
        }

        RCLCPP_INFO(get_logger(), "Start goal(%.2f, %.2f, %.2f)", goal.x, goal.y, goal.yaw);

        navigateTo(goal.x, goal.y, goal.yaw);
    }
}

//////////////////////////////////////////////////
// 发送导航目标
//////////////////////////////////////////////////

void NavigationSDK::navigateTo(double x, double y, double yaw)
{
    // if (state_ == NavState::RUNNING)
    //     return;

    NavigateToPose::Goal goal;

    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();

    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    goal.pose.pose.orientation.x = q.x();
    goal.pose.pose.orientation.y = q.y();
    goal.pose.pose.orientation.z = q.z();
    goal.pose.pose.orientation.w = q.w();

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    options.goal_response_callback =
        std::bind(&NavigationSDK::goalResponseCallback, this, std::placeholders::_1);

    options.feedback_callback =
        std::bind(&NavigationSDK::feedbackCallback, this,
                  std::placeholders::_1, std::placeholders::_2);

    options.result_callback =
        std::bind(&NavigationSDK::resultCallback, this, std::placeholders::_1);

    client_->async_send_goal(goal, options);

    state_ = NavState::RUNNING;
}

//////////////////////////////////////////////////
// 回调
//////////////////////////////////////////////////

void NavigationSDK::goalResponseCallback(GoalHandleNav::SharedPtr goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Goal rejected");
        state_ = NavState::FAILED;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Goal accepted");
    }
}

void NavigationSDK::feedbackCallback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
#ifdef LOG_MOD
    RCLCPP_INFO(get_logger(), "Distance remaining: %.2f",
                feedback->distance_remaining);
#endif
    // usleep(1000*1000);
}

void NavigationSDK::resultCallback(const GoalHandleNav::WrappedResult & result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        state_ = NavState::SUCCESS;
        break;
    case rclcpp_action::ResultCode::ABORTED:
        state_ = NavState::FAILED;
        break;
    case rclcpp_action::ResultCode::CANCELED:
        state_ = NavState::CANCELED;
        break;
    default:
        state_ = NavState::RUNNING;
        break;
    }

    RCLCPP_INFO(get_logger(), "Navigation finished");
}