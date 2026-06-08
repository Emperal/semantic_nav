#include "navigation_sdk.h"
#include <tf2/LinearMath/Quaternion.h>
#include <thread>

NavigationSDK::NavigationSDK()
: Node("navigation_sdk"), state_(NavState::IDLE)
{
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
}


void NavigationSDK::navigateTo(double x, double y, double yaw)
{
    if (!client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(get_logger(), "Nav2 action server not available");
        return;
    }

    if(state_ == NavState::RUNNING) return;

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
        std::bind(&NavigationSDK::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

    options.result_callback =
        std::bind(&NavigationSDK::resultCallback, this, std::placeholders::_1);

    client_->async_send_goal(goal, options);

    state_ = NavState::RUNNING;
}

void NavigationSDK::cancel()
{
    client_->async_cancel_all_goals();
    state_ = NavState::CANCELED;
}

NavigationSDK::NavState NavigationSDK::getState()
{
    return state_;
}

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
    RCLCPP_INFO(get_logger(), "Distance remaining: %.2f",
                feedback->distance_remaining);
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
        state_ = NavState::FAILED;
        break;
    }

    RCLCPP_INFO(get_logger(), "Navigation finished");
}
