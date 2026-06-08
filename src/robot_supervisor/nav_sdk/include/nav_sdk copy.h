#ifndef _NAV_SDK__H_
#define _NAV_SDK__H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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
        CANCELED
    };

    NavigationSDK();

    // void setInitialPose(double x, double y, double yaw);
    void navigateTo(double x, double y, double yaw);
    void cancel();
    NavState getState();

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    NavState state_;

    void goalResponseCallback(GoalHandleNav::SharedPtr goal_handle);
    void feedbackCallback(
        GoalHandleNav::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void resultCallback(const GoalHandleNav::WrappedResult & result);
};
#endif
