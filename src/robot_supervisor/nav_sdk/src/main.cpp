#include "nav_sdk.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NavigationSDK>();

    // spin线程（必须）
    std::thread spin_thread([&]() {
        rclcpp::spin(node);
    });

    // 添加多个目标点（自动排队执行）
    node->addGoal(1.0, 0.0, 0.0);
    node->addGoal(2.0, 1.0, 1.57);
    node->addGoal(0.0, 0.0, 3.14);

    spin_thread.join();

    rclcpp::shutdown();
    return 0;
}
