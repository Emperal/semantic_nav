#include "ndt_localization.h"

int main(int argc,char **argv)
{

    rclcpp::init(argc,argv);

    auto node =
        std::make_shared<NDTLocalization>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();


    // rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
