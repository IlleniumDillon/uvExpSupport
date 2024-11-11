#include <rclcpp/rclcpp.hpp>
#include "uvs_embmaster.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<UvEmbMaster>();
    if (!nh->init())
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to initialize USB driver");
        rclcpp::shutdown();
        return 1;
    }
    nh->start();
    while (rclcpp::ok())
    {
        nh->spin_some();
        rclcpp::spin_some(nh);
    }
    rclcpp::shutdown();
    return 0;
}