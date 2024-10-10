#include <rclcpp/rclcpp.hpp>
#include "uvs_embmaster.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<UvEmbMaster>();
    nh->init();
    nh->start();
    while (rclcpp::ok())
    {
        nh->spin_some();
        rclcpp::spin_some(nh);
    }
    rclcpp::shutdown();
    return 0;
}