#include "rclcpp/rclcpp.hpp"
#include "uvs_optitrack.hpp"

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<UvsOptitrack>();
    while(!nh->tryConnect())
    {
        RCLCPP_INFO(nh->get_logger(), "Failed to connect to OptiTrack server. Retrying...");
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    while (rclcpp::ok())
    {
        nh->spin_once();
        rclcpp::spin_some(nh);
    }

    rclcpp::shutdown();
    return 0;
}