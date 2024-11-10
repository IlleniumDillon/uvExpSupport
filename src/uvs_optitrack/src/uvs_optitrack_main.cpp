#include "rclcpp/rclcpp.hpp"
#include "uvs_optitrack.hpp"

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<UvsOptitrack>();

    bool initFlag = false;

    while(rclcpp::ok())
    {
        if (!nh->tryConnect())
        {
            RCLCPP_INFO(nh->get_logger(), "Failed to connect to OptiTrack server. Retrying...");
        }
        else
        {
            initFlag = true;
            break;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    if (!initFlag)
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to connect to OptiTrack server.");
        rclcpp::shutdown();
        return 1;
    }

    while (rclcpp::ok())
    {
        nh->spin_once();
        rclcpp::spin_some(nh);
    }

    rclcpp::shutdown();
    return 0;
}