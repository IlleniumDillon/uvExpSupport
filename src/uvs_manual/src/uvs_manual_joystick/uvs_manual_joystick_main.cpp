#include "rclcpp/rclcpp.hpp"
#include "uvs_manual_joystick.hpp"

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UvsManualJoystick>());
    rclcpp::shutdown();
    return 0;
}