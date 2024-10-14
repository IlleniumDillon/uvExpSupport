#include "rclcpp/rclcpp.hpp"
#include "uvs_manual_keyboard.hpp"

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UvsManualKeyboard>());
    rclcpp::shutdown();
    return 0;
}