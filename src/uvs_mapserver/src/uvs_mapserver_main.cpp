#include <rclcpp/rclcpp.hpp>
#include "uvs_mapserver.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UVSMapServer>());
  rclcpp::shutdown();
  return 0;
}