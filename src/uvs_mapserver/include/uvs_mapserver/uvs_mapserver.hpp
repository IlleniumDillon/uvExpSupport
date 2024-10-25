#ifndef UVS_MAPSERVER_HPP
#define UVS_MAPSERVER_HPP

#include <rclcpp/rclcpp.hpp>

#include "uvs_message/srv/uv_query_element.hpp"
#include "uvs_message/srv/uv_query_map.hpp"
#include "uvs_message/msg/uv_opt_pose_list.hpp"

class UVSMapServer : public rclcpp::Node
{
public:
    UVSMapServer();
    ~UVSMapServer();
};

#endif // UVS_MAPSERVER_HPP