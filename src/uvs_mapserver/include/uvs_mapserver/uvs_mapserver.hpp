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

    void loadWorld();

private:
    void uvQueryElementCallback(
        const std::shared_ptr<uvs_message::srv::UvQueryElement::Request> request,
        std::shared_ptr<uvs_message::srv::UvQueryElement::Response> response);

    void uvQueryMapCallback(
        const std::shared_ptr<uvs_message::srv::UvQueryMap::Request> request,
        std::shared_ptr<uvs_message::srv::UvQueryMap::Response> response);

    void uvOptPoseListCallback(const uvs_message::msg::UvOptPoseList::SharedPtr msg);

private:
    rclcpp::Service<uvs_message::srv::UvQueryElement>::SharedPtr uv_query_element_service_;
    rclcpp::Service<uvs_message::srv::UvQueryMap>::SharedPtr uv_query_map_service_;
    rclcpp::Subscription<uvs_message::msg::UvOptPoseList>::SharedPtr uv_opt_pose_list_sub_;

    nav_msgs::msg::OccupancyGrid map_;
    std::map<std::string, geometry_msgs::msg::Pose> uv_opt_poses_;
};

#endif // UVS_MAPSERVER_HPP