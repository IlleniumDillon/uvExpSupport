#ifndef UVS_MAPSERVER_HPP
#define UVS_MAPSERVER_HPP

#include <rclcpp/rclcpp.hpp>

#include "uvs_message/srv/uv_query_world.hpp"
#include "uvs_message/srv/uv_query_element.hpp"
#include "uvs_message/srv/uv_query_map.hpp"
#include "uvs_message/msg/uv_opt_pose_list.hpp"

#include "uvs_tools/load_world.hpp"

class UVSMapServer : public rclcpp::Node
{
public:
    UVSMapServer();
    ~UVSMapServer();

private:
    void uvQueryWorldCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<uvs_message::srv::UvQueryWorld::Request> request,
        std::shared_ptr<uvs_message::srv::UvQueryWorld::Response> response);
        
    void uvQueryElementCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<uvs_message::srv::UvQueryElement::Request> request,
        std::shared_ptr<uvs_message::srv::UvQueryElement::Response> response);

    void uvQueryMapCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<uvs_message::srv::UvQueryMap::Request> request,
        std::shared_ptr<uvs_message::srv::UvQueryMap::Response> response);

    void uvOptPoseListCallback(const uvs_message::msg::UvOptPoseList::SharedPtr msg);

    void timerCallback();

    std::vector<Point2I> getFootprint(const Polygon2D &shape, const double& resolutionX, const double& resolutionY, const Point2D &origin);

private:
    rclcpp::Service<uvs_message::srv::UvQueryElement>::SharedPtr uv_query_element_service_;
    rclcpp::Service<uvs_message::srv::UvQueryMap>::SharedPtr uv_query_map_service_;
    rclcpp::Service<uvs_message::srv::UvQueryWorld>::SharedPtr uv_query_world_service_;
    rclcpp::Subscription<uvs_message::msg::UvOptPoseList>::SharedPtr uv_opt_pose_list_sub_;
    /// test
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_static_pub_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    /// 
    nav_msgs::msg::OccupancyGrid map_static_;
    std::map<std::string, geometry_msgs::msg::Pose> uv_opt_poses_;

    DSCP_World world_;

    // std::map<std::string, std::vector<Point2I>> cargo_footprints_;
    // std::map<std::string, std::vector<Point2I>> cargo_shape_;
    std::map<std::string,
        std::pair<Polygon2D, std::vector<Point2I>>> cargo_shape_;
};

#endif // UVS_MAPSERVER_HPP