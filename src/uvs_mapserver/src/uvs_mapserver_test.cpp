#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "uvs_message/srv/uv_query_element.hpp"
#include "uvs_message/srv/uv_query_map.hpp"
#include "uvs_message/msg/uv_opt_pose_list.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

class UVSMapServerTest : public rclcpp::Node
{
public:
    UVSMapServerTest()
        : Node("uvs_mapserver_test")
    {
        qelement_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        qmap_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        uv_query_element_client_ = create_client<uvs_message::srv::UvQueryElement>("uv_query_element", rmw_qos_profile_services_default, qelement_callback_group_);
        uv_query_map_client_ = create_client<uvs_message::srv::UvQueryMap>("uv_query_map", rmw_qos_profile_services_default, qmap_callback_group_);
        uv_opt_pose_list_pub_ = create_publisher<uvs_message::msg::UvOptPoseList>("uv_opt_pose_list", 1);
        // map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&UVSMapServerTest::timerCallback, this), timer_callback_group_);
        timer2_ = create_wall_timer(std::chrono::milliseconds(3), std::bind(&UVSMapServerTest::timerCallback2, this), timer_callback_group_);
    }
private:
    void timerCallback()
    {
        auto request = std::make_shared<uvs_message::srv::UvQueryMap::Request>();
        request->name = "jxl3028";
        auto result = uv_query_map_client_->async_send_request(request);
        std::future_status status = result.wait_for(1ms);
        if (status == std::future_status::ready)
        {
            auto response = result.get();
            // map_pub_->publish(response->map);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "waiting for response...");
            return;
        }
        
    }
    void timerCallback2()
    {
        uvs_message::msg::UvOptPoseList msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        uvs_message::msg::UvOptPose uv_opt_pose;
        uv_opt_pose.name = "cargo1";
        uv_opt_pose.pose.position.x = 1.0;
        uv_opt_pose.pose.position.y = 1.0;
        uv_opt_pose.pose.position.z = 0.0;
        uv_opt_pose.pose.orientation.x = 0.0;
        uv_opt_pose.pose.orientation.y = 0.0;
        uv_opt_pose.pose.orientation.z = 0.0;
        uv_opt_pose.pose.orientation.w = 1.0;
        msg.pose_list.push_back(uv_opt_pose);

        uv_opt_pose.name = "uv05";
        uv_opt_pose.pose.position.x = -1.0;
        uv_opt_pose.pose.position.y = 1.0;
        uv_opt_pose.pose.position.z = 0.0;
        uv_opt_pose.pose.orientation.x = 0.0;
        uv_opt_pose.pose.orientation.y = 0.0;
        uv_opt_pose.pose.orientation.z = 0.0;
        uv_opt_pose.pose.orientation.w = 1.0;
        msg.pose_list.push_back(uv_opt_pose);

        uv_opt_pose_list_pub_->publish(msg);
    }

    rclcpp::CallbackGroup::SharedPtr qelement_callback_group_;
    rclcpp::CallbackGroup::SharedPtr qmap_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::Client<uvs_message::srv::UvQueryElement>::SharedPtr uv_query_element_client_;
    rclcpp::Client<uvs_message::srv::UvQueryMap>::SharedPtr uv_query_map_client_;
    rclcpp::Publisher<uvs_message::msg::UvOptPoseList>::SharedPtr uv_opt_pose_list_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<UVSMapServerTest>());
    auto node = std::make_shared<UVSMapServerTest>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}