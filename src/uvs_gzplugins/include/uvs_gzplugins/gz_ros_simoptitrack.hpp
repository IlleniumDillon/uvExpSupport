#ifndef UVS_GZPLUGINS_GZ_ROS_SIMOPTITRACK_HPP
#define UVS_GZPLUGINS_GZ_ROS_SIMOPTITRACK_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "uvs_message/msg/uv_opt_pose_list.hpp"

#include <gazebo/common/Plugin.hh>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

namespace gazebo_plugins
{

class GzRosSimOptitrack : public gazebo::WorldPlugin
{
public:
    /// Constructor
    GzRosSimOptitrack();

    /// Destructor
    ~GzRosSimOptitrack();

protected:
    // Documentation inherited
    void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override;

    void Reset() override;

    void OnUpdate(const gazebo::common::UpdateInfo & _info);

private:
    gazebo_ros::Node::SharedPtr ros_node_;
    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr update_connection_;
    std::vector<gazebo::physics::ModelPtr> models_;

    rclcpp::Publisher<uvs_message::msg::UvOptPoseList>::SharedPtr uv_opt_pose_list_pub_;
    std::string uv_opt_pose_list_topic_;
    uvs_message::msg::UvOptPoseList uv_opt_pose_list_msg_;

    double publish_period_;
    gazebo::common::Time last_publish_time_;
};
}   // namespace gazebo_plugins

#endif // UVS_GZPLUGINS_GZ_ROS_SIMOPTITRACK_HPP