#ifndef UVS_GZPLUGINS_GZ_ROS_SIMCHASSIS_HPP
#define UVS_GZPLUGINS_GZ_ROS_SIMCHASSIS_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "uvs_message/msg/uv_emb_status.hpp"
#include "uvs_message/msg/uv_emb_arm.hpp"
#include "uvs_message/msg/uv_emb_emag.hpp"
#include "uvs_message/msg/uv_emb_kinetics.hpp"

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
class GzRosSimChassisPrivate
{
public:

public:
    gazebo_ros::Node::SharedPtr ros_node_;
    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::common::Time last_update_time_;
    double update_period_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    gazebo::physics::ModelPtr pagent = nullptr;

    uvs_message::msg::UvEmbStatus status_msg_;
    uvs_message::msg::UvEmbArm arm_msg_;
    uvs_message::msg::UvEmbEmag emag_msg_;
    uvs_message::msg::UvEmbKinetics kinetics_msg_;
};

class GzRosSimChassis : public gazebo::WorldPlugin
{
public:
    /// Constructor
    GzRosSimChassis();

    /// Destructor
    ~GzRosSimChassis();

protected:
    // Documentation inherited
    void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override;

private:
    /// Private data pointer
    std::unique_ptr<GzRosSimChassisPrivate> impl_;
};
}   // namespace gazebo_plugins

#endif // UVS_GZPLUGINS_GZ_ROS_SIMCHASSIS_HPP