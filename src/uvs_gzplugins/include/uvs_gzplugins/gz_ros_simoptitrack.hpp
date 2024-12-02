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

private:
};
}   // namespace gazebo_plugins

#endif // UVS_GZPLUGINS_GZ_ROS_SIMOPTITRACK_HPP