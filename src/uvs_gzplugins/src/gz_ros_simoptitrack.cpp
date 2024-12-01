#include "gz_ros_simoptitrack.hpp"

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
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{

class GzRosSimOptitrackPrivate
{

};

GzRosSimOptitrack::GzRosSimOptitrack() : impl_(std::make_unique<GzRosSimOptitrackPrivate>())
{
}

GzRosSimOptitrack::~GzRosSimOptitrack()
{

}

void GzRosSimOptitrack::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{

}

GZ_REGISTER_WORLD_PLUGIN(GzRosSimOptitrack)

}   // namespace gazebo_plugins