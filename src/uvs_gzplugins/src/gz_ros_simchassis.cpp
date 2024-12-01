#include "gz_ros_simchassis.hpp"

namespace gazebo_plugins
{

GzRosSimChassis::GzRosSimChassis() : impl_(std::make_unique<GzRosSimChassisPrivate>())
{
}

GzRosSimChassis::~GzRosSimChassis()
{

}

void GzRosSimChassis::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{

}

GZ_REGISTER_WORLD_PLUGIN(GzRosSimChassis)

}   // namespace gazebo_plugins