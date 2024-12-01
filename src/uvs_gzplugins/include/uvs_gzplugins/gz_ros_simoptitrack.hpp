#ifndef UVS_GZPLUGINS_GZ_ROS_SIMOPTITRACK_HPP
#define UVS_GZPLUGINS_GZ_ROS_SIMOPTITRACK_HPP

#include <gazebo/common/Plugin.hh>
#include <memory>

namespace gazebo_plugins
{
class GzRosSimOptitrackPrivate;

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
    /// Private data pointer
    std::unique_ptr<GzRosSimOptitrackPrivate> impl_;
};
}   // namespace gazebo_plugins

#endif // UVS_GZPLUGINS_GZ_ROS_SIMOPTITRACK_HPP