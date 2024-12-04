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
#include "uvs_message/msg/gz_gui.hpp"

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
class GzRosSimChassis : public gazebo::ModelPlugin
{

public:
    /// Constructor
    GzRosSimChassis();

    /// Destructor
    ~GzRosSimChassis();

protected:
    // Documentation inherited
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

    void Reset() override;

    void OnUpdate(const gazebo::common::UpdateInfo & _info);


    void arm_callback(const uvs_message::msg::UvEmbArm::SharedPtr msg)
    {
        if (mutex_arm_.try_lock_for(std::chrono::milliseconds(1)))
        {
            arm_msg_ = *msg;
            mutex_arm_.unlock();
        }
    }
    void emag_callback(const uvs_message::msg::UvEmbEmag::SharedPtr msg)
    {
        if (mutex_emag_.try_lock_for(std::chrono::milliseconds(1)))
        {
            emag_msg_ = *msg;
            mutex_emag_.unlock();
        }
    }
    void kinetics_callback(const uvs_message::msg::UvEmbKinetics::SharedPtr msg)
    {
        if (mutex_kinetics_.try_lock_for(std::chrono::milliseconds(1)))
        {
            kinetics_msg_ = *msg;
            mutex_kinetics_.unlock();
        }
    }
private:
    gazebo_ros::Node::SharedPtr ros_node_;
    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr update_connection_;

    rclcpp::Publisher<uvs_message::msg::UvEmbStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<uvs_message::msg::UvEmbArm>::SharedPtr arm_sub_;
    rclcpp::Subscription<uvs_message::msg::UvEmbEmag>::SharedPtr emag_sub_;
    rclcpp::Subscription<uvs_message::msg::UvEmbKinetics>::SharedPtr kinetics_sub_;
    rclcpp::Publisher<uvs_message::msg::GzGui>::SharedPtr gui_pub_;

    // std::timed_mutex mutex_status_;
    uvs_message::msg::UvEmbStatus status_msg_;
    std::timed_mutex mutex_arm_;
    uvs_message::msg::UvEmbArm arm_msg_;
    std::timed_mutex mutex_emag_;
    uvs_message::msg::UvEmbEmag emag_msg_;
    std::timed_mutex mutex_kinetics_;
    uvs_message::msg::UvEmbKinetics kinetics_msg_;

    std::string status_topic_;
    double status_pub_period_;
    std::string arm_topic_;
    std::string emag_topic_;
    std::string kinetics_topic_;
    double limit_velocity_;
    double limit_anglevelocity_;
    double wheel_separation_;

    std::string arm_base_joint_name_;
    std::string arm_arm_joint_name_;

    gazebo::physics::JointPtr arm_base_joint_;
    gazebo::physics::JointPtr arm_arm_joint_;

    std::vector<gazebo::physics::ModelPtr> dynamic_models_;
    gazebo::physics::ModelPtr union_model_ = nullptr;
    std::string emag_link_name_;

    gazebo::common::Time last_world_update_time_;
};
}   // namespace gazebo_plugins

#endif // UVS_GZPLUGINS_GZ_ROS_SIMCHASSIS_HPP