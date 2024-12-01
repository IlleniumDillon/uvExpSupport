#include "gz_ros_simchassis.hpp"

namespace gazebo_plugins
{

GzRosSimChassis::GzRosSimChassis(){}

GzRosSimChassis::~GzRosSimChassis(){}

void GzRosSimChassis::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    model_ = model;
    world_ = model_->GetWorld();
    ros_node_ = gazebo_ros::Node::Get(sdf);
    const gazebo_ros::QoS qos = ros_node_->get_qos();

    status_topic_ = sdf->Get<std::string>("status_topic", "uvs_emb_status").first;
    arm_topic_ = sdf->Get<std::string>("arm_topic", "uvs_emb_arm").first;
    emag_topic_ = sdf->Get<std::string>("emag_topic", "uvs_emb_emag").first;
    kinetics_topic_ = sdf->Get<std::string>("kinetics_topic", "uvs_emb_kinetics").first;
    status_pub_period_ = sdf->Get<double>("status_pub_period", 0.05).first;
    limit_velocity_ = sdf->Get<double>("limit_velocity", 0.5).first;
    limit_anglevelocity_ = sdf->Get<double>("limit_anglevelocity", M_PI).first;
    wheel_separation_ = sdf->Get<double>("wheel_separation", 0.238).first;

    status_pub_ = ros_node_->create_publisher<uvs_message::msg::UvEmbStatus>(status_topic_, qos.get_publisher_qos(status_topic_, rclcpp::SensorDataQoS()));
    arm_sub_ = ros_node_->create_subscription<uvs_message::msg::UvEmbArm>(arm_topic_, qos.get_subscription_qos(arm_topic_, rclcpp::SensorDataQoS()), std::bind(&GzRosSimChassis::arm_callback, this, std::placeholders::_1));
    emag_sub_ = ros_node_->create_subscription<uvs_message::msg::UvEmbEmag>(emag_topic_, qos.get_subscription_qos(emag_topic_, rclcpp::SensorDataQoS()), std::bind(&GzRosSimChassis::emag_callback, this, std::placeholders::_1));
    kinetics_sub_ = ros_node_->create_subscription<uvs_message::msg::UvEmbKinetics>(kinetics_topic_, qos.get_subscription_qos(kinetics_topic_, rclcpp::SensorDataQoS()), std::bind(&GzRosSimChassis::kinetics_callback, this, std::placeholders::_1));

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GzRosSimChassis::OnUpdate, this, std::placeholders::_1));

    last_world_update_time_ = world_->SimTime();
}

void GzRosSimChassis::Reset()
{
    last_world_update_time_ = world_->SimTime();
    kinetics_msg_.v = 0.0;
    kinetics_msg_.w = 0.0;
}

void GzRosSimChassis::OnUpdate(const gazebo::common::UpdateInfo &_info)
{
    gazebo::common::Time current_time = world_->SimTime();
    double dt = (current_time - last_world_update_time_).Double();
    ignition::math::Pose3d pose = model_->WorldPose();
    ignition::math::Vector3d linear_velocity = model_->WorldLinearVel();
    ignition::math::Vector3d angular_velocity = model_->WorldAngularVel();

    double v_cur = sqrt(pow(linear_velocity.X(), 2) + pow(linear_velocity.Y(), 2));
    double w_cur = angular_velocity.Z();
    double theta_cur = pose.Rot().Yaw();
    double v_set, w_set;
    ignition::math::Vector3d linear_v_set, angular_v_set;

    if (dt < status_pub_period_)
    {
        return;
    }

    last_world_update_time_ = current_time;

    // Publish status
    status_msg_.left_wheel_speed = v_cur - w_cur * wheel_separation_ / 2;
    status_msg_.right_wheel_speed = v_cur + w_cur * wheel_separation_ / 2;
    status_msg_.rotation_z = theta_cur;
    status_msg_.voltage = 12.0;
    /// TODO: status of arm

    status_pub_->publish(status_msg_);

    // Update kinetics
    if (mutex_kinetics_.try_lock_for(std::chrono::milliseconds(1)))
    {
        v_set = kinetics_msg_.v;
        w_set = kinetics_msg_.w;
        mutex_kinetics_.unlock();

        if (v_set > limit_velocity_)
        {
            v_set = limit_velocity_;
        }
        if (v_set < -limit_velocity_)
        {
            v_set = -limit_velocity_;
        }
        if (w_set > limit_anglevelocity_)
        {
            w_set = limit_anglevelocity_;
        }
        if (w_set < -limit_anglevelocity_)
        {
            w_set = -limit_anglevelocity_;
        }

        linear_v_set.X() = v_set * cos(theta_cur);
        linear_v_set.Y() = v_set * sin(theta_cur);
        linear_v_set.Z() = 0.0;
        angular_v_set.X() = 0.0;
        angular_v_set.Y() = 0.0;
        angular_v_set.Z() = w_set;

        model_->SetLinearVel(linear_v_set);
        model_->SetAngularVel(angular_v_set);
    }

    /// TODO: Update arm
    if (mutex_arm_.try_lock_for(std::chrono::milliseconds(1)))
    {
        mutex_arm_.unlock();
    }

    /// TODO: Update emag
    if (mutex_emag_.try_lock_for(std::chrono::milliseconds(1)))
    {
        mutex_emag_.unlock();
    }
    
}

GZ_REGISTER_MODEL_PLUGIN (GzRosSimChassis)

}   // namespace gazebo_plugins