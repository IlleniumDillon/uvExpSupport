#include "gz_ros_simchassis.hpp"
#include <gazebo/physics/physics.hh>

#define arm_base_k (-0.001503759)
#define arm_base_b (0.463157895)
#define arm_arm_k (-0.001367742)
#define arm_arm_b (5.0126)

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
    arm_base_joint_name_ = sdf->Get<std::string>("arm_base_joint_name", "arm_base_joint").first;
    arm_arm_joint_name_ = sdf->Get<std::string>("arm_arm_joint_name", "arm_arm_joint").first;
    sdf::ElementPtr pmodelName = sdf->GetElement("model");
    while (pmodelName != nullptr)
    {
        gazebo::physics::ModelPtr model = world_->ModelByName(pmodelName->Get<std::string>());
        if (model)
        {
            dynamic_models_.push_back(model);
        }
        pmodelName = pmodelName->GetNextElement("model");
    }
    emag_link_name_ = sdf->Get<std::string>("emag_link_name", "emag_link").first;

    arm_base_joint_ = model_->GetJoint(arm_base_joint_name_);
    if (arm_base_joint_ == nullptr)
    {
        RCLCPP_ERROR(ros_node_->get_logger(), "Joint %s not found", arm_base_joint_name_.c_str());
    }
    arm_arm_joint_ = model_->GetJoint(arm_arm_joint_name_);
    if (arm_arm_joint_ == nullptr)
    {
        RCLCPP_ERROR(ros_node_->get_logger(), "Joint %s not found", arm_arm_joint_name_.c_str());
    }

    status_pub_ = ros_node_->create_publisher<uvs_message::msg::UvEmbStatus>(status_topic_, 1);
    arm_sub_ = ros_node_->create_subscription<uvs_message::msg::UvEmbArm>(arm_topic_, 1, std::bind(&GzRosSimChassis::arm_callback, this, std::placeholders::_1));
    emag_sub_ = ros_node_->create_subscription<uvs_message::msg::UvEmbEmag>(emag_topic_, 1, std::bind(&GzRosSimChassis::emag_callback, this, std::placeholders::_1));
    kinetics_sub_ = ros_node_->create_subscription<uvs_message::msg::UvEmbKinetics>(kinetics_topic_, 1, std::bind(&GzRosSimChassis::kinetics_callback, this, std::placeholders::_1));
    gui_pub_ = ros_node_->create_publisher<uvs_message::msg::GzGui>("gz_gui", 1);

    // ros_node_->declare_parameter("vp", 1.0);
    // ros_node_->declare_parameter("vi", 5.0);
    // ros_node_->declare_parameter("vd", 0.0);
    // ros_node_->declare_parameter("v_max_i", 0.5);
    // ros_node_->declare_parameter("v_max_o", 1.5); 

    // ros_node_->declare_parameter("wp", 1.0);
    // ros_node_->declare_parameter("wi", 15.0);
    // ros_node_->declare_parameter("wd", 0.0);
    // ros_node_->declare_parameter("w_max_i", 0.75);
    // ros_node_->declare_parameter("w_max_o", 6.0);   

    arm_msg_.arm_arm = 3855;
    arm_msg_.arm_base = 308;
    // arm_msg_.arm_arm = 3080;
    // arm_msg_.arm_base = 840;
    // ctrl_v_.setParams(0, 40, 0.0, status_pub_period_, 1.0, 1.5);
    // ctrl_w_.setParams(0, 40, 0.0, status_pub_period_, 3.0, 6);

    gznode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gznode_->Init();
    gzsub_ = gznode_->Subscribe("~/physics/contacts", &GzRosSimChassis::gz_contacts_callback, this);

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GzRosSimChassis::OnUpdate, this, std::placeholders::_1));

    last_world_update_time_ = world_->SimTime();
    last_status_pub_time_ = world_->SimTime();
}

void GzRosSimChassis::Reset()
{
    last_world_update_time_ = world_->SimTime();
    last_status_pub_time_ = world_->SimTime();
    kinetics_msg_.v = 0.0;
    kinetics_msg_.w = 0.0;
}

void GzRosSimChassis::OnUpdate(const gazebo::common::UpdateInfo &_info)
{
    gazebo::common::Time current_time = world_->SimTime();
    double dt = (current_time - last_world_update_time_).Double();
    last_world_update_time_ = current_time;
    ignition::math::Pose3d pose = model_->WorldPose();
    ignition::math::Vector3d linear_velocity = model_->WorldLinearVel();
    ignition::math::Vector3d angular_velocity = model_->WorldAngularVel();

    double theta_cur = pose.Rot().Yaw();
    ignition::math::Vector3d linear_velocity_cur = pose.Rot().RotateVectorReverse(linear_velocity);
    double v_cur = linear_velocity_cur.X();
    double w_cur = angular_velocity.Z();
    static double v_set, w_set;
    ignition::math::Vector3d linear_v_set, angular_v_set;

    static double arm_arm_set, arm_base_set;
    double arm_arm_cur = arm_arm_joint_->Position(0);
    double arm_base_cur = arm_base_joint_->Position(0);
    double arm_arm, arm_base;

    bool emag_flag;

    uvs_message::msg::GzGui gui_msg;


    // set velocity
    linear_v_set.X() = v_set * cos(theta_cur);
    linear_v_set.Y() = v_set * sin(theta_cur);
    linear_v_set.Z() = 0.0;
    angular_v_set.X() = 0.0;
    angular_v_set.Y() = 0.0;
    angular_v_set.Z() = w_set;

    model_->SetLinearVel(linear_v_set);
    model_->SetAngularVel(angular_v_set);

    // set arm
    if (std::abs(arm_arm_set - arm_arm_cur) > arm_arm_joint_->GetVelocityLimit(0) * dt)
    {
        if (arm_arm_set > arm_arm_cur)
        {
            arm_arm = arm_arm_cur + arm_arm_joint_->GetVelocityLimit(0) * dt;
        }
        else
        {
            arm_arm = arm_arm_cur - arm_arm_joint_->GetVelocityLimit(0) * dt;
        }
    }
    else
    {
        arm_arm = arm_arm_set;
    }

    if (std::abs(arm_base_set - arm_base_cur) > arm_base_joint_->GetVelocityLimit(0) * dt)
    {
        if (arm_base_set > arm_base_cur)
        {
            arm_base = arm_base_cur + arm_base_joint_->GetVelocityLimit(0) * dt;
        }
        else
        {
            arm_base = arm_base_cur - arm_base_joint_->GetVelocityLimit(0) * dt;
        }
    }
    else
    {
        arm_base = arm_base_set;
    }
    arm_arm_joint_->SetPosition(0, arm_arm);
    arm_base_joint_->SetPosition(0, arm_base);


    dt = (current_time - last_status_pub_time_).Double();
    if (dt < status_pub_period_)
    {
        return;
    }
    last_world_update_time_ = current_time;

    // Publish status
    status_msg_.left_wheel_speed = v_cur - w_cur * wheel_separation_ / 2;
    status_msg_.right_wheel_speed = v_cur + w_cur * wheel_separation_ / 2;
    status_msg_.rotation_z = theta_cur;
    status_msg_.arm_arm_pos = (arm_arm_cur - arm_arm_b) / arm_arm_k;
    status_msg_.arm_base_pos = (arm_base_cur - arm_base_b) / arm_base_k;
    status_msg_.voltage = 12.0;

    status_pub_->publish(status_msg_);

    gui_msg.name = model_->GetName();
    gui_msg.pose.x = pose.Pos().X();
    gui_msg.pose.y = pose.Pos().Y();
    gui_msg.pose.theta = pose.Rot().Yaw();
    gui_msg.v = v_cur;
    gui_msg.w = w_cur;

    // Update kinetics target
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
    }

    /// Update arm target
    if (mutex_arm_.try_lock_for(std::chrono::milliseconds(1)))
    {
        arm_arm_set = arm_msg_.arm_arm * arm_arm_k + arm_arm_b;
        arm_base_set = arm_msg_.arm_base * arm_base_k + arm_base_b;
        mutex_arm_.unlock();

        // arm_base_joint_->SetPosition(0, 3);
        // arm_arm_joint_ ->SetPosition(0, 1);
        // arm_arm_joint_->SetPosition(0, arm_arm * arm_arm_k + arm_arm_b);
        // arm_base_joint_->SetPosition(0, arm_base * arm_base_k + arm_base_b);
        gui_msg.arm_arm_rad = arm_arm_cur;
        gui_msg.arm_base_rad = arm_base_cur;
    }

    /// TODO: Update emag
    if (mutex_emag_.try_lock_for(std::chrono::milliseconds(1)))
    {
        emag_flag = emag_msg_.enable;
        gui_msg.emag_status = emag_flag;
        mutex_emag_.unlock();
        if (emag_flag)
        {
            if (union_model_ == nullptr)
            {
                auto emag_link = model_->GetLink(emag_link_name_);
                auto emag_collision_name = emag_link->GetCollisions()[0]->GetScopedName();
                mutex_contacts_.lock();
                for (int i = 0; i < contacts_msg_.contact_size(); i++)
                {
                    auto contact = contacts_msg_.contact(i);
                    if (!(contact.has_collision1() && contact.has_collision2())) continue;
                    if (contact.collision1() == emag_collision_name || contact.collision2() == emag_collision_name)
                    {
                        for (auto model : dynamic_models_)
                        {
                            auto model_link = model->GetLink("link_0");
                            if (model_link == nullptr)
                            {
                                continue;
                            }
                            auto model_collision_name = model_link->GetCollisions()[0]->GetScopedName();
                            if (contact.collision1() == model_collision_name || contact.collision2() == model_collision_name)
                            {
                                union_model_ = model;
                                union_model_->CreateJoint("emag_joint", "fixed", emag_link, model_link);
                                break;
                            }
                        }
                        if (union_model_ != nullptr)
                        {
                            break;
                        }
                    }
                }
                mutex_contacts_.unlock();
            }
        }
        else
        {
            if (union_model_!=nullptr)
            {
                union_model_->RemoveJoint("emag_joint");
                union_model_ = nullptr;
            }
        }
    }
    gui_pub_->publish(gui_msg);
}

void GzRosSimChassis::gz_contacts_callback(ConstContactsPtr &msg)
{
    if (mutex_contacts_.try_lock_for(std::chrono::milliseconds(0)))
    {
        contacts_msg_.CopyFrom(*msg);
        mutex_contacts_.unlock();
    }
}

GZ_REGISTER_MODEL_PLUGIN (GzRosSimChassis)

}   // namespace gazebo_plugins