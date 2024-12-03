#include "gz_ros_simoptitrack.hpp"

namespace gazebo_plugins
{

GzRosSimOptitrack::GzRosSimOptitrack(){}

GzRosSimOptitrack::~GzRosSimOptitrack(){}

void GzRosSimOptitrack::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{
    world_ = world;
    ros_node_ = gazebo_ros::Node::Get(sdf);
    const gazebo_ros::QoS qos = ros_node_->get_qos();

    uv_opt_pose_list_topic_ = sdf->Get<std::string>("topic", "uvs_opt_pose_list").first;
    publish_period_ = sdf->Get<double>("publish_period", 0.05).first;
    sdf::ElementPtr pmodelName = sdf->GetElement("model");
    while (pmodelName != nullptr)
    {
        gazebo::physics::ModelPtr model = world_->ModelByName(pmodelName->Get<std::string>());
        if (model)
        {
            models_.push_back(model);
        }
        pmodelName = pmodelName->GetNextElement("model");
    }

    uv_opt_pose_list_pub_ = ros_node_->create_publisher<uvs_message::msg::UvOptPoseList>(uv_opt_pose_list_topic_, 1);

    last_publish_time_ = world_->SimTime();

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GzRosSimOptitrack::OnUpdate, this, std::placeholders::_1));
}

void GzRosSimOptitrack::Reset()
{
    last_publish_time_ = world_->SimTime();
}

void GzRosSimOptitrack::OnUpdate(const gazebo::common::UpdateInfo &_info)
{
    gazebo::common::Time current_time = world_->SimTime();
    double dt = (current_time - last_publish_time_).Double();

    if (dt < publish_period_)
    {
        return;
    }
    // RCLCPP_INFO(ros_node_->get_logger(), "Publishing UV Optitrack Pose List");
    uv_opt_pose_list_msg_.header.stamp = ros_node_->now();
    uv_opt_pose_list_msg_.header.frame_id = "world";
    for (auto model : models_)
    {
        uvs_message::msg::UvOptPose temp;
        temp.name = model->GetName();
        temp.pose.position.x = model->WorldPose().Pos().X();
        temp.pose.position.y = model->WorldPose().Pos().Y();
        temp.pose.position.z = model->WorldPose().Pos().Z();
        temp.pose.orientation.x = model->WorldPose().Rot().X();
        temp.pose.orientation.y = model->WorldPose().Rot().Y();
        temp.pose.orientation.z = model->WorldPose().Rot().Z();
        temp.pose.orientation.w = model->WorldPose().Rot().W();
        uv_opt_pose_list_msg_.pose_list.push_back(temp);
    }

    uv_opt_pose_list_pub_->publish(uv_opt_pose_list_msg_);

    uv_opt_pose_list_msg_.pose_list.clear();
    last_publish_time_ = current_time;
}

GZ_REGISTER_WORLD_PLUGIN(GzRosSimOptitrack)

}   // namespace gazebo_plugins