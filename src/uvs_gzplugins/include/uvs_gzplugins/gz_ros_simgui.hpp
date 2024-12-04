#ifndef UVS_GZPLUGINS_GZ_ROS_SIMGUI_HPP
#define UVS_GZPLUGINS_GZ_ROS_SIMGUI_HPP

#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
// moc parsing error of tbb headers
#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#endif
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>

#include <QTimerEvent>

// #include <rclcpp/rclcpp.hpp>
// #include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include "uvs_message/msg/gz_gui.hpp"

namespace gazebo
{

class GAZEBO_VISIBLE GzRosSimGui : public gazebo::GUIPlugin
{
Q_OBJECT
public:
    GzRosSimGui();
    ~GzRosSimGui();
    void callback(const uvs_message::msg::GzGui::SharedPtr msg);
private:
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<uvs_message::msg::GzGui>::SharedPtr arm_sub_;

    std::string model_name_;
    gazebo::physics::ModelPtr model_;

    QLabel *model_name_label_;
    QLabel *model_pose_label_;
    QLabel *model_kinematics_label_;
    QLabel *model_arm_label_;
    QLabel *model_emag_label_;

    int timer_id_;
};

}   // namespace gazebo_plugins

#endif // UVS_GZPLUGINS_GZ_ROS_SIMGUI_HPP