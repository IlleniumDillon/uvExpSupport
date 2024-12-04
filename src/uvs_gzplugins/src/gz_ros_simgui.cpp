#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gui/gui.hh>
#include <gazebo/physics/physics.hh>
#include "gz_ros_simgui.hpp"

gazebo::physics::ModelPtr pmodel = nullptr;

using namespace gazebo;

GzRosSimGui::GzRosSimGui()
{
    // Set the frame background and foreground colors
    this->setStyleSheet(
        "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    // Create the layout that sits inside the frame
    QVBoxLayout *frameLayout = new QVBoxLayout();

    model_name_label_ = new QLabel("Model: ");
    model_pose_label_ = new QLabel("Pose: ");
    model_kinematics_label_ = new QLabel("Kinematics: ");
    model_arm_label_ = new QLabel("Arm: ");
    model_emag_label_ = new QLabel("EMAG: ");

    // Add the label to the frame's layout
    frameLayout->addWidget(model_name_label_);
    frameLayout->addWidget(model_pose_label_);
    frameLayout->addWidget(model_kinematics_label_);
    frameLayout->addWidget(model_arm_label_);
    frameLayout->addWidget(model_emag_label_);

    // Add frameLayout to the frame
    mainFrame->setLayout(frameLayout);

    // Add the frame to the main layout
    mainLayout->addWidget(mainFrame);

    // Remove margins to reduce space
    frameLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    this->setLayout(mainLayout);

    // Position and resize this widget
    this->move(10, 10);
    this->resize(200, 100);

    // Create a node for ROS
    ros_node_ = gazebo_ros::Node::Get();

    // Create a subscriber
    arm_sub_ = ros_node_->create_subscription<uvs_message::msg::GzGui>(
        "gz_gui", 1, std::bind(&GzRosSimGui::callback, this, std::placeholders::_1));
}

GzRosSimGui::~GzRosSimGui()
{
}

void gazebo::GzRosSimGui::callback(const uvs_message::msg::GzGui::SharedPtr msg)
{
    std::stringstream ss;
    ss << "Model: " << msg->name;
    model_name_label_->setText(ss.str().c_str());

    ss.str("");
    ss << std::fixed << std::setprecision(2) << "Pose: (" << msg->pose.x << ", " << msg->pose.y << ", " << msg->pose.theta << ")";
    model_pose_label_->setText(ss.str().c_str());

    ss.str("");
    ss.clear();
    ss << "Kinematics: (" << msg->v << ", " << msg->w << ")";
    model_kinematics_label_->setText(ss.str().c_str());

    ss.str("");
    ss.clear();
    ss << "Arm: (" << msg->arm_arm_rad << ", " << msg->arm_base_rad << ")";
    model_arm_label_->setText(ss.str().c_str());

    ss.str("");
    ss.clear();
    ss << "EMAG: " << (msg->emag_status ? "ON" : "OFF");
    model_emag_label_->setText(ss.str().c_str());
}

GZ_REGISTER_GUI_PLUGIN(GzRosSimGui)
