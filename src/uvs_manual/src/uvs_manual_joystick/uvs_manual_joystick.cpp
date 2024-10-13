#include "uvs_manual_joystick.hpp"

UvsManualJoystick::UvsManualJoystick()
    : Node("uvs_manual_joystick")
{
    arm_publisher = this->create_publisher<uvs_message::msg::UvEmbArm>("uvs_emb_arm", 1);
    emag_publisher = this->create_publisher<uvs_message::msg::UvEmbEmag>("uvs_emb_emag", 1);
    kinetics_publisher = this->create_publisher<uvs_message::msg::UvEmbKinetics>("uvs_emb_kinetics", 1);
    status_subscriber = this->create_subscription<uvs_message::msg::UvEmbStatus>("uvs_emb_status", 1, std::bind(&UvsManualJoystick::status_callback, this, std::placeholders::_1));
    timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&UvsManualJoystick::timer_callback, this));
}

UvsManualJoystick::~UvsManualJoystick()
{
}

void UvsManualJoystick::timer_callback()
{
}

void UvsManualJoystick::status_callback(const uvs_message::msg::UvEmbStatus::SharedPtr msg)
{
    status_msg = *msg;
}
