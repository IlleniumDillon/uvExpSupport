#ifndef UVS_MANUAL_JOYSTICK_HPP
#define UVS_MANUAL_JOYSTICK_HPP

#include "rclcpp/rclcpp.hpp"

#include "uvs_message/msg/uv_emb_arm.hpp"
#include "uvs_message/msg/uv_emb_emag.hpp"
#include "uvs_message/msg/uv_emb_kinetics.hpp"
#include "uvs_message/msg/uv_emb_status.hpp"

class UvsManualJoystick : public rclcpp::Node
{
public:
    UvsManualJoystick();
    ~UvsManualJoystick();

private:
    void timer_callback();
    void status_callback(const uvs_message::msg::UvEmbStatus::SharedPtr msg);

    rclcpp::Publisher<uvs_message::msg::UvEmbArm>::SharedPtr arm_publisher;
    rclcpp::Publisher<uvs_message::msg::UvEmbEmag>::SharedPtr emag_publisher;
    rclcpp::Publisher<uvs_message::msg::UvEmbKinetics>::SharedPtr kinetics_publisher;
    rclcpp::Subscription<uvs_message::msg::UvEmbStatus>::SharedPtr status_subscriber;
    uvs_message::msg::UvEmbArm arm_msg;
    uvs_message::msg::UvEmbEmag emag_msg;
    uvs_message::msg::UvEmbKinetics kinetics_msg;
    uvs_message::msg::UvEmbStatus status_msg;
    rclcpp::TimerBase::SharedPtr timer;
};

#endif // UVS_MANUAL_JOYSTICK_HPP