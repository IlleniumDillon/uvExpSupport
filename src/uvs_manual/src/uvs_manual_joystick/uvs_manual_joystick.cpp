#include "uvs_manual_joystick.hpp"

UvsManualJoystick::UvsManualJoystick()
    : Node("uvs_manual_joystick")
{
    joystick = std::make_shared<Joystick>();

    arm_publisher = this->create_publisher<uvs_message::msg::UvEmbArm>("uvs_emb_arm", 1);
    emag_publisher = this->create_publisher<uvs_message::msg::UvEmbEmag>("uvs_emb_emag", 1);
    kinetics_publisher = this->create_publisher<uvs_message::msg::UvEmbKinetics>("uvs_emb_kinetics", 1);
    status_subscriber = this->create_subscription<uvs_message::msg::UvEmbStatus>("uvs_emb_status", 1, std::bind(&UvsManualJoystick::status_callback, this, std::placeholders::_1));

    if (!joystick->isFound())
    {
        printf("open failed.\n");
        return;
    }

    timer = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&UvsManualJoystick::timer_callback, this));
}

UvsManualJoystick::~UvsManualJoystick()
{
}

void UvsManualJoystick::status_callback(const uvs_message::msg::UvEmbStatus::SharedPtr msg)
{
    status_msg = *msg;
    getFeedback = true;
}

void UvsManualJoystick::timer_callback()
{
    if (!getFeedback)
    {
        RCLCPP_INFO(get_logger(), "No feedback from embedded system.");
        return;
    }
    bool update = false;
    while (joystick->sample(&joystickEvent)) {update = true;}
    if (update)
    {
        // linear speed
        kinetics_msg.v = -LINEAR_SPEED * (float)joystickEvent.axis_r_ud / JOYSTICK_AXIS_MAX;
        // angular speed
        kinetics_msg.w = -ANGULAR_SPEED * (float)joystickEvent.axis_r_lr / JOYSTICK_AXIS_MAX;
        // arm base
        int delta_base = -joystickEvent.axis_cross_ud / JOYSTICK_AXIS_MAX;
        uint16_t arm_base = status_msg.arm_base_pos + delta_base;
        if (arm_base < ARM_ANGLE_MIN)
        {
            arm_base = ARM_ANGLE_MIN;
        }
        else if (arm_base > ARM_ANGLE_MAX)
        {
            arm_base = ARM_ANGLE_MAX;
        }
        arm_msg.arm_base = arm_base;
        // arm arm
        int delta_arm = joystickEvent.button_y - joystickEvent.button_a;
        uint16_t arm_arm = status_msg.arm_arm_pos + delta_arm;
        if (arm_arm < ARM_ANGLE_MIN)
        {
            arm_arm = ARM_ANGLE_MIN;
        }
        else if (arm_arm > ARM_ANGLE_MAX)
        {
            arm_arm = ARM_ANGLE_MAX;
        }
        arm_msg.arm_arm = arm_arm;
        // emag
        if (joystickEvent.axis_r2 == JOYSTICK_AXIS_MIN)
        {
            emag_msg.enable = 0;
        }
        else
        {
            emag_msg.enable = 1;
        }

        arm_publisher->publish(arm_msg);
        emag_publisher->publish(emag_msg);
        kinetics_publisher->publish(kinetics_msg);
    }
}
