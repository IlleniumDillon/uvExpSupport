#include "uvs_manual_keyboard.hpp"

UvsManualKeyboard::UvsManualKeyboard()
    : Node("uvs_manual_keyboard")
{
    keyboard = std::make_shared<Keyboard>();

    arm_publisher = this->create_publisher<uvs_message::msg::UvEmbArm>("uvs_emb_arm", 1);
    emag_publisher = this->create_publisher<uvs_message::msg::UvEmbEmag>("uvs_emb_emag", 1);
    kinetics_publisher = this->create_publisher<uvs_message::msg::UvEmbKinetics>("uvs_emb_kinetics", 1);
    status_subscriber = this->create_subscription<uvs_message::msg::UvEmbStatus>("uvs_emb_status", 1, std::bind(&UvsManualKeyboard::status_callback, this, std::placeholders::_1));

    if (!keyboard->isFound())
    {
        printf("open failed.\n");
        return;
    }

    timer = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&UvsManualKeyboard::timer_callback, this));
}

UvsManualKeyboard::~UvsManualKeyboard()
{
}

void UvsManualKeyboard::status_callback(const uvs_message::msg::UvEmbStatus::SharedPtr msg)
{
    status_msg = *msg;
    getFeedback = true;
}

void UvsManualKeyboard::timer_callback()
{
    if (!getFeedback)
    {
        RCLCPP_INFO(get_logger(), "No feedback from embedded system.");
        return;
    }
    bool update = false;
    while (keyboard->sample(&keyboardEvent))
    {
        update = true;
    }
    if (update)
    {
        // linear speed
        kinetics_msg.v = (keyboardEvent.w ? LINEAR_SPEED : 0) - (keyboardEvent.s ? LINEAR_SPEED : 0);
        // angular speed
        kinetics_msg.w = (keyboardEvent.a ? ANGULAR_SPEED : 0) - (keyboardEvent.d ? ANGULAR_SPEED : 0);
        // arm base
        int delta_base = (keyboardEvent.u ? 1 : 0) - (keyboardEvent.o ? 1 : 0);
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
        int delta_arm = (keyboardEvent.j ? 1 : 0) - (keyboardEvent.l ? 1 : 0);
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
        // electromagnet
        emag_msg.enable = keyboardEvent.space ? 1 : 0;
        // publish
        arm_publisher->publish(arm_msg);
        emag_publisher->publish(emag_msg);
        kinetics_publisher->publish(kinetics_msg);
    }
}
