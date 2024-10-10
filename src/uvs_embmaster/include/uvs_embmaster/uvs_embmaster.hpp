#ifndef UVS_EMBMASTER_HPP
#define UVS_EMBMASTER_HPP

#include <rclcpp/rclcpp.hpp>

#include <iostream>

#include "uvs_embusb.hpp"
#include "uvs_message/msg/uv_emb_arm.hpp"
#include "uvs_message/msg/uv_emb_emag.hpp"
#include "uvs_message/msg/uv_emb_kinetics.hpp"
#include "uvs_message/msg/uv_emb_status.hpp"

#include "tmTask.hpp"
#include "tmQueue.hpp"
#include "tmSharedMem.hpp"

class UvEmbMaster : public rclcpp::Node
{
public:
    UvEmbMaster();
    ~UvEmbMaster();

    void init();
    void start();
    void spin_some();

private:
    rclcpp::Publisher<uvs_message::msg::UvEmbStatus>::SharedPtr pubStatus;
    rclcpp::Subscription<uvs_message::msg::UvEmbArm>::SharedPtr subArm;
    rclcpp::Subscription<uvs_message::msg::UvEmbEmag>::SharedPtr subEmag;
    rclcpp::Subscription<uvs_message::msg::UvEmbKinetics>::SharedPtr subKinetics;

    void armCallback(const uvs_message::msg::UvEmbArm::SharedPtr msg);
    void emagCallback(const uvs_message::msg::UvEmbEmag::SharedPtr msg);
    void kineticsCallback(const uvs_message::msg::UvEmbKinetics::SharedPtr msg);

    std::shared_ptr<UvEmbUsb> drv;
    std::future<void> futTx;
    std::future<void> futRx;
    bool upFlag = false;
};

#endif // UVS_EMBMASTER_HPP