#include "uvs_embmaster.hpp"

#include <thread>
#include <future>
#include <chrono>

#include <sys/unistd.h>

#include "protocol.h"
#include "uvs_embusb.hpp"

tmQueue<std::vector<uint8_t>, 10> qUvTxReg;
tmQueue<std::vector<uint8_t>, 10> qUvTxBur;

tmQueue<std::vector<uint8_t>, 10> qUvRxReg;
tmQueue<std::vector<uint8_t>, 10> qUvRxBur;

tmSharedMem<bool> smUSBown;

class UvTxRxTask : public tmTask
{
public:
    UvTxRxTask()
        : tmTask("UvTxRxTask"){}
    ~UvTxRxTask(){}

    void run(const bool& runFlag) override
    {
        while(!stopRequested() && runFlag)
        {
            std::vector<uint8_t> tdatar(64);
            if(qUvTxReg.get(tdatar, std::chrono::milliseconds(1)))
            {
                int ret = drv->write_regCh((void*)tdatar.data(), tdatar.size());
            }

            std::vector<uint8_t> tdatab(64);
            if(qUvTxBur.get(tdatab, std::chrono::milliseconds(1)))
            {  
                int ret = drv->write_burCh((void*)tdatab.data(), tdatab.size());
            }

            std::vector<uint8_t> rdatar(64);
            std::vector<uint8_t> rdatab(64);

            size_t lenr = drv->read_regCh((void*)rdatar.data(), rdatar.size());
            size_t lenb = drv->read_burCh((void*)rdatab.data(), rdatab.size());
            if (lenr > 0)
            {
                rdatar.resize(lenr);
                qUvRxReg.put(rdatar, std::chrono::milliseconds(1));
            }
            if (lenb > 0)
            {
                rdatab.resize(lenb);
                qUvRxBur.put(rdatab, std::chrono::milliseconds(1));
            }
        }
    }

    std::shared_ptr<UvEmbUsb> drv;
};

UvTxRxTask txrxTask;

UvEmbMaster::UvEmbMaster()
    : Node("uvs_embmaster")
{
    drv = std::make_shared<UvEmbUsb>();
    
    pubStatus = this->create_publisher<uvs_message::msg::UvEmbStatus>
        ("uvs_emb_status", 1);
    subArm = this->create_subscription<uvs_message::msg::UvEmbArm>
        ("uvs_emb_arm", 1, std::bind(&UvEmbMaster::armCallback, this, std::placeholders::_1));
    subEmag = this->create_subscription<uvs_message::msg::UvEmbEmag>
        ("uvs_emb_emag", 1, std::bind(&UvEmbMaster::emagCallback, this, std::placeholders::_1));
    subKinetics = this->create_subscription<uvs_message::msg::UvEmbKinetics>
        ("uvs_emb_kinetics", 1, std::bind(&UvEmbMaster::kineticsCallback, this, std::placeholders::_1));
    
}

UvEmbMaster::~UvEmbMaster()
{
    upFlag = false;
    futTx.wait();
    // futRx.wait();
}

void UvEmbMaster::init()
{
    while(!drv->init())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void UvEmbMaster::start()
{
    upFlag = true;
    txrxTask.drv = drv;
    futTx = std::async(std::launch::async, &UvTxRxTask::run, &txrxTask, std::ref(upFlag));
    // futRx = std::async(std::launch::async, &UvRxTask::run, &rxTask, std::ref(upFlag));
}

void UvEmbMaster::spin_some()
{
    uvs_message::msg::UvEmbStatus msg;
    
    std::vector<uint8_t> data(64);
    if(qUvRxReg.get(data, std::chrono::milliseconds(0)))
    {
        ///@todo: read status from device
        // RCLCPP_INFO(this->get_logger(), "Received data: %d", data.size());
        if (data.size() == sizeof(ComSDO_Reg))
        {
            ComSDO_Reg* reg = (ComSDO_Reg*)data.data();
            msg.voltage = udrift8_to_float(reg->voltage);
            msg.rotation_z = 0;
            msg.left_wheel_speed = drift16_to_float(reg->wheelSpeeds[0]);
            msg.right_wheel_speed = drift16_to_float(reg->wheelSpeeds[1]);
            msg.arm_base_pos = reg->armAngles[0];
            msg.arm_arm_pos = reg->armAngles[1];
            pubStatus->publish(msg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Received data with wrong size: %d", data.size());
        }
    }
    
    // pubStatus->publish(msg);
}

void UvEmbMaster::armCallback(const uvs_message::msg::UvEmbArm::SharedPtr msg)
{
    std::vector<uint8_t> data(sizeof(ComSDI_Bur));
    ///@todo: fill data with msg
    ComSDI_Bur* pdata = (ComSDI_Bur*)data.data();
    pdata->flag = COM_FLAG_ARM;
    pdata->armAngles[0] = msg->arm_base;
    pdata->armAngles[1] = msg->arm_arm;
    qUvTxBur.put(data, std::chrono::milliseconds(20));
    // RCLCPP_INFO(this->get_logger(), "Arm angles: %d, %d", msg->arm_base, msg->arm_arm);
}

void UvEmbMaster::emagCallback(const uvs_message::msg::UvEmbEmag::SharedPtr msg)
{
    std::vector<uint8_t> data(sizeof(ComSDI_Bur));
    ///@todo: fill data with msg
    ComSDI_Bur* pdata = (ComSDI_Bur*)data.data();
    pdata->flag = COM_FLAG_EMAG;
    pdata->emag = msg->enable;
    qUvTxBur.put(data, std::chrono::milliseconds(20));
    // RCLCPP_INFO(this->get_logger(), "Electromagnet: %d", msg->enable);
}

void UvEmbMaster::kineticsCallback(const uvs_message::msg::UvEmbKinetics::SharedPtr msg)
{
    std::vector<uint8_t> data(sizeof(ComSDI_Reg));
    ///@todo: fill data with msg
    ComSDI_Reg* pdata = (ComSDI_Reg*)data.data();
    pdata->linearVel = drift16_from_float(msg->v);
    pdata->angularVel = drift16_from_float(msg->w);
    qUvTxReg.put(data, std::chrono::milliseconds(20));
    // RCLCPP_INFO(this->get_logger(), "Linear speed: %f, Angular speed: %f", msg->v, msg->w);
}
