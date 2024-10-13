#include "uvs_embmaster.hpp"

#include <thread>
#include <future>
#include <chrono>

#include <sys/unistd.h>

#include "uvs_embusb.hpp"

tmQueue<std::vector<uint8_t>, 10> qUvTx;
tmQueue<std::vector<uint8_t>, 10> qUvRx;
tmSharedMem<bool> smUSBown;

class UvTxTask : public tmTask
{
public:
    UvTxTask()
        : tmTask("UvTxTask"){}
    ~UvTxTask(){}

    void run(const bool& runFlag) override
    {
        while(!stopRequested() && runFlag)
        {
            std::vector<uint8_t> data(64);
            if(qUvTx.get(data, std::chrono::milliseconds(1)))
            {
                if (smUSBown.acquire(std::chrono::milliseconds(1)))
                {
                    drv->write_regCh((void*)data.data(), data.size());
                    smUSBown.release();
                }
            }
        }
    }

    std::shared_ptr<UvEmbUsb> drv;
};

class UvRxTask : public tmTask
{
public:
    UvRxTask()
        : tmTask("UvRxTask"){}
    ~UvRxTask(){}

    void run(const bool& runFlag) override
    {
        while(!stopRequested() && runFlag)
        {
            std::vector<uint8_t> data(64);
            if (smUSBown.acquire(std::chrono::milliseconds(1)))
            {
                size_t len = drv->read_regCh((void*)data.data(), data.size());
                if (len > 0)
                {
                    data.resize(len);
                    qUvRx.put(data, std::chrono::milliseconds(1));
                }
                smUSBown.release();
            }

        }
    }

    std::shared_ptr<UvEmbUsb> drv;
};

UvTxTask txTask;
UvRxTask rxTask;

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
    futRx.wait();
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
    txTask.drv = drv;
    rxTask.drv = drv;
    futTx = std::async(std::launch::async, &UvTxTask::run, &txTask, std::ref(upFlag));
    futRx = std::async(std::launch::async, &UvRxTask::run, &rxTask, std::ref(upFlag));
}

void UvEmbMaster::spin_some()
{
    uvs_message::msg::UvEmbStatus msg;
    
    std::vector<uint8_t> data(64);
    if(qUvRx.get(data, std::chrono::milliseconds(0)))
    {
        ///@todo: read status from device
    }
    
    pubStatus->publish(msg);
}

void UvEmbMaster::armCallback(const uvs_message::msg::UvEmbArm::SharedPtr msg)
{
    std::vector<uint8_t> data(64);
    ///@todo: fill data with msg
    qUvTx.put(data, std::chrono::milliseconds(20));
}

void UvEmbMaster::emagCallback(const uvs_message::msg::UvEmbEmag::SharedPtr msg)
{
    std::vector<uint8_t> data(64);
    ///@todo: fill data with msg
    qUvTx.put(data, std::chrono::milliseconds(20));
}

void UvEmbMaster::kineticsCallback(const uvs_message::msg::UvEmbKinetics::SharedPtr msg)
{
    std::vector<uint8_t> data(64);
    ///@todo: fill data with msg
    qUvTx.put(data, std::chrono::milliseconds(20));
}
