
/**
 * @file uvs_embmaster.hpp
 * @brief Header file for the UvEmbMaster class.
 * 
 * This file contains the declaration of the UvEmbMaster class, which is responsible for
 * managing the communication with the embedded system via USB and handling various
 * ROS2 topics related to the embedded system's arm, electromagnet, and kinetics.
 */

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

/**
 * @class UvEmbMaster
 * @brief A class to manage communication with the embedded system and handle ROS2 topics.
 * 
 * The UvEmbMaster class inherits from rclcpp::Node and is responsible for initializing
 * the communication with the embedded system, starting the communication threads, and
 * handling the ROS2 topics for the arm, electromagnet, and kinetics.
 */
class UvEmbMaster : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the UvEmbMaster class.
     * 
     * Constructs a UvEmbMaster node with the specified name, and initializes the ROS2 topics.
     */
    UvEmbMaster();

    /**
     * @brief Destructor for the UvEmbMaster class.
     * 
     * Destructs the UvEmbMaster node. Waits for the transmission and reception threads to finish.
     */
    ~UvEmbMaster();

    /**
     * @brief Initialize the UvEmbMaster node.
     * 
     * This function initializes the USB driver, and try to connect to the embedded system.
     */
    void init();

    /**
     * @brief Start the communication with the embedded system.
     * 
     * Start the transmission and reception threads to handle the communication with the embedded system.
     */
    void start();

    /**
     * @brief Spin the node to process incoming messages.
     * 
     * Get data from embedded system and publish it to the ROS2 topics.
     */
    void spin_some();

private:
    rclcpp::Publisher<uvs_message::msg::UvEmbStatus>::SharedPtr pubStatus; ///< Publisher for the embedded system status.
    rclcpp::Subscription<uvs_message::msg::UvEmbArm>::SharedPtr subArm; ///< Subscription for the arm messages.
    rclcpp::Subscription<uvs_message::msg::UvEmbEmag>::SharedPtr subEmag; ///< Subscription for the electromagnet messages.
    rclcpp::Subscription<uvs_message::msg::UvEmbKinetics>::SharedPtr subKinetics; ///< Subscription for the kinetics messages.

    /**
     * @brief Callback function for the arm messages.
     * @param msg Shared pointer to the arm message.
     */
    void armCallback(const uvs_message::msg::UvEmbArm::SharedPtr msg);

    /**
     * @brief Callback function for the electromagnet messages.
     * @param msg Shared pointer to the electromagnet message.
     */
    void emagCallback(const uvs_message::msg::UvEmbEmag::SharedPtr msg);

    /**
     * @brief Callback function for the kinetics messages.
     * @param msg Shared pointer to the kinetics message.
     */
    void kineticsCallback(const uvs_message::msg::UvEmbKinetics::SharedPtr msg);

    std::shared_ptr<UvEmbUsb> drv; ///< Shared pointer to the USB driver.
    std::future<void> futTx; ///< Future for the transmission thread.
    std::future<void> futRx; ///< Future for the reception thread.
    bool upFlag = false; ///< Flag indicating if the communication is up.
};

#endif // UVS_EMBMASTER_HPP