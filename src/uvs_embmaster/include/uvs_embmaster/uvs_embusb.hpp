
/**
 * @file uvs_embusb.hpp
 * @brief Header file for the UvEmbUsb class.
 *
 * This file contains the declaration of the UvEmbUsb class, which provides
 * methods for initializing the USB interface and performing read/write
 * operations on both register and burst channels.
 */

#ifndef UVS_EMBUSB_HPP
#define UVS_EMBUSB_HPP

#include <iostream>

class UvEmbUsb_private;
/**
 * @class UvEmbUsb
 * @brief A class for managing USB communication.
 *
 * The UvEmbUsb class provides an interface for initializing the USB
 * communication and performing read and write operations on register and
 * burst channels.
 */
class UvEmbUsb
{
public:
    /**
     * @brief Constructor for the UvEmbUsb class.
     *
     * Initializes a new instance of the UvEmbUsb class.
     */
    UvEmbUsb();
    /**
     * @brief Destructor for the UvEmbUsb class.
     *
     * Cleans up any resources used by the UvEmbUsb instance.
     */
    ~UvEmbUsb();
    /**
     * @brief Initializes the USB interface.
     *
     * @return true if initialization is successful, false otherwise.
     */
    bool init();
    /**
     * @brief Reads data from the register channel.
     *
     * @param pdata Pointer to the buffer where the read data will be stored.
     * @param size Size of the buffer in bytes.
     * @return The number of bytes read.
     */
    size_t read_regCh(void* pdata, size_t size);
    /**
     * @brief Writes data to the register channel.
     *
     * @param pdata Pointer to the buffer containing the data to be written.
     * @param size Size of the buffer in bytes.
     * @return The number of bytes written.
     */
    size_t write_regCh(void* pdata, size_t size);
    /**
     * @brief Reads data from the burst channel.
     *
     * @param pdata Pointer to the buffer where the read data will be stored.
     * @param size Size of the buffer in bytes.
     * @return The number of bytes read.
     */
    size_t read_burCh(void* pdata, size_t size);
    /**
     * @brief Writes data to the burst channel.
     *
     * @param pdata Pointer to the buffer containing the data to be written.
     * @param size Size of the buffer in bytes.
     * @return The number of bytes written.
     */
    size_t write_burCh(void* pdata, size_t size);
private:
    UvEmbUsb_private* impl;
};

#endif // UVS_EMBUSB_HPP