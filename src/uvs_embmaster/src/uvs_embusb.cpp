#include "uvs_embusb.hpp"

#include "libusb-1.0/libusb.h"

#include <rclcpp/rclcpp.hpp>

#define USB_VID   0xCafe
#define USB_BCD   0x0200
#define USB_PID   0x4002

#define EPNUM_CDC_0_NOTIF   0x81
#define EPNUM_CDC_0_OUT     0x02
#define EPNUM_CDC_0_IN      0x82

#define EPNUM_CDC_1_NOTIF   0x83
#define EPNUM_CDC_1_OUT     0x04
#define EPNUM_CDC_1_IN      0x84

class UvEmbUsb_private
{
public:
    bool initDone = false;
    struct libusb_device_handle *devh = nullptr;
};

UvEmbUsb::UvEmbUsb()
    : impl(new UvEmbUsb_private)
{
}

UvEmbUsb::~UvEmbUsb()
{
    if (impl->devh != nullptr)
    {
        libusb_release_interface(impl->devh, 0);
        libusb_close(impl->devh);
        impl->devh = nullptr;
    }
    libusb_exit(NULL);
    delete impl;
}

bool UvEmbUsb::init()
{
    impl->initDone = false;
    if (impl->devh != nullptr)
    {
        libusb_release_interface(impl->devh, 0);
        libusb_close(impl->devh);
        impl->devh = nullptr;
    }

    int rc;
    rc = libusb_init(NULL);
    if (rc < 0) 
    {
        // std::cerr << "Failed to initialize libusb: " << libusb_error_name(rc) << std::endl;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to initialize libusb: %s", libusb_error_name(rc));
        return false;
    }
    impl->devh = libusb_open_device_with_vid_pid(NULL, USB_VID, USB_PID);
    if (impl->devh == NULL) 
    {
        // std::cerr << "Failed to open device" << std::endl;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open device");
        return false;
    }

    for (int if_num = 0; if_num < 4; if_num++) 
    {
        if (libusb_kernel_driver_active(impl->devh, if_num)) 
        {
            rc = libusb_detach_kernel_driver(impl->devh, if_num);
        }
        rc = libusb_claim_interface(impl->devh, if_num);
        if (rc < 0) 
        {
            // std::cerr << "Failed to claim interface " << if_num << ": " << libusb_error_name(rc) << std::endl;
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to claim interface %d: %s", if_num, libusb_error_name(rc));
            return false;
        }
    }

    impl->initDone = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "USB initialized");
    return true;
}

size_t UvEmbUsb::read_regCh(void *pdata, size_t size)
{
    if (impl->initDone)
    {
        int actual_length;
        int rc = libusb_bulk_transfer(impl->devh, EPNUM_CDC_0_IN, (uint8_t*)pdata, size, &actual_length, 10);
        return actual_length;
    }
    else
    {
        return 0;
    }
}

size_t UvEmbUsb::write_regCh(void *pdata, size_t size)
{
    if (impl->initDone)
    {
        int actual_length;
        int rc = libusb_bulk_transfer(impl->devh, EPNUM_CDC_0_OUT, (uint8_t*)pdata, size, &actual_length, 10);
        return actual_length;
    }
    else
    {
        return 0;
    }
}

size_t UvEmbUsb::read_burCh(void *pdata, size_t size)
{
    if (impl->initDone)
    {
        int actual_length;
        int rc = libusb_bulk_transfer(impl->devh, EPNUM_CDC_1_IN, (uint8_t*)pdata, size, &actual_length, 10);
        return actual_length;
    }
    else
    {
        return 0;
    }
}

size_t UvEmbUsb::write_burCh(void *pdata, size_t size)
{
    if (impl->initDone)
    {
        int actual_length;
        int rc = libusb_bulk_transfer(impl->devh, EPNUM_CDC_1_OUT, (uint8_t*)pdata, size, &actual_length, 10);
        return actual_length;
    }
    else
    {
        return 0;
    }
}
