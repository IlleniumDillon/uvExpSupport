#!/bin/bash

ROOT_DIR=$(pwd)

LIBUSB_INSTALL_DIR=$(pwd)/src/uvs_embmaster/libusb
LINUSB_VERSION=1.0.27
LIBUSB_URL=https://github.com/libusb/libusb.git

# install libusb
if [ ! -d uvs_embmaster ]; then
    git clone $LIBUSB_URL
    cd libusb
    git checkout v$LINUSB_VERSION
    ./autogen.sh
    ./configure --prefix=$LIBUSB_INSTALL_DIR
    make -j4
    # make install
    if [ ! -d $LIBUSB_INSTALL_DIR ]; then
        mkdir -p $LIBUSB_INSTALL_DIR
    fi
    cp libusb/libusb.h $LIBUSB_INSTALL_DIR
    cp libusb/.libs/libusb*.so $LIBUSB_INSTALL_DIR
    cp libusb/.libs/libusb*.a $LIBUSB_INSTALL_DIR
fi

# clean
cd $ROOT_DIR
rm -rf libusb

# install tinyusb rule
touch 99-tinyusb.rules
echo 'SUBSYSTEMS=="hidraw", KERNEL=="hidraw*", MODE="0666", GROUP="dialout"' >> 99-tinyusb.rules
echo 'ATTRS{idVendor}=="cafe", MODE="0666", GROUP="dialout"' >> 99-tinyusb.rules
echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="cafe", ENV{ID_MM_DEVICE_IGNORE}="1"' >> 99-tinyusb.rules
echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="03eb", ATTRS{idProduct}=="2111", MODE="0666", GROUP="users", ENV{ID_MM_DEVICE_IGNORE}="1"' >> 99-tinyusb.rules
echo 'SUBSYSTEMS=="tty", ATTRS{idVendor}=="03eb", ATTRS{idProduct}=="2111", MODE="0666", GROUP="users", ENV{ID_MM_DEVICE_IGNORE}="1"' >> 99-tinyusb.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1cbe", ATTRS{idProduct}=="00fd", MODE="0666"' >> 99-tinyusb.rules
sudo cp 99-tinyusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# clean
rm 99-tinyusb.rules
