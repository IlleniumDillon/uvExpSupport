#!/bin/bash

ROOT_DIR=$(pwd)
INSTALL_DIR=$(pwd)/src/uvs_tools/thirdparty

# INSTALL LIBUSB
LINUSB_VERSION=1.0.27
LIBUSB_URL=https://github.com/libusb/libusb.git
git clone $LIBUSB_URL
cd libusb
git checkout v$LINUSB_VERSION
./autogen.sh
./configure --prefix=$INSTALL_DIR
make -j8
make install

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
rm 99-tinyusb.rules

cd $ROOT_DIR
rm -rf libusb

# INSTALL JSONCPP
JSONCPP_VERSION=1.9.4
JSONCPP_URL=https://github.com/open-source-parsers/jsoncpp.git
git clone $JSONCPP_URL
cd jsoncpp
git checkout $JSONCPP_VERSION
mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR ..
make -j8
make install

cd $ROOT_DIR
rm -rf jsoncpp

# INSTALL TINYXML2
TINYXML2_VERSION=10.0.0
TINYXML2_URL=https://github.com/leethomason/tinyxml2.git
git clone $TINYXML2_URL
cd tinyxml2
git checkout $TINYXML2_VERSION
mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DCMAKE_CXX_FLAGS="-fPIC" ..
make -j8
make install

cd $ROOT_DIR
rm -rf tinyxml2

# INSTALL OSQP
OSQP_URL=https://github.com/osqp/osqp/releases/download/v0.6.3/osqp-v0.6.3-src.tar.gz
mkdir -p osqp
cd osqp
wget $OSQP_URL
tar -zxvf osqp-v0.6.3-src.tar.gz
mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR ..
make -j8
make install

cd $ROOT_DIR
rm -rf osqp

