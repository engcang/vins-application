#!/bin/bash
LIBREALSENSE_VERSION=v2.21.0

cd ~/
git clone https://github.com/jetsonhacks/buildLibrealsense2Xavier
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense && git checkout $LIBREALSENSE_VERSION
cd ~/

sudo apt-add-repository universe
sudo apt-get update
sudo apt-get install pkg-config -y

cd /usr/src
sudo wget -N https://developer.nvidia.com/embedded/dlc/l4t-sources-32-1-JAX-TX2
sudo tar -xvf JAX-TX2-public_sources.tbz2
sudo tar -xvf public_sources/kernel_src.tbz2

sudo rm -r JAX-TX2-public_sources.tbz2
sudo rm -r public_sources
cd kernel/kernel-4.9

#sudo -s
zcat /proc/config.gz > .config
#exit

##################

cd /usr/src/kernel/kernel-4.9
sudo bash scripts/config --file .config \
	--set-str LOCALVERSION 4.9.140-tegra \
        --module HID_SENSOR_IIO_COMMON \
        --module HID_SENSOR_ACCEL_3D \
	--module HID_SENSOR_GYRO_3D

yes "" | sudo make olddefconfig

#####################

cd /usr/src/kernel/kernel-4.9
sudo patch -p1 < ~/buildLibrealsense2Xavier/patches/realsense-camera-formats_ubuntu-bionic-Xavier-4.9.108.patch
sudo patch -p1 < ~/buildLibrealsense2Xavier/patches/realsense-metadata-ubuntu-bionic-Xavier-4.9.108.patch ##error, ignored
## patch should be done manually, from line 220~ from uvcvideo.h file
## gedit /usr/src/kernel/kernel-4.9/drivers/media/usb/uvc/uvcvideo.h
## #define UVC_MAX_STATUS_SIZE	32
## #define UVC_QUIRK_APPEND_UVC_HEADER	0x00001000
## #define UVC_QUIRK_DISABLE_AUTOSUSPEND	0x00001200
sudo patch -p1 < ~/buildLibrealsense2Xavier/patches/realsense-hid-ubuntu-bionic-Xavier-4.9.108.patch

sudo patch -p1 < ~/librealsense/scripts/realsense-powerlinefrequency-control-fix.patch

###################

cd /usr/src/kernel/kernel-4.9
sudo make prepare
sudo make modules_prepare
time sudo make -j4 Image
time sudo make -j4 modules
 # if failed, try without -j4

sudo make modules_install

##################

cd /usr/src/kernel/kernel-4.9
mkdir -p ~/buildLibrealsense2Xavier/image
sudo cp /usr/src/kernel/kernel-4.9/arch/arm64/boot/Image ./image/Image


##################################################################################################


cd ~/librealsense

sudo apt-add-repository universe
sudo apt-get update

sudo apt-get install libssl-dev libusb-1.0-0-dev pkg-config -y
sudo apt-get install build-essential cmake cmake-curses-gui -y
sudo apt-get install libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev -y

sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

mkdir build && cd build
cmake ../ -DBUILD_EXAMPLES=true -DBUILD_WITH_CUDA=true
time sudo make -j4
sudo make install
