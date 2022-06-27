#!/bin/bash

#判断交叉编译工具链是否存在，使用arm-poky-linux-gnueabi- (gcc-9.3.0)
if [ ! -e "/opt/st/stm32mp1/3.1-snapshot/environment-setup-cortexa7t2hf-neon-vfpv4-ostl-linux-gnueabi" ]; then
    echo ""
    echo "请先安装正点原子的st-example-image-qtwayland-openstlinux-weston-stm32mp1-x86_64-toolchain-3.1-snapshot.sh"
    echo ""
fi

#使用Yocto SDK里的GCC 9.3.0交叉编译器编译出厂Linux源码,可不用指定ARCH等，直接执行Make
source /opt/st/stm32mp1/3.1-snapshot/environment-setup-cortexa7t2hf-neon-vfpv4-ostl-linux-gnueabi

#清除编译文件
if [ -e "../build" ]; then
    rm ../build -rf
fi

#开始编译tf-a-stm32mp157d-atk.stm32 
make -f ../Makefile.sdk  TFA_DEVICETREE=stm32mp157d-atk TF_A_CONFIG=trusted ELF_DEBUG_ENABLE='1' all
