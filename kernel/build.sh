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
make distclean

#配置defconfig文件
make stm32mp1_atk_defconfig

#开始编译内核和设备树
make uImage dtbs LOADADDR=0XC2000040 vmlinux -j16

#编译内核模块
make modules -j16

#在当前目录下亲新建一个tmp目录，用于存放编译后的目标文件
if [ ! -e "./tmp" ]; then
    mkdir tmp
fi
rm -rf tmp/*

make modules_install INSTALL_MOD_PATH=tmp

#删除source目录
rm -rf tmp/lib/modules/5.4.31/source

#删除build目录
rm -rf tmp/lib/modules/5.4.31/build

#裁剪模块的调试信息
find ./tmp -name "*.ko" | xargs $STRIP --strip-debug --remove-section=.comment --remove-section=.note --preserve-dates

cd tmp/lib/modules
tar -jcvf ../../modules.tar.bz2 .
cd -
rm -rf tmp/lib

#拷贝zImage到tmp目录下
cp arch/arm/boot/uImage tmp

#拷贝所有编译的设备树文件到当前的tmp目录下
cp arch/arm/boot/dts/stm32mp157d-atk*.dtb tmp
echo "编译完成，请查看当前目录下的tmp文件夹查看编译好的目标文件"
