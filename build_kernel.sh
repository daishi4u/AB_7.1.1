#!/bin/bash
export CROSS_COMPILE=../aarch64-linux-android-4.9/bin/aarch64-linux-gnu-

#export CROSS_COMPILE=/opt/toolchains/arm-eabi-4.9/bin/aarch64-linux-android-
export ARCH=arm64

make j7elte_00_defconfig
make -j64
#24 2>&1 | tee -a  log.txt
