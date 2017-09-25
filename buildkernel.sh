#!/bin/bash

export ANDROID_VERSION=0
export ANDROID_MAJOR_VERSION=7

export CROSS_COMPILE=/home/brett/android/lineage/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin/aarch64-linux-android-

make j7elte_00_defconfig
make -j2