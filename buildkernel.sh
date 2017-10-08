#!/bin/bash

export ANDROID_VERSION=0
export ANDROID_MAJOR_VERSION=7

make j7elte_00_defconfig
make -j4 2> build.log
