#!/bin/bash

export CROSS_COMPILE=/home/android/Desktop/ndk/toolchains/aarch64-linux-android-4.9/prebuilt/linux-x86_64/bin/aarch64-linux-android-
export ARCH=arm64

make ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE O=out msm8952-tulip_defconfig
make ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE O=out -j3

mv out/arch/arm64/boot/Image.gz-dtb .

