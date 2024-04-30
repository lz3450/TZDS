#!/bin/bash

sdk_dir="../SDK"

# CMSIS
cp -rl --remove-destination $sdk_dir/CMSIS .

# LPC55S69
mkdir -p LPC55S69
cp -rl --remove-destination $sdk_dir/devices/LPC55S69/*.c LPC55S69
cp -rl --remove-destination $sdk_dir/devices/LPC55S69/*.h LPC55S69

# drivers
cp -rl --remove-destination $sdk_dir/devices/LPC55S69/drivers .

# cmsis_drivers
cp -rl --remove-destination $sdk_dir/devices/LPC55S69/cmsis_drivers .

# components
cp -rl --remove-destination $sdk_dir/components .

# utilities
cp -rl --remove-destination $sdk_dir/devices/LPC55S69/utilities .
rm utilities/debug_console/fsl_debug_console_conf.h

# sdmmc
cp -rl --remove-destination $sdk_dir/middleware/sdmmc .
rm sdmmc/host/sdif/ChangeLogKSDK.txt
rm sdmmc/mmc/ChangeLogKSDK.txt
rm sdmmc/sd/ChangeLogKSDK.txt
rm sdmmc/sdio/ChangeLogKSDK.txt

# USB
cp -rl --remove-destination $sdk_dir/middleware/usb .
rm usb/ChangeLogKSDK.txt

# FatFs
mkdir -p fatfs
cp -rl --remove-destination $sdk_dir/middleware/fatfs/source fatfs/
cp -rl --remove-destination $sdk_dir/middleware/fatfs/template fatfs/

# mbedtls
mkdir -p mbedtls
cp -rl --remove-destination $sdk_dir/middleware/mbedtls/include mbedtls/
cp -rl --remove-destination $sdk_dir/middleware/mbedtls/library mbedtls/
cp -rl --remove-destination $sdk_dir/middleware/mbedtls/port mbedtls/
rm mbedtls/include/.gitignore
rm mbedtls/include/CMakeLists.txt
rm mbedtls/library/.gitignore
rm mbedtls/library/CMakeLists.txt
rm mbedtls/library/Makefile

# freeRTOS
cp -rl --remove-destination $sdk_dir/rtos/freertos/freertos-kernel/* freertos
cp -rl --remove-destination freertos/template/ARM_CM33_3_priority_bits/FreeRTOSConfig.h source/FreeRTOSConfig.h
rm -r freertos/template/ARM_CM33_3_priority_bits
rm -r freertos/portable/{ARMClang,ARMv8M,readme.txt}
rm freertos/{ChangeLogKSDK.txt,GitHub-FreeRTOS-Kernel-Home.url,History.txt,LICENSE.md,Quick_Start_Guide.url,README.md}

# startup
mkdir -p startup
cp -rl --remove-destination $sdk_dir/devices/LPC55S69/gcc/*.S startup

# ld
mkdir -p ld
cp -rl --remove-destination $sdk_dir/devices/LPC55S69/gcc/*.ld ld

find . -name "*.cmake" -exec rm {} \;
