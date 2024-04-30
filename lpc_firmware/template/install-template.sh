#!/bin/bash

cd -- "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1

template_dir="../template"

# SDK
cp -rl --remove-destination ${template_dir}/{CMSIS,LPC55S69,drivers,cmsis_drivers,utilities,components,startup,ld,Makefile,script.jlink.template,.clang-format,.clangd,.vscode} .

# SDMMC
cp -rl --remove-destination ${template_dir}/sdmmc .

# USB
cp -rl --remove-destination ${template_dir}/usb .

# FatFs
cp -rl --remove-destination ${template_dir}/fatfs .

# mbedtls
cp -rl --remove-destination ${template_dir}/mbedtls .

# freeRTOS
cp -rl --remove-destination ${template_dir}/freertos .

# user
cp -sf ${template_dir}/install-template.sh .
cp --no-clobber ${template_dir}/config.mk .
if [ ! -d source/ ]; then
    cp -r --no-clobber ${template_dir}/source .
fi

# development environment
cp -rl --remove-destination ${template_dir}/{.clang-format,.clangd,.vscode} .
