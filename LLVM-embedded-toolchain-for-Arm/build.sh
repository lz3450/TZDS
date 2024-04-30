#!/bin/bash

. ./setup.sh

. /etc/os-release

if [[ $NAME == "Ubuntu" ]]; then
    build.py \
        --verbose \
        --revision "branch-14" \
        --variants armv8m.main_hard_cortex-m33_fpv5-sp-d16 \
        --host-toolchain clang \
        --native-toolchain clang \
        --use-ninja \
        --build-mode reconfigure > build.log 2>&1
fi
