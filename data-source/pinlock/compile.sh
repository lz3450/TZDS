#!/bin/bash

set -e

project="pinlock"

pushd ../../../lpc_firmware/$project > /dev/null
./install-template.sh
make clean > /dev/null
make COMPILER=clang -j16 > /dev/null
popd > /dev/null

source ~/Projects/SVF/setup.sh > /dev/null
ddfg ../../../lpc_firmware/pinlock/output/$project.ll > /dev/null
# opt -S turtle_tf2_listener.cpp.o.svf.bc -o turtle_tf2_listener.cpp.o.svf.ll
