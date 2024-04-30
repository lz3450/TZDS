#!/bin/bash

rm -rf src/{binutils,gcc,newlib,gdb}
rm -rf src/{gmp-*,mpfr-*,mpc-*,isl-*,expat-*,libiconv-*,zlib-*,elfutils-*}
rm -rf build-native install-native
rm -f build-prerequisites.log build-toolchain.log install-sources.log build.log
