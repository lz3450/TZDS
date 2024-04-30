#!/bin/bash

. /etc/os-release

if [[ $NAME == "Ubuntu" ]]; then
    sudo apt-get update
    sudo apt-get install -y \
        libmicrohttpd-dev \
        libsqlite3-dev \
        texinfo \
        bison \
        flex
fi
