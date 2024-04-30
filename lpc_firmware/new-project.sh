#!/bin/bash

set -e

if [ $# -ne 1 ]; then
    echo "Usage: ./new-project.sh <project_name>"
    exit 1
fi

project_name="$1"
mkdir -p $project_name
ln -rsf template/install-template.sh $project_name
cd $project_name
./install-template.sh
