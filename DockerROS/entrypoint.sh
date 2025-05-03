#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
xhost +

echo "Provided arguments: $@"


exec $@