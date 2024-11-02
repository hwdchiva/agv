#!/bin/bash

cd ./obstacle_detection
mkdir -p ./build
cd ./build
cmake ..
make lidar_node