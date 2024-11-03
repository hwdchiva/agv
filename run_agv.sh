#!/bin/bash

if [ $# -eq 0 ]; then
  ign launch agv_launch.ign
else
  ign launch actors_agv_launch.ign
fi
