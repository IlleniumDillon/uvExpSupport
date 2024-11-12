#!/bin/bash

colcon build --packages-select uvs_message
colcon build --packages-select uvs_tools
colcon build --packages-select uvs_optitrack uvs_mapserver uvs_launch
