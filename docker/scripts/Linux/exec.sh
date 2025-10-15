#!/usr/bin/env bash


xhost +local:root 1>/dev/null 2>&1
# 进入bash

docker exec \
    -u root \
    -it pathplanner_ws \
    /bin/bash
xhost -local:root 1>/dev/null 2>&1


