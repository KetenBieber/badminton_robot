#!/usr/bin/env bash

# 这一行非常关键：告诉 MSYS2 不要做路径转换
export MSYS_NO_PATHCONV=1

xhost +local:root 1>/dev/null 2>&1
# 进入bash

docker exec \
    -u root \
    -it pathplanner_ws \
    /bin/bash
xhost -local:root 1>/dev/null 2>&1


