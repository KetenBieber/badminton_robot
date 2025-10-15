#!/usr/bin/env bash
set -euxo pipefail

# 禁止 Git-bash/MSYS2 自动路径转换
export MSYS_NO_PATHCONV=1


# Linux或WSL2（Ubuntu）下，用Linux下文件系统的格式
# ${BASH_SOURCE[0]} 当前执行脚本的完整路径
MONITOR_HOME_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd -P)"

display="${DISPLAY:-host.docker.internal:0.0}"

echo "MONITOR_HOME_DIR=${MONITOR_HOME_DIR}"
echo "DISPLAY=${display}"
echo "USER=${USER}, UID=$(id -u), GID=$(id -g)"

docker stop badminton_controls >/dev/null 2>&1 || true
docker rm -v -f badminton_controls >/dev/null 2>&1 || true

echo "start docker"
docker run -d \
  --privileged \
  --name badminton_controls \
  -e DISPLAY="${display}" \
  --add-host host.docker.internal:host-gateway \
  -v "${MONITOR_HOME_DIR}:/pathplanner_ws" \
  badminton_controls:kinetic \
  tail -f /dev/null

echo ">>> container started, tailing logs"
