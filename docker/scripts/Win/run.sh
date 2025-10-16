#!/usr/bin/env bash
set -euxo pipefail

# MSYS2/Git-bash 下禁止自动路径转换
export MSYS_NO_PATHCONV=1
MONITOR_HOME_DIR="E:/pathplanner_ws"

if [ -z "${DISPLAY-}" ]; then
    display="host.docker.internal:0.0"
else
    display="${DISPLAY}"
fi

echo "MONITOR_HOME_DIR=${MONITOR_HOME_DIR}"
echo "DISPLAY=${display}"
echo "USER=${USER}, UID=$(id -u), GID=$(id -g)"

# 停止并删除旧容器（失败也继续）
docker stop pathplanner_ws  >/dev/null || true
docker rm -v -f pathplanner_ws >/dev/null || true

echo "start docker"
# -i 保持标准输入stdin打开，以便容器可以接收输入
# -t 分配一个伪终端TTY，可以看到输出并进行交互
# -d 让容器在后台运行，命令执行后你会得到一个容器 ID 而不是直接进入交互式终端
# --privileged=true 以root模式启动容器，赋予容器访问宿主更多设备和内核功能的权限
# --name badminton_controls 指定容器名称为badminton_controls，后续可以通过这个名字来停止、删除或执行命令
# -e "DISPLAY=${display}" 在容器内部设置环境变量DISPLAY，告诉X client 将图形输出发送到指定的 X server上
# --add-host host.docker.internal:host-gateway 在容器的 /etc/hosts 文件中添加一条，把 host.docker.internal 映射到宿主机的网关 IP，从而让 host.docker.internal 在容器内可解析到 Windows 主机
# -v "${MONITOR_HOME_DIR}:/work" 把宿主机的项目根目录挂载到容器的/work，容器里对 /work 的所有读写都映射回本地文件系统，实现代码和数据的实时同步
# -v /etc/localtime:/etc/localtime:ro \ 同步宿主机和容器的时钟
# pathplanner_ws:kinetic 里面预装了ROS Kinetic 以及你需要的依赖
docker run -it -d \
  --privileged=true \
  --name pathplanner_ws \
  -e "DISPLAY=${display}" \
  --add-host host.docker.internal:host-gateway \
  -v "${MONITOR_HOME_DIR}:/pathplanner_ws" \
  -v /etc/localtime:/etc/localtime:ro \
  pathplanner_ws:kinetic
