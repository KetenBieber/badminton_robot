# 设置基础镜像 Ubuntu16.04
FROM ubuntu:16.04 

# 注意这个文件修改的顺序，每次重新docker build 会从修改的地方从头到尾重新执行
# 因此需要将经常修改以及新添加的部分放置于文件末尾


# DEBIAN_FRONTEND=noninteractive,如此设置就可以直接运行命令，而无需向用户请求输入，这可以方便我们跳过apt-get install时频繁要求我们输入Y/N
ENV DEBIAN_FRONTEND=noninteractive \
  TZ=Etc/UTC \
  LANG=C.UTF-8 \
  LC_ALL=C.UTF-8

# 复制添加的软件源--即国内源
COPY apt/sources.list /etc/apt/

# 清空已有软件源
RUN apt-get clean && \
  apt-get autoclean

# 安装软件时，习惯性添加apt-get update 之后再install ，-y参数用于自动确认(Y/N)
# 并在末尾rm -rf /var/lib/apt/lists/* 这里存放了拉取的软件包索引都会缓存到/var/lib/apt/lists/ 下，这些文件在镜像运行时其实并不需要保留
RUN apt-get update && \
  apt-get install -y \
  curl \
  lsb-release \
  gnupg \
  gdb \
  && rm -rf /var/lib/apt/lists/* 

# 镜像中安装ros kinetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654


RUN apt-get update && \
  apt-get install -y \
  ros-kinetic-desktop-full \
  python-rosdep python-rosinstall python-rosinstall-generator python-wstool \
  python-catkin-tools \
  && rm -rf /var/lib/apt/lists/*


RUN apt update && \
  apt install -y \
  vim \
  htop \
  apt-utils \
  curl \
  cmake \
  net-tools

# 将ros系统路径导入到bashrc,每次打开终端就可以快速加载ros环境
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

# 安装x-server
RUN apt-get update && \
  apt-get install -y \
  libgl1-mesa-glx \
  libgl1-mesa-dri \
  libx11-6 \
  xauth \
  libglu1-mesa \
  mesa-utils \
  x11-xserver-utils && \
  rm -rf /var/lib/apt/lists/*

# 安装新版 CMake (>=3.10) 及常用工具
RUN apt-get update && \
  apt-get install -y \
  software-properties-common \
  wget \
  apt-transport-https \
  ca-certificates \
  gnupg && \
  # 添加 Kitware 仓库公钥与源
  wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | apt-key add - && \
  apt-add-repository "deb https://apt.kitware.com/ubuntu/ xenial main" && \
  apt-get update && \
  # 安装新版 cmake 及其他工具
  apt-get install -y \
  cmake \
  vim \
  htop \
  apt-utils \
  curl \
  net-tools && \
  rm -rf /var/lib/apt/lists/*

# 将ros所需要的东西也放在这里，免得重开容器需要重新安装
RUN apt-get update && \
  apt-get install -y \
  ros-kinetic-move-base 

WORKDIR /pathplanner_ws