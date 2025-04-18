FROM ubuntu:22.04

# Install dependencies  # rm -rf /var/lib/apt/lists/*  删除了 APT 包管理器的缓存文件
RUN apt-get update && \
    echo "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" && \
    # 添加ros2相关的源，从而可以安装python3-rocker
    # check for UTF-8
    locale && \
    echo "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb" && \
    apt install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    # verify settings 
    locale && \
    echo "ccccccccccccccccccccccccccccccccccccccc" && \
    apt install software-properties-common -y && \
    add-apt-repository universe && \
    apt update && \
    apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    # 安装autoware_carla_bridge的依赖
    apt-get update && \
    apt-get install -y \
    docker.io \
    python3-rocker && \
    sleep 2 \
    # 删除了 APT 包管理器的缓存文件
    && rm -rf /var/lib/apt/lists/*


RUN cd ~/ && \
    git clone https://github.com/evshary/autoware_carla_launch.git && \
    cd autoware_carla_launch/
#     ./container/run-bridge-docker.sh && \
#     echo "get into the bridge container" && \
#     cd autoware_carla_launch && \
#     source env.sh && \
#     echo "source env.sh successfully" && \
#     make prepare_bridge && \
#     echo "make prepare_bridge successfully" && \
#     make build_bridge && \
#     echo "make build_bridge successfully"


