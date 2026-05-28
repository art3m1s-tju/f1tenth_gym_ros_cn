# syntax=docker/dockerfile:1

FROM ros:foxy

SHELL ["/bin/bash", "-c"]

# apt mirror: aliyun + disable HTTP cache to avoid hash mismatch behind CDN/proxy
RUN sed -i 's@//.*archive.ubuntu.com@//mirrors.aliyun.com@g; s@//security.ubuntu.com@//mirrors.aliyun.com@g' /etc/apt/sources.list && \
    echo 'Acquire::http::No-Cache "true";' > /etc/apt/apt.conf.d/99nocache && \
    echo 'Acquire::http::Pipeline-Depth "0";' >> /etc/apt/apt.conf.d/99nocache && \
    echo 'Acquire::BrokenProxy "true";' >> /etc/apt/apt.conf.d/99nocache && \
    echo 'Acquire::Retries "5";' >> /etc/apt/apt.conf.d/99nocache

# pip mirror: aliyun (HTTP to avoid SSL issues in Docker)
RUN mkdir -p /root/.pip && \
    printf '[global]\nindex-url = http://mirrors.aliyun.com/pypi/simple/\ntrusted-host = mirrors.aliyun.com\n' > /root/.pip/pip.conf

# system dependencies + scientific computing via apt (fast, no compilation)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git nano vim tmux \
        python3-pip python3-dev python3-setuptools python3-wheel python3-pytest \
        build-essential cmake \
        libeigen3-dev \
        python3-numpy python3-scipy python3-pandas python3-pil python3-matplotlib \
        ros-foxy-ament-copyright ros-foxy-ament-flake8 ros-foxy-ament-pep257 \
        ros-foxy-rviz2 && \
    rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get -y dist-upgrade && rm -rf /var/lib/apt/lists/*

# upgrade pip, then install only the packages not available via apt
RUN python3 -m pip install 'pip<24.1'
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install --prefer-binary --timeout 300 --retries 10 \
    transforms3d 'osqp==0.6.7' 'pandas>=1.3,<2' tqdm

# f1tenth gym
RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN cd f1tenth_gym && \
    python3 -m pip install -e .

# ros2 gym bridge
RUN mkdir -p sim_ws/src/f1tenth_gym_ros
COPY . /sim_ws/src/f1tenth_gym_ros
ENV PYTHONPATH="/sim_ws/src/f1tenth_gym_ros/code:${PYTHONPATH}"
RUN source /opt/ros/foxy/setup.bash && \
    cd sim_ws/ && \
    apt-get update && \
    rosdep install -i --from-path src --rosdistro foxy -y && \
    colcon build

WORKDIR '/sim_ws'
ENTRYPOINT ["/bin/bash"]
