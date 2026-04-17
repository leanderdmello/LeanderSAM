FROM ghcr.io/avular-robotics/telerob-slam-bringup-dev:nightly

USER root
SHELL ["/bin/bash", "-lc"]

RUN mkdir -p /var/lib/apt/lists/partial \
    && apt-get update \
    && apt-get install -y \
    curl \
    ca-certificates \
    xz-utils \
    ros-humble-ros2bag \
    ros-humble-rosbag2-storage-default-plugins \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://install.julialang.org | sh -s -- --yes --path /usr/local/juliaup \
    && /usr/local/juliaup/bin/juliaup add 1.12.5 \
    && /usr/local/juliaup/bin/juliaup default 1.12.5 \
    && ln -sf /usr/local/juliaup/bin/julia /usr/local/bin/julia \
    && ln -sf /usr/local/juliaup/bin/juliaup /usr/local/bin/juliaup

ENV JULIA_DEPOT_PATH=/usr/local/julia-depot
RUN mkdir -p /usr/local/julia-depot && chmod -R 777 /usr/local/julia-depot

RUN chown -R avular:avular /usr/local/juliaup /usr/local/julia-depot

USER avular