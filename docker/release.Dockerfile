# Base Image
FROM --platform=linux/amd64 ubuntu:jammy AS base_amd64
FROM --platform=linux/arm64 nvcr.io/nvidia/l4t-base:r36.2.0 AS base_arm64

FROM base_${TARGETPLATFORM##linux/}

ARG ROS_DISTRO=humble
ENV ROS_DISTRO=${ROS_DISTRO}

# configure APT to retry on failures
RUN echo 'APT::Acquire::Retries "3";' > /etc/apt/apt.conf.d/80-retries \
    && echo 'APT::Acquire::http::timeout "10";' >> /etc/apt/apt.conf.d/80-retries \
    && echo 'APT::Acquire::https::timeout "10";' >> /etc/apt/apt.conf.d/80-retries \
    && echo 'APT::Acquire::ftp::timeout "10";' >> /etc/apt/apt.conf.d/80-retries

# install dependencies to set up package repositories
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr gnupg2 wget ca-certificates curl python3-pip \
    && rm -rf /var/lib/apt/lists/*

# set up Avular apt repositories and pip repositories
ARG FEED_TYPE=stable
RUN curl https://packages.avular.dev/debian-feeds/development.pub | gpg --dearmor > /etc/apt/keyrings/avular-packages.gpg \
    && curl https://packages.avular.dev/debian-feeds/${ROS_DISTRO}-${FEED_TYPE}.pub | gpg --dearmor > /etc/apt/keyrings/avular-${FEED_TYPE}-packages.gpg \
    && curl https://archive.avular.dev/key.pub | gpg --dearmor > /etc/apt/keyrings/avular-archive.gpg \
    && chmod 644 /etc/apt/keyrings/avular-*.gpg \
    && export DEB_ARCH=$(dpkg --print-architecture) \
    # The priority is determined by the order of the sources.list entries. The first entry has the highest priority. \
    && echo "# Avular package repositories" > /etc/apt/sources.list.d/avular.list \
    && echo "deb [signed-by=/etc/apt/keyrings/avular-${FEED_TYPE}-packages.gpg arch=all,${DEB_ARCH}] https://packages.avular.dev ${ROS_DISTRO}-${FEED_TYPE} main" >> /etc/apt/sources.list.d/avular.list \
    && echo "deb [signed-by=/etc/apt/keyrings/avular-packages.gpg arch=all,${DEB_ARCH}] https://packages.avular.dev development main" >> /etc/apt/sources.list.d/avular.list \
    && echo "deb [signed-by=/etc/apt/keyrings/avular-archive.gpg] https://archive.avular.dev/${DEB_ARCH}/ jammy main" >> /etc/apt/sources.list.d/avular.list \
    && echo "deb [signed-by=/etc/apt/keyrings/avular-archive.gpg] https://archive.avular.dev/${DEB_ARCH}/ jammy-updates main" >> /etc/apt/sources.list.d/avular.list \
    && echo "deb [signed-by=/etc/apt/keyrings/avular-archive.gpg] https://archive.avular.dev/${DEB_ARCH}/ jammy-security main" >> /etc/apt/sources.list.d/avular.list \
    && echo "deb [signed-by=/etc/apt/keyrings/avular-archive.gpg] https://archive.avular.dev/ros/ jammy main" >> /etc/apt/sources.list.d/avular.list \
    && echo "" > /etc/apt/sources.list # remove default package repositories \
    && python3 -m pip config --global set global.index-url https://packages.avular.dev/pypi/avular-pypi/simple \
    && python3 -m pip config --global set global.trusted-host packages.avular.dev

# setup timezone
RUN echo 'tzdata tzdata/Areas select Europe' | debconf-set-selections && \
    echo 'tzdata tzdata/Zones/Europe select Amsterdam' | debconf-set-selections && \
    DEBIAN_FRONTEND="noninteractive" apt update && DEBIAN_FRONTEND="noninteractive" apt install -y tzdata && \
    rm -rf /var/lib/apt/lists/*

# Setup rosdep
RUN apt-get update && apt-get install -y --no-install-recommends python3-rosdep python3-apt \
    && rosdep init \
    && curl https://static-host.avular.dev/avular-dependencies/10-avular.list -o /etc/ros/rosdep/sources.list.d/10-avular.list \
    && rm -rf /var/lib/apt/lists/*

# Setup user
RUN useradd -ms /bin/bash avular
USER avular
WORKDIR /home/avular

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

CMD ["bash"]

#ros ${ROS_DISTRO}

USER root

# Add Avular pinfile
ADD --chown=root:root --chmod=664 avular-ros-${ROS_DISTRO}.pin /etc/apt/preferences.d/avular-ros-${ROS_DISTRO}-pin-600

# Install ROS core
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-core \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

# Set CycloneDDS as default RMW implementation
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Setup ROS user data
USER avular
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc


# Arguments
ARG USER="avular"
ARG UID=1000
ARG GID=1000
ARG RENDER_GID=110
ARG PACKAGES_FROM_DEVENV=""

# System Setup
USER root

# Set bash as the default shell
SHELL ["/bin/bash", "-c"]

# Install system dependencies
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
    --mount=target=/var/cache/apt,type=cache,sharing=locked \
    apt-get update \
    && apt-get install -y --no-install-recommends \
    gosu ssh \
    # Install bringup package of specified environment, which should also install environment packages
    telerob-${PACKAGES_FROM_DEVENV}-bringup \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN python3 -m pip install --upgrade pip; \
    # Extract all pip dependendencies from ROS packages
    pip_depends=$(grep -r "pip_depend" /opt/ros/${ROS_DISTRO}/ | awk -F'[<>]' '{print $3}' | sort | uniq); \
    if [ -n "$pip_depends" ]; then python3 -m pip install $pip_depends; fi

# Add render group, default to 110 if not provided
RUN getent group render || groupadd --gid $RENDER_GID render
RUN usermod -aG dialout,sudo,adm,video,audio,render ${USER}

# Add ROS entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# Set default working directory
WORKDIR /home/${USER}

# Change back to the default user
USER ${USER}

# Set Environment Variables
ENV TERM=xterm-256color
ENV CLICOLOR=1
ENV LSCOLORS=ExFxCxDxBxegedabagacad

# Force openGL to nvidia GPU instead of defaulting to intel mesa
ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia

# Create .local user directory
RUN mkdir -p /home/${USER}/.local/bin \
    && chown -R ${UID}:${GID} /home/${USER}/.local

# Update users .bashrc
# <<<<<<< START HEREDOC MULTILINE STRING
RUN cat <<'EOF' >> /home/${USER}/.bashrc

# Update local path
if [ -d "$HOME/bin" ] ; then
    PATH="$HOME/bin:$PATH"
fi

if [ -d "$HOME/.local/bin" ] ; then
    PATH="$HOME/.local/bin:$PATH"
fi

# Source ROS environment
export RCUTILS_COLORIZED_OUTPUT=1
source /opt/ros/$ROS_DISTRO/setup.bash
EOF
# >>>>>>> END HEREDOC MULTILINE STRING
