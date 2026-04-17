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

# ros ${ROS_DISTRO} dev

USER root

# Install GitHub CLI repository
RUN curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg \
    && chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list > /dev/null

# Install ROS development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Misc
    python3-apt python3-pip sudo git gh \
    # Cross building
    gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
    mmdebstrap bdebstrap qemu-user-static \
    # Packaging
    binfmt-support dpkg-dev file \
    # Build tools
    cmake build-essential ninja-build make clang-15 clang-format-15 clang-tools-15 clang-tidy-15 \
    gcc-12 g++-12 \
    # ROS tools
    python3-colcon-common-extensions \
    ros-dev-tools \
    cmake-avular \
    get-dependencies \
    ros-${ROS_DISTRO}-ament-package \
    ros-${ROS_DISTRO}-ament-clang-format \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-auto \
    ros-${ROS_DISTRO}-ros2bag \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
    # Testing
    python3-pytest lcov ros-${ROS_DISTRO}-ros-gz python3-behave \
    # Docker
    && curl -fsSL https://get.docker.com | sh \
    && usermod -aG docker avular \
    # Set default versions
    # FIXME: There is a problem in gcc-12, where it fails to build googletest. For now, we will use gcc-11
    # && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 100 \
    # && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 100 \
    && update-alternatives --install /usr/bin/clang clang /usr/bin/clang-15 100 \
    && update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-15 100 \
    && update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-15 100 \
    && update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-15 100 \
    # Install python packages
    && pip3 install --no-cache-dir --upgrade \
    gcovr \
    # Cleanup
    && rm -rf /var/lib/apt/lists/*

# Add user to sudoers
RUN usermod -aG sudo avular \
    && echo "avular ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/avular \
    && chmod 0440 /etc/sudoers.d/avular

# Arguments
ARG USER="avular"
ARG UID=1000
ARG GID=1000
ARG RENDER_GID=110
ARG WS_DIR="/home/${USER}/ws"
ARG BUILD_CONTEXT="false"
ARG PACKAGES_FROM_DEVENV=""

# Set bash as the default shell
SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
    --mount=target=/var/cache/apt,type=cache,sharing=locked \
    apt-get update \
    && apt-get install -y --no-install-recommends \
    gosu ssh git-lfs neovim nano gdb openssh-client \
    python3-colcon-common-extensions python3-pip \
    rclcpp-avular postgresql-client clang-format \
    && rm -rf /var/lib/apt/lists/*

# Install avular-ci-tools
RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install avular-ci-tools -i https://packages.avular.dev/pypi/avular-pypi/simple

# Add GitHub to known SSH hosts
RUN ssh-keyscan github.com >> /etc/ssh/ssh_known_hosts

# Create Development workspace
RUN mkdir -p ${WS_DIR}/src ${WS_DIR}/context \
    && touch ${WS_DIR}/context/COLCON_IGNORE

# Include context files in workspace
# COPY . ${WS_DIR}/context/
RUN --mount=source=.,target=/tmp/context \
    # Copy selected packages from development environment if specified
    set -e; if [ ${PACKAGES_FROM_DEVENV} != "" ]; then \
    cat /tmp/context/.devcontainer/${PACKAGES_FROM_DEVENV}/ros_packages | \
    xargs -I {} cp -r /tmp/context/{} ${WS_DIR}/context/{}; \
    # Also copy other non-ros-package context folders (except for some explicit exclusions)
    find /tmp/context/ -mindepth 1 -maxdepth 1 -type d \
    ! -path /tmp/context/docker \
    ! -path /tmp/context/.devcontainer \
    ! -path /tmp/context/.vscode \
    ! -path /tmp/context/.github \
    ! -path /tmp/context/telerob_deployment \
    ! -exec bash -c 'grep -qx "$(basename "$1")" /tmp/context/.gitignore' _ {} \; \
    ! -exec bash -c '[ -f "$1/package.xml" ]' _ {} \; \
    -exec bash -c 'cp -r $1 ${WS_DIR}/context/$(basename "$1")' _ {} \;;\
    # Otherwise copy all relevant context folders, including all ROS packages
    else \
    find /tmp/context/ -mindepth 1 -maxdepth 1 -type d \
    ! -path /tmp/context/docker \
    ! -path /tmp/context/.devcontainer \
    ! -path /tmp/context/.vscode \
    ! -path /tmp/context/.github \
    ! -path /tmp/context/telerob_deployment \
    ! -exec bash -c 'grep -qx "$(basename "$1")" /tmp/context/.gitignore' _ {} \; \
    -exec bash -c 'cp -r $1 ${WS_DIR}/context/$(basename "$1")' _ {} \;;\
    fi

# Install system and Python dependencies for cloned repositories
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
    --mount=target=/var/cache/apt,type=cache,sharing=locked \
    set -e; apt-get update \
    && DEBIAN_DEPS=$(get-dependencies --dir ${WS_DIR}/context --prefix telerob- --print --print-version --separator "="  2>/dev/null); \
    if [ -n "$DEBIAN_DEPS" ]; then apt-get install -y --no-install-recommends $DEBIAN_DEPS; fi; \
    rm -rf /var/lib/apt/lists/*

RUN set -e; PIP_DEPS=$(get-dependencies --dir ${WS_DIR}/context --tag pip_depend --prefix telerob- --print --print-version --separator "==" 2>/dev/null); \
    if [ -n "$PIP_DEPS" ]; then python3 -m pip install $PIP_DEPS; \
    fi

# Change ownership of workspace to user
RUN chown -R ${UID}:${GID} ${WS_DIR}

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
ENV USER=${USER}

# Set Environment Variables
ENV TERM=xterm-256color
ENV CLICOLOR=1
ENV LSCOLORS=ExFxCxDxBxegedabagacad

# Force openGL to nvidia GPU instead of defaulting to intel mesa
ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia


# Configure EGL for headless rendering
ENV EGL_PLATFORM=surfaceless

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

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
fi

if [ -f "$HOME/ws/install/setup.bash" ]; then
    source $HOME/ws/install/setup.bash
fi
EOF
# >>>>>>> END HEREDOC MULTILINE STRING

# Build the workspace if it includes the context
RUN set -e; if [ "${BUILD_CONTEXT}" != "false" ] ; then \
    cd ${WS_DIR}/src; \
    find ../context/ -mindepth 1 -maxdepth 1 -type d \
    -exec bash -c 'ln -s $1 ./$(basename "$1")' _ {} \; ;\
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${WS_DIR} && \
    colcon build --parallel-workers 1"; \
    fi
