# This Dockerfile downloads all required packages for offline installation
# It runs on the target architecture (arm64 Jetson) to ensure correct dependencies
FROM nvcr.io/nvidia/l4t-base:r36.2.0

# Create directories for packages
RUN mkdir -p /packages/apt /packages/pip

# Download apt packages
# These are the packages referenced in the Ansible playbook system_setup role
RUN apt-get update -o=dir::cache=/packages/apt && apt-get install -y --no-install-recommends \
    --download-only \
    -o=dir::cache=/packages/apt \
    apt \
    apt-utils \
    python3-libgpiod \
    gpiod \
    evtest \
    socat \
    ansible \
    jq \
    python3-pip \
    j2cli \
    tcpdump \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Install required tools for Docker setup
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    apt-utils \
    && rm -rf /var/lib/apt/lists/*

# Set up Docker repository
RUN curl -fsSL https://get.docker.com -o get-docker.sh \
    && sh get-docker.sh --setup-repo

# Also download docker and docker-cli (these come from NVIDIA's repos and have many dependencies)
RUN apt-get update -o=dir::cache=/packages/apt && apt-get install -y --no-install-recommends \
    --download-only \
    -o=dir::cache=/packages/apt \
    docker-ce=5:27.5* \
    docker-ce-cli=5:27.5* \
    --allow-downgrades \
    && rm -rf /var/lib/apt/lists/*

# Install pip separately to download packages
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip

# Download pip packages
RUN python3 -m pip download --dest /packages/pip yq toml

# Create a summary file
RUN echo "Offline packages for Jetson deployment" > /packages/MANIFEST.txt

# Output the packages directory
CMD ["tar", "-czf", "-", "-C", "/packages", "."]
