#!/bin/bash

set -e

function usage() {
    echo "Usage: $0 (local|remote) [build] [ansible-args]"
    exit 1
}

if [ "$#" -lt 1 ]; then
    usage
fi

if [ "$1" != "local" ] && [ "$1" != "remote" ]; then
    usage
fi

host="$1"
shift

# Determine working directory
wdir=$(dirname "$0")

if [[ $1 == build* ]]; then
    build_arg="$1"
    shift
    make -C "$(dirname "$0")" ${build_arg}
    wdir="$(dirname $0)/build/flash_archive/"
fi

# Run ansible playbook
if ! command -v ansible >/dev/null 2>&1 || ! command -v jq >/dev/null 2>&1; then \
    echo "Installing ansible and jq..."
    sudo apt update && sudo apt install -y ansible jq; \
else \
    echo "Ansible and jq are already installed."; \
fi

cd ${wdir}/ansible
ansible-playbook -i hosts.yml main.yml --limit "$host" "$@"
