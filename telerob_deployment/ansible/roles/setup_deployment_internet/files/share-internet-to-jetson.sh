#!/bin/bash

# This script configures the host machine to share its internet connection with a Jetson device.
# The script assumes that the Jetson device is connected to the host machine via USB.

set -e

SCRIPT_DIR="$( cd "$( dirname $(realpath "${BASH_SOURCE[0]}") )" &> /dev/null && pwd )"
source ${SCRIPT_DIR}/common.sh

# Check that we are running as root
if [ "$EUID" -ne 0 ]; then
    error "Please run as root"
    exit 1
fi

usage() {
    info "Usage: $0"
    info "  -h, --help: Display help"
    info "  -p, --password: Password for the Jetson device"
    info "  -i, --internet-interface: Internet interface to share with the Jetson device"
    info "  -j, --jetson-interface: Network interface for the Jetson device"
    info "  -u, --username: Username for the Jetson device, default: avular"
    info "  --jetson-ip: IP address of the Jetson device, default: 192.168.55.1"
    info "  --jetson-remote-interface: Remote interface on the Jetson device connected to this PC, default: l4tbr0"
    info "  --ap: Change the defaults to reflect if we are connected to the Jetson via AP"
    info "  --ethernet: Change the defaults to reflect if we are connected to the Jetson via Ethernet"
    exit 1
}

find_interface_in_subnet() {
    local subnet=$1
    local interface
    interface=$(ip -o -f inet addr show | awk '$4 ~ "'${subnet}'" {print $2}')
    echo ${interface}
}

# Parse arguments
JETSON_USERNAME="avular"
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -p|--password) JETSON_PASSWORD="$2"; shift ;;
        -i|--internet-interface) INTERNET_INTERFACE="$2"; shift ;;
        -j|--jetson-interface) JETSON_INTERFACE="$2"; shift ;;
        --jetson-ip) JETSON_IP="$2"; shift ;;
        --jetson-remote-interface) JETSON_REMOTE_INTERFACE="$2"; shift ;;
        --ap ) AP=true ;;
        --ethernet ) ETHERNET=true ;;
        -u|--username) JETSON_USERNAME="$2"; shift ;;
        -h|--help) usage ;;
        *) error "Unknown parameter passed: $1"; usage ;;
    esac
    shift
done

# Check if we change the defaults to reflect if we are connected to the Jetson via AP or Ethernet
if [ "${AP}" = true -a "${ETHERNET}" = true ]; then
    error "Please choose either --ap or --ethernet, not both"
    exit 1
fi

if [ "${AP}" = true ]; then
    JETSON_IP="192.168.192.1"
    JETSON_REMOTE_INTERFACE="ap0"
    JETSON_INTERFACE=$(find_interface_in_subnet 192.168.192)
elif [ "${ETHERNET}" = true ]; then
    JETSON_IP="192.168.100.11"
    JETSON_REMOTE_INTERFACE="eth0"
    JETSON_INTERFACE=$(find_interface_in_subnet 192.168.100)
fi

# Check if a Jetson device is connected and get the network interface for the Jetson device
if [ -z "${JETSON_INTERFACE}" ]; then
    JETSON_INTERFACE=$(${SCRIPT_DIR}/get-connected-jetson-interface.sh)
fi
info "Jetson interface: ${JETSON_INTERFACE}"

# Try to find the internet interface
if [ -z "${INTERNET_INTERFACE}" ]; then
    # Get the internet interface
    INTERNET_INTERFACE=$(ip route | grep --max-count=1 default | awk '{print $5}')
fi
info "Internet interface: ${INTERNET_INTERFACE}"

# Add default route and DNS server to the Jetson device
if [ -z "${JETSON_PASSWORD}" ]; then
    read -s -p "Enter the jetson password: " JETSON_PASSWORD
    echo ""
fi

# Set the default jetson IP, if not provided
if [ -z "${JETSON_IP}" ]; then
    info "Using default Jetson IP: 192.168.55.1"
    JETSON_IP="192.168.55.1"
fi

# Set the default jetson remote interface, if not provided
if [ -z "${JETSON_REMOTE_INTERFACE}" ]; then
    info "Using default Jetson remote interface: l4tbr0"
    JETSON_REMOTE_INTERFACE="l4tbr0"
fi

# Get our IP address on the jetson interface
HOST_IP=$(ip addr show ${JETSON_INTERFACE} | grep -oP 'inet \K[\d.]+')

# Get the DNS server for the Internet interface
# resolvectl will output a line like: Link 30 (enxac1a3de4af0f): 10.10.100.50 10.10.100.12
DNS_SERVER=$(resolvectl -i ${INTERNET_INTERFACE} -4 dns | cut -d ':' -f 2)
if [ -z "${DNS_SERVER}" ]; then
    error "Failed to get the DNS server for the internet interface"
    exit 1
fi

# Enable IP forwarding
info "Enabling IP forwarding"
sysctl -w net.ipv4.ip_forward=1 > /dev/null

# Remove existing forwarding rules
info "Removing existing forwarding rules"
iptables -t nat -D POSTROUTING -o ${INTERNET_INTERFACE} -j MASQUERADE 2> /dev/null || true
iptables -D FORWARD -i ${INTERNET_INTERFACE} -o ${JETSON_INTERFACE} -m state --state RELATED,ESTABLISHED -j ACCEPT 2> /dev/null || true
iptables -D FORWARD -i ${JETSON_INTERFACE} -o ${INTERNET_INTERFACE} -j ACCEPT 2> /dev/null || true

# Setup Forwarding rules
info "Setting up forwarding rules"
iptables -t nat -A POSTROUTING -o ${INTERNET_INTERFACE} -j MASQUERADE
iptables -A FORWARD -i ${INTERNET_INTERFACE} -o ${JETSON_INTERFACE} -m state --state RELATED,ESTABLISHED -j ACCEPT
iptables -A FORWARD -i ${JETSON_INTERFACE} -o ${INTERNET_INTERFACE} -j ACCEPT

info "Adding default route to the Jetson device"
sshpass -p "${JETSON_PASSWORD}" ssh -t -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null ${JETSON_USERNAME}@${JETSON_IP} \
    "echo \"${JETSON_PASSWORD}\" | sudo -S -p ' ' bash -c \"ip route add default via ${HOST_IP} dev ${JETSON_REMOTE_INTERFACE} metric 150 ; \
        resolvectl dns ${JETSON_REMOTE_INTERFACE} ${DNS_SERVER} ; \
        resolvectl domain ${JETSON_REMOTE_INTERFACE} '~.' ; \
        resolvectl default-route ${JETSON_REMOTE_INTERFACE} yes ; \
        echo 'Success'\""
