#!/bin/bash

# Set the target PID:VID
TARGET_PID="7020"
TARGET_VID="955"

# Iterate over all network interfaces
for interface in $(ls /sys/class/net); do
    # Check if the interface is a USB interface
    if [[ -d /sys/class/net/${interface}/device ]]; then
        # Get the USB device ID
        DEVICE_ID=$(cat /sys/class/net/${interface}/device/uevent | grep "PRODUCT=" | cut -d'=' -f2 | cut -d'/' -f2)
        VENDOR_ID=$(cat /sys/class/net/${interface}/device/uevent | grep "PRODUCT=" | cut -d'=' -f2 | cut -d'/' -f1)
        STATUS=$(cat /sys/class/net/${interface}/operstate)
        HAS_IP=$((ip addr show ${interface} | grep -q "inet ") && echo 1 || echo 0)

        # Apparently the status may not be up, but the device is still connected with an working IP address
        # Check if the device is a Jetson and it is up and has an IP address
        if [[ "${DEVICE_ID}" == "${TARGET_PID}" && "${VENDOR_ID}" == "${TARGET_VID}" && "$HAS_IP" == "1" ]]; then
            echo "${interface}"
            exit 0
        fi
    fi
done

echo "No Jetson connected" >&2
exit 1
