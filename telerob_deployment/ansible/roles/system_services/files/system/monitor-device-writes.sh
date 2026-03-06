#!/bin/bash

DEVICE=$1

if [ ! -b "$DEVICE" ]; then
    echo "Error: $DEVICE is not a valid block device."
    exit 1
fi

LED_SIGNAL_PATH="/var/opt/telerob/led-manager-in_use_sd_card.signal"
CHECK_INTERVAL=1  # Check every second
INACTIVITY_THRESHOLD=2  # Seconds of inactivity before LED off

echo "0" > "$LED_SIGNAL_PATH"

# Get device name without /dev/
DEVICE_NAME=$(basename "$DEVICE")

get_write_sectors() {
    # Use grep to find the device in /proc/diskstats
    # If no result is found, return 0
    grep "$DEVICE_NAME" /proc/diskstats | awk '{print $7}' || echo "0" 
}

echo "Starting continuous monitoring of $DEVICE..."

LAST_WRITES=$(get_write_sectors)
LAST_ACTIVITY_TIME=$(date +%s)

while true; do
    sleep $CHECK_INTERVAL
    
    CURRENT_WRITES=$(get_write_sectors)
    CURRENT_TIME=$(date +%s)
    
    if [ "$CURRENT_WRITES" != "$LAST_WRITES" ]; then
        # Write activity detected
        echo "Write activity detected on $DEVICE"
        echo "2" > "$LED_SIGNAL_PATH"
        LAST_WRITES=$CURRENT_WRITES
        LAST_ACTIVITY_TIME=$CURRENT_TIME
    else
        # No write activity
        TIME_SINCE_ACTIVITY=$((CURRENT_TIME - LAST_ACTIVITY_TIME))
        
        if [ $TIME_SINCE_ACTIVITY -ge $INACTIVITY_THRESHOLD ]; then
            echo "0" > "$LED_SIGNAL_PATH"
        fi
    fi
done