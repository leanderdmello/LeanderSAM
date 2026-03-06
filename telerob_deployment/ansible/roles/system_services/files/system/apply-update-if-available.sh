#!/bin/bash

# Check whether update package is available on sd card
# If yes, copy it to the system and apply the update
# Remove the update package from sd card after application was successful

echo "---"
echo "Applying update from sd card device $1 if available..."

LED_SIGNAL_PATH="/var/opt/telerob/led-manager-update.signal"

# Get mount path of the sd card device
SD_CARD_PATH=/media/avular/sd-card

ls ${SD_CARD_PATH}

if [ $? != 0 ]; then
    echo "SD card device $1 is not mounted"
    echo "1" > "$LED_SIGNAL_PATH"  # Turn on LED on failure
    exit 1
fi

UPDATE_PACKAGE_PATH="$SD_CARD_PATH/update"

UPDATE_PACKAGE_FILE=$(find "$UPDATE_PACKAGE_PATH" -maxdepth 1 -type f -name '*.tar.gz' | head -n 1)

if [ -z "$UPDATE_PACKAGE_FILE" ]; then
    echo "No update package found on sd card at $UPDATE_PACKAGE_PATH"
    echo "0" > "$LED_SIGNAL_PATH"  # Do not turn on LED 
    exit 0
fi

echo "Update package found: $UPDATE_PACKAGE_FILE"

# Copy update package to system's /tmp directory
mkdir -p /tmp/update
cp "$UPDATE_PACKAGE_FILE" /tmp/update
if [ $? -ne 0 ]; then
    echo "Failed to copy update package to /tmp/"
    echo "1" > "$LED_SIGNAL_PATH"  # Turn on LED on failure
    exit 1
fi

# Apply the update
echo "Applying update package..."

# Signal: Update in progress (flashing LED)
echo "2" > "$LED_SIGNAL_PATH"
tar -xzf "/tmp/update/$(basename "$UPDATE_PACKAGE_FILE")" -C /tmp/update

if [ $? -ne 0 ]; then
    echo "Failed to extract update package"
    echo "1" > "$LED_SIGNAL_PATH"  # Turn on LED on failure
    exit 1
fi

# Run deploy script if it exists (which it should)
if [ -f /tmp/update/deploy.sh ]; then
    set -e
    set -x
    bash /tmp/update/deploy.sh local 2>&1 | tee "$UPDATE_PACKAGE_PATH/latest_update.log"
    if [ ${PIPESTATUS[0]} -ne 0 ]; then
        echo "Update deployment failed"
        echo "1" > "$LED_SIGNAL_PATH"  # Turn on LED on failure
        exit 1
    fi
    set +x
    set +e
else
    echo "No deploy.sh script found in update package"
    echo "1" > "$LED_SIGNAL_PATH"  # Turn on LED on failure
    exit 1
fi

echo "Update applied successfully"

# Signal: Update complete ( LED off)
echo "0" > "$LED_SIGNAL_PATH"

# Remove update package from sd card
rm -f "$UPDATE_PACKAGE_FILE"

# Clean up
rm -rf /tmp/update
