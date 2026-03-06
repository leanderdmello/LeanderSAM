#!/bin/bash

echo "---"
echo "Updating configuration file from sd card device $1 if available..."

# Get mount path of the sd card device
SD_CARD_PATH=/media/avular/sd-card

ls ${SD_CARD_PATH}

if [ $? != 0 ]; then
    echo "SD card device $1 is not mounted"
    exit 1
fi

CONFIG_FILE_PATH="$SD_CARD_PATH/config"

CONFIG_FILE=$(find "$CONFIG_FILE_PATH" -maxdepth 1 -type f -name '*.toml' | head -n 1)

if [ -z "$CONFIG_FILE" ]; then
    echo "No configuration file found on sd card at $CONFIG_FILE_PATH"
    exit 1
fi

echo "Configuration file found: $CONFIG_FILE"

CURRENT_DEPLOYMENT_MARKER_FILE="/opt/telerob/current"
if [ ! -f "$CURRENT_DEPLOYMENT_MARKER_FILE" ]; then
    echo "Current deployment marker file not found at $CURRENT_DEPLOYMENT_MARKER_FILE"
    exit 1
fi
CURRENT_DEPLOYMENT_DIR="/opt/telerob/deployment$(cat $CURRENT_DEPLOYMENT_MARKER_FILE)"
if [ ! -d "$CURRENT_DEPLOYMENT_DIR" ]; then
    echo "Current deployment directory not found at $CURRENT_DEPLOYMENT_DIR"
    exit 1
fi

validate_configuration() {
    local file="$1"
    if [ ! -f "$file" ]; then
        echo "ERROR: File not found at $file"
        exit 1
    fi
    VALIDATOR_SCRIPT="$(dirname "$0")/validate-configuration-file.py"
    SCHEMA_FILE="$(dirname "$0")/configuration-schema.toml"
    if [ ! -f "$VALIDATOR_SCRIPT" ]; then
        echo "ERROR: Configuration validator script not found at $VALIDATOR_SCRIPT"
        exit 1
    fi
    echo "Validating configuration file..."
    if [ -f "$SCHEMA_FILE" ]; then
        python3 "$VALIDATOR_SCRIPT" "$file" "$SCHEMA_FILE"
    else
        python3 "$VALIDATOR_SCRIPT" "$file"
    fi
    if [ $? -ne 0 ]; then
        echo "Configuration file validation failed. Aborting operation."
        exit 1
    fi
    echo "Configuration file validation passed."
}

# Merge incoming configuration with priority to allow for incomplete incoming; as OLD in merge script
CURRENT_CONFIG_FILE="${CURRENT_DEPLOYMENT_DIR}/config/configuration.toml"
if [ -f "$CURRENT_CONFIG_FILE" ]; then
    MERGE_SCRIPT="/opt/telerob/system/merge-configuration-files.py"
    if [ ! -f "$MERGE_SCRIPT" ]; then
        echo "ERROR: Configuration merge script not found at $MERGE_SCRIPT"
        exit 1
    fi
    echo "Merging incoming configuration with existing configuration at $CURRENT_CONFIG_FILE"
    if ! python3 "$MERGE_SCRIPT" "$CONFIG_FILE" "$CURRENT_CONFIG_FILE" /tmp/merged_config.toml.tmp; then
        echo "ERROR: Configuration merge failed"
        exit 1
    fi
    # Validate merged configuration file
    validate_configuration /tmp/merged_config.toml.tmp
    # If validation passed, use merged configuration
    cp -f /tmp/merged_config.toml.tmp "$CURRENT_CONFIG_FILE"
else
    echo "No existing configuration file found at $CURRENT_CONFIG_FILE. Using incoming configuration directly."
    # Validate incoming configuration file
    validate_configuration "$CONFIG_FILE"
    cp -f "$CONFIG_FILE" "$CURRENT_CONFIG_FILE"
fi

# Remove configuration file from sd card
rm -f "$CONFIG_FILE"
