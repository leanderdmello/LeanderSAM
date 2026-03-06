#!/bin/bash

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <new_compose_project_id>"
    exit 1
fi

# Copy config from current deployment to new deployment
CURRENT_COMPOSE_PROJECT_ID=$(cat /opt/telerob/current)
CURRENT_CONFIG="/opt/telerob/deployment${CURRENT_COMPOSE_PROJECT_ID}/config/configuration.toml"
NEW_COMPOSE_PROJECT_ID="$1"
NEW_CONFIG="/opt/telerob/deployment${NEW_COMPOSE_PROJECT_ID}/config/configuration.toml"

# Assuming scripts are installed, as this is done earlier in the deployment process
MERGE_SCRIPT="/opt/telerob/system/merge-configuration-files.py"
VALIDATION_SCRIPT="/opt/telerob/system/validate-configuration-file.py"

if [ -f "$CURRENT_CONFIG" ]; then
    if ! python3 "$MERGE_SCRIPT" "$CURRENT_CONFIG" "$NEW_CONFIG" "/tmp/merged_config.toml.tmp"; then
        echo "Merging of old configuration failed"
        exit
    fi

    if ! python3 "$VALIDATION_SCRIPT" "/tmp/merged_config.toml.tmp"; then
        echo "Merged configuration is not valid, directly using new configuration"
    else
        cp "/tmp/merged_config.toml.tmp" "$NEW_CONFIG"
        echo "Merged ${CURRENT_CONFIG} into ${NEW_CONFIG}"
    fi
else
    echo "No existing configuration found at $CURRENT_CONFIG. Skipping copy."
fi
