#!/bin/bash

# Read currently deployed configuration file and process its contents
# Currently only updates network configuration for Telerob ethernet interface based on file contents

managed_interface="enP8p1s0"

echo "---"
echo "Processing currently deployed configuration file..."

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

CONFIG_FILE="$CURRENT_DEPLOYMENT_DIR/config/configuration.toml"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "Configuration file not found at $CONFIG_FILE"
    exit 1
fi

# Extract network configuration for Telerob ethernet interface
ip_address=$(cat $CONFIG_FILE | tomlq -Mr .network.address)
gateway=$(cat $CONFIG_FILE | tomlq -Mr .network.gateway)

if [ -z "$ip_address" ]; then
    echo "Failed to extract network configuration from $CONFIG_FILE"
    exit 1
fi

echo "Configuring network interface $managed_interface with:"
echo "  IP Address: $ip_address"
echo "  Gateway: $gateway"

# Apply network configuration using nmcli
connection_name=""
while read -r con; do
    iface=$(nmcli -g connection.interface-name con show "$con" 2>/dev/null)
    if [ "$iface" = "$managed_interface" ]; then
        connection_name="$con"
        break
    fi
done < <(nmcli -g NAME con show)

if [ -z "$connection_name" ]; then
    echo "Failed to find NetworkManager connection for interface $managed_interface"
    exit 1
fi

nmcli con mod "$connection_name" ipv4.addresses "$ip_address"
if [ -n "$gateway" ]; then
    nmcli con mod "$connection_name" ipv4.gateway "$gateway"
fi
nmcli con mod "$connection_name" ipv4.method manual
nmcli con up "$connection_name"

# Render parameter override templates with updated configuration
echo "Rendering parameter override templates with updated configuration..."
cd "$CURRENT_DEPLOYMENT_DIR/config/override_templates" || exit 1
[ -d /var/opt/telerob/overrides ] || mkdir /var/opt/telerob/overrides
template_files=$(find . -type f -name '*.yaml.j2')
parsed_config=$(cat "$CONFIG_FILE" | tomlq -M .)
for template_file in $template_files; do
    output_file="/var/opt/telerob/overrides/$(basename "$template_file" .j2)"
    echo "$parsed_config" | j2 -f json "$template_file" -o "$output_file"
done

# Restart powerkey-shutdown service to apply any changes in compose, respecting current power state
echo "Reloading powerkey service..."
sudo systemctl reload telerob-powerkey.service
