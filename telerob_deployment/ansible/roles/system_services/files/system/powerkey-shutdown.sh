#!/bin/bash

DEVICE="/dev/input/event0"


# Read power key configuration from currently deployed configuration file
current_deployment_suffix=$(cat /opt/telerob/current)
current_deployment_dir="/opt/telerob/deployment${current_deployment_suffix}"
config_file="${current_deployment_dir}/config/configuration.toml"
read_config_value() {
    local key="$1"
    local value=$(cat $config_file | tomlq -Mr "$key" 2>/dev/null | sed 's/null/false/')
    echo "$value"
}

save_on_hold=$(read_config_value .power.save_on_hold)

current_state="unknown"

process_current_button_state() {
    evtest "$DEVICE" --query EV_KEY KEY_POWER && handle_release || handle_press
}

enter_power_save() {
    if [ "$current_state" == "power_save" ]; then
        echo "Already in power save mode, skipping..."
        return
    fi
    echo "Entering power save mode..."

    # Set lidar to stand-by mode
    echo "Setting lidar to stand-by mode..."
    echo -e "set_config_param operating_mode STANDBY\nreinitialize\n" | nc -q 5 192.168.100.13 7501

    # Stop docker containers
    echo "Stopping docker containers..."
    telerob compose stop

    current_state="power_save"
}

exit_power_save() {
    if [ "$current_state" == "normal" ]; then
        echo "Already in normal mode, only restarting compose..."
        telerob compose restart
        return
    fi
    echo "Exiting power save mode..."

    # Set lidar to normal mode
    echo "Setting lidar to normal mode..."
    echo -e "set_config_param operating_mode NORMAL\nreinitialize\n" | nc -q 5 192.168.100.13 7501

    # Start docker containers
    echo "Starting docker containers..."
    telerob compose start

    current_state="normal"
}

ondemand_reload() {
    echo "Received ondemand reload signal"

    # Update save_on_hold
    save_on_hold=$(read_config_value .power.save_on_hold)

    # Retrigger current button state given the new configuration
    process_current_button_state
}

handle_press() {
    echo "Power key pressed"
    if [ "$save_on_hold" == "false" ]; then
        exit_power_save
    else
        enter_power_save
    fi
}

handle_release() {
    echo "Power key released"
    if [ "$save_on_hold" == "false" ]; then
        enter_power_save
    else
        exit_power_save
    fi
}


# Initial query on startup to exit power saving mode if the button is already held down
process_current_button_state

trap ondemand_reload SIGUSR1

while read line; do
    echo "$line" | grep -q "code 116 (KEY_POWER), value 1" && handle_press
    echo "$line" | grep -q "code 116 (KEY_POWER), value 0" && handle_release
done < <(evtest "$DEVICE")
