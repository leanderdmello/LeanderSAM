#!/bin/bash

CHIP="gpiochip1"
LINE=25
SD_CARD_PATH="/media/avular/sd-card"

on_gpio_power_loss_imminent() {
  
    sync
    docker ps -q | xargs -r docker kill
    umount "$SD_CARD_PATH" || umount -l "$SD_CARD_PATH"
    echo "Succesfully handled emergency shutdown"
    exit 0 

}


echo "Monitoring GPIO line $LINE on $CHIP for power error events"

# Monitor GPIO line for RISING events
stdbuf -oL gpiomon "$CHIP" "$LINE" | while read -r event; do
    if echo "$event" | grep -q "FALLING"; then
        on_gpio_power_loss_imminent
    fi
done