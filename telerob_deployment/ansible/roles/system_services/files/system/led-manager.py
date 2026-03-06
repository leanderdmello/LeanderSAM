#!/usr/bin/env python3
import sys
import gpiod
import time
import os 

instance = sys.argv[1] if len(sys.argv) > 1 else sys.exit(1)
print(f"LED Manager instance: {instance}")

SIGNAL_FILE = f"/var/opt/telerob/led-manager-{instance}.signal"

chip_name = "gpiochip0"
blue_led_line_num = 85  # Blue LED line number corresponds to GPIO 12 on 40 pin header
red_led_line_number = 43  # Red LED line number corresponds to GPIO 13 on 40 pin header
flash_freq = 1  # hz
check_freq = 1  # hz

print("Setting up GPIOs:")
print(f"  Instance {instance}: Line {blue_led_line_num if instance == 'update' else red_led_line_number}: OUTPUT (LED {instance})")

with gpiod.Chip(chip_name) as chip:
    if instance == "update":
        led_line_num = blue_led_line_num
        led_line = chip.get_line(blue_led_line_num)
        led_line.request(consumer="led1", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
    elif instance == "in_use_sd_card":
        led_line_num = red_led_line_number
        led_line = chip.get_line(red_led_line_number)
        led_line.request(consumer="led2", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
    else:
        print("Unknown led GPIO select either 1 or 2")
        sys.exit(1)
    print("GPIO configured successfully")
    print(f"Monitoring {SIGNAL_FILE} and controlling LED-{instance}\n")
    led_state = False
    while True:
        if os.path.exists(SIGNAL_FILE):
            with open(SIGNAL_FILE, 'r') as f:
                command = f.read().strip()
                    
            # Command 0 corresponds to stop the led
            if command == "0":
                # If the led is already off we will sleep here to only update at "check_freq" rate
                if not led_state:
                    time.sleep(1/check_freq)
                    continue
                # Turn led off from an on state
                print(f"LED-{instance}: Turning OFF")
                led_state = False    
                led_line.set_value(0)
                
            # Command 1 corresponds to solid led
            elif command == "1":
                # If the led is already on we will sleep here to only update at "check_freq" rate
                if led_state:
                    time.sleep(1/check_freq)
                    continue
                # Turn led on from an off state
                print(f"LED-{instance}: Turning ON (solid)")
                led_state = True    
                led_line.set_value(1) 
            # Command 2 corresponds to flashing the led
            elif command == "2":
                led_state = not led_state
                led_line.set_value(1 if led_state else 0)
                print(f"LED-{instance}: Flashing {'ON' if led_state else 'OFF'}")
                time.sleep(1/flash_freq)
            else:
                print(f"LED-{instance}: Unknown command '{command}'")
                led_state = False    
                led_line.set_value(0)
                time.sleep(1/check_freq)
        else:
            print(f"Signal file not found at {SIGNAL_FILE}, waiting till it is available. Turning LED-{instance} to off state")
            led_state = False    
            led_line.set_value(0)
            time.sleep(1/check_freq)