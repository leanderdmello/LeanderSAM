#!/bin/bash

for port in 5000 5001; do
    systemctl restart telerob-udp-forwarder@${port}.service
done
