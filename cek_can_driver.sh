#!/bin/bash 

sudo ip link set can1 up type can bitrate 125000
sleep 5s
if ! timeout 5s candump can1 | grep -q 388; then
    echo "disconnect"
    poweroff
fi