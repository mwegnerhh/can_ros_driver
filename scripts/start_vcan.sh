#!/bin/bash

# Exit on error, treat unset variables as errors, and catch failures in pipelines
set -euo pipefail

# Function to check if a vcan interface exists
function interface_exists() {
    ip link show "$1" &> /dev/null
}

echo "Loading vcan module..."
if ! lsmod | grep -q "^vcan"; then
    sudo modprobe vcan
else
    echo "vcan module already loaded."
fi

for iface in can1 can2; do
    echo "Setting up interface: $iface"

    if interface_exists "$iface"; then
        echo "Interface $iface already exists. Skipping creation."
    else
        sudo ip link add dev "$iface" type vcan
    fi

    sudo ip link set "$iface" up
    echo "$iface is up."
done