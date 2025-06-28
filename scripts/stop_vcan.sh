#!/bin/bash

set -euo pipefail

# List of custom vcan interfaces to remove
VCAN_INTERFACES=("can1" "can2")

for iface in "${VCAN_INTERFACES[@]}"; do
    if ip link show "$iface" &>/dev/null; then
        echo "Removed interface: $iface"
        sudo ip link set "$iface" down
        sudo ip link delete "$iface" type vcan
    else
        echo "Interface $iface not found. Skipping."
    fi
done
