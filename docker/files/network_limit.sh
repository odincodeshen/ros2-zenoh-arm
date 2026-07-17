#!/bin/bash

# Note: this script was made with the help of https://chat.mistral.ai

# WiFi medium connection simulation parameters
TARGET_IP="172.1.0.3"     # Target IP (ROS container)
RATE="25mbit"             # Bandwidth: 25 Mbit/s (typical for medium 2.4GHz WiFi)
LATENCY="20ms"            # Base latency: 20 ms
JITTER="10ms"             # Latency variation: ±10 ms
LOSS="0.5%"               # Packet loss: 0.5%
REORDER="1% 25%"          # Packet reordering: 1% of packets, 25% correlation
DUPLICATE="0.1%"          # Duplicates: 0.1%
CORRUPT="0.01%"           # Corruptions: 0.01%

apply_rules() {
    echo "Applying WiFi medium connection simulation to $TARGET_IP..."

    # 1. Create root HTB queue
    sudo tc qdisc add dev eth0 root handle 1: htb default 30

    # 2. Create a class for limited traffic
    sudo tc class add dev eth0 parent 1: classid 1:1 htb rate $RATE

    # 3. Add netem queue with WiFi-like characteristics
    sudo tc qdisc add dev eth0 parent 1:1 handle 10: \
        netem rate $RATE delay $LATENCY $JITTER loss $LOSS reorder $REORDER duplicate $DUPLICATE corrupt $CORRUPT

    # 4. Mark packets destined for TARGET_IP
    sudo iptables -t mangle -A OUTPUT -d $TARGET_IP -j CLASSIFY --set-class 1:1

    echo "WiFi medium connection simulation applied to $TARGET_IP:"
    echo "  - Rate: $RATE"
    echo "  - Latency: $LATENCY ± $JITTER"
    echo "  - Packet loss: $LOSS"
    echo "  - Reordering: $REORDER"
    echo "  - Duplicates: $DUPLICATE"
    echo "  - Corruptions: $CORRUPT"
}

cancel_rules() {
    echo "Removing all traffic shaping rules..."
    sudo tc qdisc del dev eth0 root 2>/dev/null
    sudo iptables -t mangle -F OUTPUT 2>/dev/null
    echo "All rules removed."
}

if [ "$1" == "cancel" ]; then
    cancel_rules
else
    apply_rules
fi
