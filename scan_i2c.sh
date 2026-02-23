#!/bin/bash

echo "Scanning all I2C buses for connected devices..."
echo ""

found=0

for bus in 1 4 6 10 11 13 14; do
    result=$(i2cdetect -y "$bus" 2>/dev/null)
    # Check if any device address was found (non-'--' entries)
    devices=$(echo "$result" | grep -oE '[0-9a-f]{2}' | grep -v '^[0-9a-f]0$' | grep -vE '^(00|0[0-9a-f])$' | head -20)

    if echo "$result" | grep -qE ' [0-9a-fA-F]{2} '; then
        echo ">>> Bus $bus: DEVICE(S) FOUND <<<"
        echo "$result"
        found=1
    else
        echo "Bus $bus: nothing found"
    fi
    echo ""
done

if [ "$found" -eq 0 ]; then
    echo "No I2C devices detected on any bus."
    echo "Check your wiring: VCC->3.3V, GND->GND, SDA->GPIO2(pin3), SCL->GPIO3(pin5)"
fi
