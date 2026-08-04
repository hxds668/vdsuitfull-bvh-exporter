sudo tee /usr/local/sbin/vdsuit-xr21x-rebind.sh >/dev/null <<'EOF'
#!/bin/sh

VID=04e2
PID=1410

modprobe xr_usb_serial_common 2>/dev/null || true
sleep 1

for intf in /sys/bus/usb/devices/*:*; do
    dev="${intf%:*}"

    [ -r "$dev/idVendor" ] || continue
    [ -r "$dev/idProduct" ] || continue

    [ "$(cat "$dev/idVendor")" = "$VID" ] || continue
    [ "$(cat "$dev/idProduct")" = "$PID" ] || continue

    iface="$(basename "$intf")"

    if [ -L "$intf/driver" ]; then
        drv="$(basename "$(readlink "$intf/driver")")"
        if [ "$drv" = "cdc_xr_usb_serial" ]; then
            continue
        fi
        echo -n "$iface" > "$intf/driver/unbind" 2>/dev/null || true
    fi

    sleep 0.2

    if [ -d /sys/bus/usb/drivers/cdc_xr_usb_serial ]; then
        echo -n "$iface" > /sys/bus/usb/drivers/cdc_xr_usb_serial/bind 2>/dev/null || true
    fi
done
EOF