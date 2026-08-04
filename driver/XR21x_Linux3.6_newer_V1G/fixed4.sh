sudo tee /etc/udev/rules.d/99-vdsuit-xr21x.rules >/dev/null <<'EOF'
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="04e2", ATTR{idProduct}=="1410", TAG+="systemd", ENV{SYSTEMD_WANTS}+="vdsuit-xr21x-rebind.service"
KERNEL=="ttyXRUSB[0-9]*", SUBSYSTEM=="tty", GROUP="dialout", MODE="0660"
EOF